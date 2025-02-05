use core::future::Future;
use core::iter::zip;

use bincode::config;
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{Read, ReadExactError, Seek, SeekFrom};
use embedded_usb_pd::{Error as DeviceError, PdError};

use super::interrupt::InterruptController;
use crate::command::*;
use crate::fw_update::*;
use crate::{error, info, warn, PORT0};

/// Trait for updating the firmware of a target device
pub trait UpdateTarget: InterruptController {
    /// Enter FW update mode
    fn fw_update_mode_enter(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> impl Future<Output = Result<(), DeviceError<Self::BusError>>>;

    /// Start FW update
    fn fw_update_init(
        &mut self,
        delay: &mut impl DelayNs,
        args: &TfuiArgs,
    ) -> impl Future<Output = Result<ReturnValue, DeviceError<Self::BusError>>>;

    /// Abort FW update
    fn fw_update_mode_exit(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> impl Future<Output = Result<(), DeviceError<Self::BusError>>>;

    /// Validate the most recent block supplied to the device
    fn fw_update_validate_stream(
        &mut self,
        delay: &mut impl DelayNs,
        block_index: usize,
    ) -> impl Future<Output = Result<TfuqBlockStatus, DeviceError<Self::BusError>>>;

    /// Stream a block to the device
    fn fw_update_stream_data(
        &mut self,
        delay: &mut impl DelayNs,
        args: &TfudArgs,
    ) -> impl Future<Output = Result<(), DeviceError<Self::BusError>>>;

    /// Complete the FW update process
    fn fw_update_complete(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> impl Future<Output = Result<(), DeviceError<Self::BusError>>>;

    /// Write data to all supplied devices
    fn fw_update_burst_write(
        &mut self,
        address: u8,
        data: &[u8],
    ) -> impl Future<Output = Result<(), DeviceError<Self::BusError>>>;
}

/// Trait for anything that can be used as an image for firmware update
pub trait Image: Read + Seek {}

/// Error type for the firmware update process
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<T: UpdateTarget, I: Image> {
    Bus(T::BusError),
    Pd(PdError),
    Io(I::Error),
    ReadExact(ReadExactError<I::Error>),
}

impl<T: UpdateTarget, I: Image> From<DeviceError<T::BusError>> for Error<T, I> {
    fn from(e: DeviceError<T::BusError>) -> Self {
        match e {
            DeviceError::Bus(e) => Error::Bus(e),
            DeviceError::Pd(e) => Error::Pd(e),
        }
    }
}

/// Enter FW update mode on all controllers
async fn enter_fw_update_mode<T: UpdateTarget>(
    delay: &mut impl DelayNs,
    controllers: &mut [&mut T],
) -> Result<(), DeviceError<T::BusError>> {
    for (i, controller) in controllers.iter_mut().enumerate() {
        info!("Controller {}: Entering FW update mode", i);
        if let Err(e) = controller.fw_update_mode_enter(delay).await {
            info!("Controller {}: Failed to enter FW update mode", i);

            exit_fw_update_mode(delay, controllers).await?;
            return Err(e);
        }
    }

    Ok(())
}

/// Exit FW update mode on all controllers
async fn exit_fw_update_mode<T: UpdateTarget>(
    delay: &mut impl DelayNs,
    controllers: &mut [&mut T],
) -> Result<(), DeviceError<T::BusError>> {
    for (i, controller) in controllers.iter_mut().enumerate() {
        info!("Controller {}: Exiting FW update mode", i);
        if controller.fw_update_mode_exit(delay).await.is_err() {
            info!("Controller {}: Failed to exit FW update mode", i);
            // Don't return to allow the other controllers to exit FW update mode
        }
    }

    Ok(())
}

/// Helper function to read from an image at a specific offset
async fn read_from_exact<I: Image>(
    image: &mut I,
    offset: usize,
    buf: &mut [u8],
) -> Result<(), ReadExactError<I::Error>> {
    image
        .seek(SeekFrom::Start(offset as u64))
        .await
        .map_err(ReadExactError::Other)?;
    image.read_exact(buf).await
}

/// Initialize FW update on all controller
async fn fw_update_init<T: UpdateTarget, I: Image>(
    delay: &mut impl DelayNs,
    controllers: &mut [&mut T],
    image: &mut I,
) -> Result<TfuiArgs, Error<T, I>> {
    if controllers.is_empty() {
        return Err(Error::Pd(PdError::InvalidParams));
    }

    let mut arg_bytes = [0u8; HEADER_METADATA_LEN];

    // Get TFUi args from image header metadata
    read_from_exact(image, HEADER_METADATA_OFFSET, &mut arg_bytes)
        .await
        .map_err(Error::ReadExact)?;
    let (args, _) = bincode::decode_from_slice(&arg_bytes, config::standard().with_fixed_int_encoding())
        .map_err(|_| Error::Pd(PdError::Serialize))?;

    // Initialize FW update on all controllers
    for (i, controller) in controllers.iter_mut().enumerate() {
        info!("Controller {}: Initializing FW update", i);

        match controller.fw_update_init(delay, &args).await {
            Ok(ReturnValue::Success) => (),
            Ok(r) => {
                info!("Controller {}: Failed to initialize FW update, result {}", i, r);
                exit_fw_update_mode(delay, controllers).await?;
                return Err(Error::Pd(PdError::Failed));
            }
            Err(e) => {
                info!("Controller {}: Failed to initialize FW update", i);
                exit_fw_update_mode(delay, controllers).await?;
                return Err(e.into());
            }
        }
    }

    let mut buf = [0u8; BURST_WRITE_SIZE];
    image
        .seek(SeekFrom::Start(HEADER_BLOCK_OFFSET as u64))
        .await
        .map_err(Error::Io)?;

    // Supply the header block to all controllers
    info!("Broadcasting header block");
    for _ in 0..HEADER_BLOCK_LEN / BURST_WRITE_SIZE {
        image.read_exact(&mut buf).await.map_err(Error::ReadExact)?;
        // All the controllers share the same bus, so just use the first controller
        controllers[0]
            .fw_update_burst_write(args.broadcast_u16_address as u8, &buf)
            .await?;
    }

    delay.delay_ms(TFUI_BURST_WRITE_DELAY_MS).await;

    // Validate the block on all controllers
    for (i, controller) in controllers.iter_mut().enumerate() {
        info!("Controller {}: Validating header block", i);
        match controller.fw_update_validate_stream(delay, HEADER_BLOCK_INDEX).await {
            Ok(TfuqBlockStatus::HeaderValidAndAuthentic) => (),
            Ok(r) => {
                error!("Controller {}: Header block validation failed, result {}", i, r);
                exit_fw_update_mode(delay, controllers).await?;
                return Err(Error::Pd(PdError::Failed));
            }
            Err(_) => {
                error!("Controller {}: Header block validation failed", i);
                exit_fw_update_mode(delay, controllers).await?;
                return Err(Error::Pd(PdError::Failed));
            }
        }
    }

    Ok(args)
}

/// Computes the offset of a data block's metadata
const fn data_block_metadata_offset(block: usize) -> usize {
    HEADER_BLOCK_OFFSET + HEADER_BLOCK_LEN + (block * (DATA_BLOCK_LEN + DATA_BLOCK_METADATA_LEN))
}

/// Computes the offset of a data block's data
const fn block_offset(metadata_offset: usize) -> usize {
    metadata_offset + DATA_BLOCK_METADATA_LEN
}

/// Computes the offset of the app config block's metadata
const fn app_config_block_metadata_offset(num_data_blocks: usize, app_size: usize) -> usize {
    app_size + IMAGE_ID_LEN + HEADER_METADATA_LEN + HEADER_BLOCK_LEN + num_data_blocks * DATA_BLOCK_METADATA_LEN
}

/// Get the size of the image
async fn get_image_size<I: Image>(image: &mut I) -> Result<usize, ReadExactError<I::Error>> {
    let mut image_size_data = [0; 4];
    read_from_exact(image, APP_IMAGE_SIZE_OFFSET, &mut image_size_data).await?;
    Ok(u32::from_le_bytes(image_size_data) as usize)
}

/// Converts a data block into a block index
fn data_block_index_to_block_index(block_index: usize) -> usize {
    block_index + DATA_BLOCK_START_INDEX
}

/// Stream and validate a block of data to all controllers
async fn fw_update_stream_data<T: UpdateTarget, I: Image>(
    delay: &mut impl DelayNs,
    controllers: &mut [&mut T],
    image: &mut I,
    block_index: usize,
    metadata_offset: usize,
    metadata_size: usize,
) -> Result<(), Error<T, I>> {
    if controllers.is_empty() {
        return Err(Error::Pd(PdError::InvalidParams));
    }

    let mut arg_bytes = [0u8; MAX_METADATA_LEN];

    // Get TFUd args from image
    read_from_exact(image, metadata_offset, &mut arg_bytes[..metadata_size])
        .await
        .map_err(Error::ReadExact)?;

    let (args, _) = bincode::decode_from_slice(&arg_bytes, config::standard().with_fixed_int_encoding())
        .map_err(|_| Error::Pd(PdError::Serialize))?;

    for (i, controller) in controllers.iter_mut().enumerate() {
        info!("Controller {}: Streaming data block", i);
        if controller.fw_update_stream_data(delay, &args).await.is_err() {
            error!("Controller {}: Failed to stream data block", i);
            exit_fw_update_mode(delay, controllers).await?;
            return Err(Error::Pd(PdError::Failed));
        }
    }

    let data_len = args.data_len as usize;
    let data_offset = block_offset(metadata_offset);
    let mut buf = [0u8; BURST_WRITE_SIZE];

    image
        .seek(SeekFrom::Start(data_offset as u64))
        .await
        .map_err(Error::Io)?;

    for _ in 0..data_len / BURST_WRITE_SIZE {
        image.read_exact(&mut buf).await.map_err(Error::ReadExact)?;
        // All the controllers share the same bus, so just use the first controller
        controllers[0]
            .fw_update_burst_write(args.broadcast_u16_address as u8, &buf)
            .await?;
    }

    delay.delay_ms(TFUD_BURST_WRITE_DELAY_MS).await;

    // Validate the block on all controllers
    for (i, controller) in controllers.iter_mut().enumerate() {
        info!("Controller {}: Validating block {}", i, block_index);
        match controller.fw_update_validate_stream(delay, block_index).await {
            Ok(TfuqBlockStatus::DataValidAndAuthentic)
            | Ok(TfuqBlockStatus::DataValidButRepeated)
            | Ok(TfuqBlockStatus::HeaderValidAndAuthentic) => (),
            Ok(r) => {
                error!("Controller {}: Block validation failed, result {}", i, r);
                exit_fw_update_mode(delay, controllers).await?;
                return Err(Error::Pd(PdError::Failed));
            }
            Err(_) => {
                error!("Controller {}: Block validation failed", i);
                exit_fw_update_mode(delay, controllers).await?;
                return Err(Error::Pd(PdError::Failed));
            }
        }
    }
    Ok(())
}

/// Load the app image to all controllers
async fn fw_update_load_app_image<T: UpdateTarget, I: Image>(
    delay: &mut impl DelayNs,
    controllers: &mut [&mut T],
    image: &mut I,
    num_data_blocks: usize,
) -> Result<(), Error<T, I>> {
    for i in 0..num_data_blocks {
        info!("Broadcasting block {}", i + 1);
        fw_update_stream_data(
            delay,
            controllers,
            image,
            data_block_index_to_block_index(i),
            data_block_metadata_offset(i),
            DATA_BLOCK_METADATA_LEN,
        )
        .await?;
    }

    Ok(())
}

/// Load the app config to all controllers
async fn fw_update_load_app_config<T: UpdateTarget, I: Image>(
    delay: &mut impl DelayNs,
    controllers: &mut [&mut T],
    image: &mut I,
    num_data_blocks: usize,
) -> Result<(), Error<T, I>> {
    let app_size = get_image_size(image).await.map_err(Error::ReadExact)? as usize;
    let metadata_offset = app_config_block_metadata_offset(num_data_blocks, app_size);
    info!("Broadcasting app config block");
    fw_update_stream_data(
        delay,
        controllers,
        image,
        APP_CONFIG_BLOCK_INDEX,
        metadata_offset,
        APP_CONFIG_METADATA_LEN,
    )
    .await
}

/// Complete the FW update on all controllers
async fn fw_update_complete<T: UpdateTarget, I: Image>(
    delay: &mut impl DelayNs,
    controllers: &mut [&mut T],
) -> Result<(), Error<T, I>> {
    for (i, controller) in controllers.iter_mut().enumerate() {
        info!("Controller {}: Completing FW update", i);
        if controller.fw_update_complete(delay).await.is_err() {
            warn!("Controller {}: Failed to complete FW update, attempting to exit", i);
            controller.fw_update_mode_exit(delay).await?;
            // Don't return to allow the other controllers to exit FW update mode
        }
    }

    Ok(())
}

/// Updates the firmware of all given controllers
pub async fn perform_fw_update<const N: usize, T: UpdateTarget, I: Image>(
    delay: &mut impl DelayNs,
    mut controllers: [&mut T; N],
    image: &mut I,
) -> Result<(), Error<T, I>> {
    info!("Starting FW update");

    // Disable all interrupts during the reset into FW update mode
    let mut main_guards: [Option<T::Guard>; N] = [const { None }; N];
    for (guard, controller) in zip(main_guards.iter_mut(), controllers.iter_mut()) {
        *guard = Some(controller.disable_all_interrupts_guarded().await.map_err(Error::from)?);
    }

    enter_fw_update_mode(delay, controllers.as_mut_slice()).await?;

    // Re-enable interrupts on port 0 only, other ports don't respond to interrupts in FW update mode
    let mut port0_guards: [Option<T::Guard>; N] = [const { None }; N];
    for (guard, controller) in zip(port0_guards.iter_mut(), controllers.iter_mut()) {
        *guard = Some(
            controller
                .enable_interrupt_guarded(PORT0, true)
                .await
                .map_err(Error::from)?,
        );
    }

    let tfui_args = fw_update_init(delay, controllers.as_mut_slice(), image).await?;
    fw_update_load_app_image(
        delay,
        controllers.as_mut_slice(),
        image,
        tfui_args.num_data_blocks_tx as usize,
    )
    .await?;
    fw_update_load_app_config(
        delay,
        controllers.as_mut_slice(),
        image,
        tfui_args.num_data_blocks_tx as usize,
    )
    .await?;
    fw_update_complete(delay, controllers.as_mut_slice()).await?;

    info!("FW update complete");

    Ok(())
}

/// Simple wrapper around a slice to allow for use as a FW update image
pub struct SliceImage<'a> {
    image: &'a [u8],
    pos: usize,
}

impl<'a> SliceImage<'a> {
    pub fn new(image: &'a [u8]) -> Self {
        Self { image, pos: 0 }
    }
}

impl embedded_io_async::ErrorType for SliceImage<'_> {
    type Error = embedded_io_async::ErrorKind;
}

impl Read for SliceImage<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.len() > self.image.len() - self.pos {
            return Err(embedded_io_async::ErrorKind::InvalidInput);
        }

        buf.copy_from_slice(&self.image[self.pos..self.pos + buf.len()]);
        self.pos += buf.len();

        Ok(buf.len())
    }
}

impl Seek for SliceImage<'_> {
    async fn seek(&mut self, pos: SeekFrom) -> Result<u64, Self::Error> {
        match pos {
            SeekFrom::Start(p) => {
                if p as usize > self.image.len() {
                    return Err(embedded_io_async::ErrorKind::InvalidInput);
                }
                self.pos = p as usize;
            }
            SeekFrom::End(i) => {
                if i > 0 || (-i) as usize > self.image.len() {
                    return Err(embedded_io_async::ErrorKind::InvalidInput);
                }

                self.pos = (self.image.len() as i64 + i) as usize;
            }
            SeekFrom::Current(off) => {
                let pos = self.pos as i64;
                if pos + off < 0 || (pos + off) as usize > self.image.len() {
                    return Err(embedded_io_async::ErrorKind::InvalidInput);
                }

                self.pos = (pos + off) as usize;
            }
        }

        Ok(self.pos as u64)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::test::Delay;

    /// Simple mock update target for testing that does nothing
    #[derive(Debug)]
    struct UpdateTargetNoop {}

    impl InterruptController for UpdateTargetNoop {
        type Guard = ();
        type BusError = ();

        async fn interrupts_enabled(&self) -> Result<[bool; MAX_SUPPORTED_PORTS], Error<Self, Self::BusError>> {
            Ok([true; MAX_SUPPORTED_PORTS])
        }

        async fn enable_interrupts_guarded(
            &mut self,
            enabled: [bool; MAX_SUPPORTED_PORTS],
        ) -> Result<Self::Guard, Error<Self, Self::BusError>> {
            Ok(())
        }
    }

    impl UpdateTarget for UpdateTargetNoop {
        async fn fw_update_mode_enter(&mut self, _delay: &mut impl DelayNs) -> Result<(), DeviceError<Self::BusError>> {
            Ok(())
        }

        async fn fw_update_init(
            &mut self,
            _delay: &mut impl DelayNs,
            _args: &TfuiArgs,
        ) -> Result<ReturnValue, DeviceError<Self::BusError>> {
            Ok(ReturnValue::Success)
        }

        async fn fw_update_mode_exit(&mut self, _delay: &mut impl DelayNs) -> Result<(), DeviceError<Self::BusError>> {
            Ok(())
        }

        async fn fw_update_validate_stream(
            &mut self,
            _delay: &mut impl DelayNs,
            block_index: usize,
        ) -> Result<TfuqBlockStatus, DeviceError<Self::BusError>> {
            match block_index {
                HEADER_BLOCK_INDEX => Ok(TfuqBlockStatus::HeaderValidAndAuthentic),
                _ => Ok(TfuqBlockStatus::DataValidAndAuthentic),
            }
        }

        async fn fw_update_stream_data(
            &mut self,
            _delay: &mut impl DelayNs,
            _args: &TfudArgs,
        ) -> Result<(), DeviceError<Self::BusError>> {
            Ok(())
        }

        async fn fw_update_complete(&mut self, _delay: &mut impl DelayNs) -> Result<(), DeviceError<Self::BusError>> {
            Ok(())
        }

        async fn fw_update_burst_write(
            &mut self,
            _address: u8,
            _data: &[u8],
        ) -> Result<(), DeviceError<Self::BusError>> {
            Ok(())
        }
    }

    /// Simple mock image for testing that panics if a seek backward is attempted
    #[derive(Debug)]
    struct ImageSeekForward {
        pos: u64,
    }

    impl ImageSeekForward {
        fn new() -> Self {
            Self { pos: 0 }
        }
    }

    impl Seek for ImageSeekForward {
        async fn seek(&mut self, pos: SeekFrom) -> Result<u64, Self::Error> {
            match pos {
                SeekFrom::Start(p) => {
                    assert!(p >= self.pos);
                    self.pos = p;
                }
                SeekFrom::End(_) => panic!("Seeking backwards is not supported"),
                SeekFrom::Current(off) => {
                    assert!(off >= 0);
                    self.pos += off as u64;
                }
            }

            Ok(self.pos)
        }
    }

    #[derive(Debug)]
    struct ErrorMock {}

    impl embedded_io_async::Error for ErrorMock {
        fn kind(&self) -> embedded_io_async::ErrorKind {
            embedded_io_async::ErrorKind::Other
        }
    }

    impl embedded_io_async::ErrorType for ImageSeekForward {
        type Error = ErrorMock;
    }

    impl Read for ImageSeekForward {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            Ok(buf.len())
        }
    }

    /// Test that the fw update only seeks forward
    /// This ensures that the process will work if we're getting the update from another device
    #[tokio::test]
    async fn test_fw_update_seek_forward() {
        let mut delay = Delay {};
        let mut target = UpdateTargetNoop {};
        let mut image = ImageSeekForward::new();

        perform_fw_update(&mut delay, &mut [&mut target], &mut image)
            .await
            .unwrap();
    }
}
