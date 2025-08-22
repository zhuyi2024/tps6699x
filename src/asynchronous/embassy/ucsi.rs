//! UCSI related functionality
use bincode::{decode_from_slice_with_context, encode_into_slice};
use embedded_usb_pd::ucsi::lpm;

use super::*;
use crate::registers::REG_DATA1_LEN;

impl<'a, M: RawMutex, B: I2c> Tps6699x<'a, M, B> {
    pub async fn execute_ucsi_command(
        &mut self,
        command: &lpm::LocalCommand,
    ) -> Result<lpm::ResponseData, Error<B::Error>> {
        let mut indata = [0u8; REG_DATA1_LEN];
        // -1 because the first byte is used for the standard TPS6699x command return value
        let mut outdata = [0u8; REG_DATA1_LEN - 1];
        let port = command.port;

        // Internally the controller uses 1-based port numbering
        let mut command = command.clone();
        command.port = LocalPortId(command.port.0 + 1);

        encode_into_slice(
            command,
            &mut indata,
            bincode::config::standard().with_fixed_int_encoding(),
        )
        .map_err(|_| Error::Pd(PdError::Serialize))?;

        let ret = self
            .execute_command(port, Command::Ucsi, Some(&indata), Some(&mut outdata))
            .await?;

        if ret != ReturnValue::Success {
            error!("UCSI command failed with return value: {:?}", ret);
            return Err(PdError::Failed.into());
        }

        trace!("UCSI command response: {:?}", outdata);
        let (response_data, _) = decode_from_slice_with_context(
            &outdata,
            bincode::config::standard().with_fixed_int_encoding(),
            command.command_type(),
        )
        .map_err(|_| Error::Pd(PdError::Serialize))?;
        Ok(response_data)
    }
}
