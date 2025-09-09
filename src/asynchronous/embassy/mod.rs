//! This module contains a high-level API uses embassy synchronization types
use core::array::from_fn;
use core::iter::zip;
use core::sync::atomic::AtomicBool;

use bincode::config;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::{Mutex, MutexGuard};
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration};
use embedded_hal::digital::InputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use embedded_usb_pd::ado::{self, Ado};
use embedded_usb_pd::pdinfo::AltMode;
use embedded_usb_pd::{pdo, Error, LocalPortId, PdError};
use itertools::izip;

use super::interrupt::{self, InterruptController};
use crate::asynchronous::internal;
use crate::command::{muxr, trig, vdms, Command, ReturnValue, SrdySwitch};
use crate::registers::autonegotiate_sink::AutoComputeSinkMaxVoltage;
use crate::registers::field_sets::IntEventBus1;
use crate::{error, registers, trace, warn, DeviceError, Mode, MAX_SUPPORTED_PORTS};

pub mod fw_update;
pub mod rx_src_caps;
pub mod task;
pub mod ucsi;

pub mod controller {
    use super::*;
    use crate::{TPS66993_NUM_PORTS, TPS66994_NUM_PORTS};

    /// Controller struct. This struct is meant to be created and then immediately broken into its parts
    pub struct Controller<M: RawMutex, B: I2c> {
        /// Low-level TPS6699x driver
        pub(super) inner: Mutex<M, internal::Tps6699x<B>>,
        /// Signal for awaiting an interrupt
        pub(super) interrupt_waker: Signal<M, [IntEventBus1; MAX_SUPPORTED_PORTS]>,
        /// Current interrupt state
        pub(super) interrupts_enabled: [AtomicBool; MAX_SUPPORTED_PORTS],
        /// Number of active ports
        pub(super) num_ports: usize,
    }

    impl<M: RawMutex, B: I2c> Controller<M, B> {
        /// Private constructor
        pub fn new(bus: B, addr: [u8; MAX_SUPPORTED_PORTS], num_ports: usize) -> Result<Self, Error<B::Error>> {
            Ok(Self {
                inner: Mutex::new(internal::Tps6699x::new(bus, addr, num_ports)),
                interrupt_waker: Signal::new(),
                interrupts_enabled: [const { AtomicBool::new(true) }; MAX_SUPPORTED_PORTS],
                num_ports,
            })
        }

        /// Create a new controller for the TPS66993
        pub fn new_tps66993(bus: B, addr: u8) -> Result<Self, Error<B::Error>> {
            Self::new(bus, [addr, 0], TPS66993_NUM_PORTS)
        }

        /// Create a new controller for the TPS66994
        pub fn new_tps66994(bus: B, addr: [u8; TPS66994_NUM_PORTS]) -> Result<Self, Error<B::Error>> {
            Self::new(bus, addr, TPS66994_NUM_PORTS)
        }

        /// Breaks the controller into its parts
        pub fn make_parts(&mut self) -> (Tps6699x<'_, M, B>, Interrupt<'_, M, B>) {
            let tps = Tps6699x { controller: self };
            let interrupt = Interrupt { controller: self };
            (tps, interrupt)
        }

        /// Enable or disable interrupts for the given ports
        pub(super) fn enable_interrupts(&self, enabled: [bool; MAX_SUPPORTED_PORTS]) {
            for (enabled, s) in zip(enabled.iter(), self.interrupts_enabled.iter()) {
                s.store(*enabled, core::sync::atomic::Ordering::SeqCst);
            }
        }

        /// Returns current interrupt state
        pub(super) fn interrupts_enabled(&self) -> [bool; MAX_SUPPORTED_PORTS] {
            let mut interrupts_enabled = [false; MAX_SUPPORTED_PORTS];
            for (copy, enabled) in zip(interrupts_enabled.iter_mut(), self.interrupts_enabled.iter()) {
                *copy = enabled.load(core::sync::atomic::Ordering::SeqCst);
            }

            interrupts_enabled
        }
    }
}

/// Struct for controlling a TP6699x device
pub struct Tps6699x<'a, M: RawMutex, B: I2c> {
    controller: &'a controller::Controller<M, B>,
}

impl<'a, M: RawMutex, B: I2c> Tps6699x<'a, M, B> {
    /// Locks the inner device
    pub async fn lock_inner(&mut self) -> MutexGuard<'_, M, internal::Tps6699x<B>> {
        self.controller.inner.lock().await
    }

    /// Wrapper for `modify_interrupt_mask`
    pub async fn modify_interrupt_mask(
        &mut self,
        port: LocalPortId,
        f: impl FnOnce(&mut registers::field_sets::IntEventBus1) -> registers::field_sets::IntEventBus1,
    ) -> Result<registers::field_sets::IntEventBus1, Error<B::Error>> {
        self.lock_inner().await.modify_interrupt_mask(port, f).await
    }

    /// Wrapper for `modify_interrupt_mask_all`
    pub async fn modify_interrupt_mask_all(
        &mut self,
        f: impl Fn(&mut registers::field_sets::IntEventBus1) -> registers::field_sets::IntEventBus1,
    ) -> Result<(), Error<B::Error>> {
        self.lock_inner().await.modify_interrupt_mask_all(f).await
    }

    /// Wrapper for `get_port_status``
    pub async fn get_port_status(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::field_sets::Status, Error<B::Error>> {
        self.lock_inner().await.get_port_status(port).await
    }

    /// Wrapper for `get_active_pdo_contract`
    pub async fn get_active_pdo_contract(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::field_sets::ActivePdoContract, Error<B::Error>> {
        self.lock_inner().await.get_active_pdo_contract(port).await
    }

    /// Wrapper for `get_active_rdo_contract`
    pub async fn get_active_rdo_contract(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::field_sets::ActiveRdoContract, Error<B::Error>> {
        self.lock_inner().await.get_active_rdo_contract(port).await
    }

    /// Get the Autonegotiate Sink register (`0x37`).
    pub async fn get_autonegotiate_sink(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::autonegotiate_sink::AutonegotiateSink, Error<B::Error>> {
        self.lock_inner().await.get_autonegotiate_sink(port).await
    }

    /// Set the Autonegotiate Sink register (`0x37`).
    pub async fn set_autonegotiate_sink(
        &mut self,
        port: LocalPortId,
        value: registers::autonegotiate_sink::AutonegotiateSink,
    ) -> Result<(), Error<B::Error>> {
        self.lock_inner().await.set_autonegotiate_sink(port, value).await
    }

    /// Modify the Autonegotiate Sink register (`0x37`).
    pub async fn modify_autonegotiate_sink(
        &mut self,
        port: LocalPortId,
        f: impl FnOnce(
            &mut registers::autonegotiate_sink::AutonegotiateSink,
        ) -> registers::autonegotiate_sink::AutonegotiateSink,
    ) -> Result<registers::autonegotiate_sink::AutonegotiateSink, Error<B::Error>> {
        self.lock_inner().await.modify_autonegotiate_sink(port, f).await
    }

    /// Wrapper for `get_mode`
    pub async fn get_mode(&mut self) -> Result<Mode, Error<B::Error>> {
        self.lock_inner().await.get_mode().await
    }

    /// Wrapper for `get_fw_version`
    pub async fn get_fw_version(&mut self) -> Result<u32, Error<B::Error>> {
        self.lock_inner().await.get_fw_version().await
    }

    /// Wrapper for `get_customer_use`
    pub async fn get_customer_use(&mut self) -> Result<u64, Error<B::Error>> {
        self.lock_inner().await.get_customer_use().await
    }

    /// Wrapper for `get_power_path_status`
    pub async fn get_power_path_status(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::field_sets::PowerPathStatus, Error<B::Error>> {
        self.lock_inner().await.get_power_path_status(port).await
    }

    /// Wrapper for `get_pd_status`
    pub async fn get_pd_status(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::field_sets::PdStatus, Error<B::Error>> {
        self.lock_inner().await.get_pd_status(port).await
    }

    /// Wrapper for `get_port_control`
    pub async fn get_port_control(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::field_sets::PortControl, Error<B::Error>> {
        self.lock_inner().await.get_port_control(port).await
    }

    /// Wrapper for `set_port_control`
    pub async fn set_port_control(
        &mut self,
        port: LocalPortId,
        control: registers::field_sets::PortControl,
    ) -> Result<(), Error<B::Error>> {
        self.lock_inner().await.set_port_control(port, control).await
    }

    /// Wrapper for `get_system_config`
    pub async fn get_system_config(&mut self) -> Result<registers::field_sets::SystemConfig, Error<B::Error>> {
        self.lock_inner().await.get_system_config().await
    }

    /// Wrapper for `set_system_config`
    pub async fn set_system_config(
        &mut self,
        config: registers::field_sets::SystemConfig,
    ) -> Result<(), Error<B::Error>> {
        self.lock_inner().await.set_system_config(config).await
    }

    /// Wrapper for `enable_source`
    pub async fn enable_source(&mut self, port: LocalPortId, enable: bool) -> Result<(), Error<B::Error>> {
        self.lock_inner().await.enable_source(port, enable).await
    }

    /// Returns the number of ports
    pub fn num_ports(&self) -> usize {
        self.controller.num_ports
    }

    /// Wait for an interrupt to occur that matches any bits in the given mask.
    pub async fn wait_interrupt_any(
        &mut self,
        clear_current: bool,
        mask: [IntEventBus1; MAX_SUPPORTED_PORTS],
    ) -> [IntEventBus1; MAX_SUPPORTED_PORTS] {
        // No interrupts set, return immediately because there is nothing to wait for
        // Also log a warning because this likely isn't what the user intended
        if mask == [IntEventBus1::new_zero(); MAX_SUPPORTED_PORTS] {
            warn!("Interrupt masks are empty, returning immediately");
            return [IntEventBus1::new_zero(); MAX_SUPPORTED_PORTS];
        }

        if clear_current {
            self.controller.interrupt_waker.reset();
        }

        let mut accumulated_flags = [IntEventBus1::new_zero(); MAX_SUPPORTED_PORTS];
        loop {
            let mut done = false;
            let flags = self.controller.interrupt_waker.wait().await;
            for (&flags, &mask, accumulated) in izip!(flags.iter(), mask.iter(), accumulated_flags.iter_mut(),) {
                *accumulated |= flags;
                let consumed_flags = flags & mask;
                if consumed_flags != IntEventBus1::new_zero() {
                    done = true;
                }
            }

            if done {
                // Put back any unhandled interrupt flags for future processing
                let unhandled = from_fn(|i| accumulated_flags[i] & !mask[i]);
                if unhandled.iter().any(|&f| f != IntEventBus1::new_zero()) {
                    // If there are unhandled flags, signal them for future processing
                    trace!("Signaling unhandled interrupt flags: {:?}", unhandled);
                    self.controller.interrupt_waker.signal(unhandled);
                }
                return from_fn(|i| accumulated_flags[i] & mask[i]);
            }
        }
    }

    /// Execute the given command with no timeout
    async fn execute_command_no_timeout(
        &mut self,
        port: LocalPortId,
        cmd: Command,
        indata: Option<&[u8]>,
        outdata: Option<&mut [u8]>,
    ) -> Result<ReturnValue, Error<B::Error>> {
        {
            let mut inner = self.lock_inner().await;
            inner.send_command(port, cmd, indata).await?;
        }

        let mut cmd_complete = IntEventBus1::new_zero();
        cmd_complete.set_cmd_1_completed(true);

        let _flags = self
            .wait_interrupt_any(
                false,
                from_fn(|i| {
                    if i == port.0 as usize {
                        cmd_complete
                    } else {
                        IntEventBus1::new_zero()
                    }
                }),
            )
            .await;
        {
            let mut inner = self.lock_inner().await;
            inner.read_command_result(port, outdata).await
            // todo: map command result here
        }
    }

    /// Execute the given command with a timeout determined by [`Command::timeout`].
    async fn execute_command(
        &mut self,
        port: LocalPortId,
        cmd: Command,
        indata: Option<&[u8]>,
        outdata: Option<&mut [u8]>,
    ) -> Result<ReturnValue, Error<B::Error>> {
        let timeout = cmd.timeout();
        let result = with_timeout(timeout, self.execute_command_no_timeout(port, cmd, indata, outdata)).await;
        if result.is_err() {
            error!("Command {:#?} timed out", cmd);
            // See if there's a definite error we can read
            let mut inner = self.lock_inner().await;
            return match inner.read_command_result(port, None).await? {
                ReturnValue::Rejected => PdError::Rejected,
                _ => PdError::Timeout,
            }
            .into();
        }

        result.unwrap()
    }

    async fn execute_srdy(&mut self, port: LocalPortId, switch: SrdySwitch) -> Result<ReturnValue, Error<B::Error>> {
        let arg_bytes = [switch.into()];
        self.execute_command(port, Command::Srdy, Some(&arg_bytes), None).await
    }

    async fn execute_sryr(&mut self, port: LocalPortId) -> Result<ReturnValue, Error<B::Error>> {
        self.execute_command(port, Command::Sryr, None, None).await
    }

    /// Enable or disable the given power path
    pub async fn enable_sink_path(&mut self, port: LocalPortId, enable: bool) -> Result<(), Error<B::Error>> {
        if enable {
            self.execute_srdy(
                port,
                match port.0 {
                    0 => Ok(SrdySwitch::PpExt1),
                    1 => Ok(SrdySwitch::PpExt2),
                    _ => PdError::InvalidPort.into(),
                }?,
            )
            .await?;
        } else {
            self.execute_sryr(port).await?;
        }

        Ok(())
    }

    /// Trigger an `ANeg` command to autonegotiate the sink contract.
    pub async fn autonegotiate_sink(&mut self, port: LocalPortId) -> Result<(), Error<B::Error>> {
        match self.execute_command(port, Command::Aneg, None, None).await? {
            ReturnValue::Success => Ok(()),
            ReturnValue::Rejected => PdError::Rejected.into(),
            _ => PdError::Failed.into(),
        }
    }

    /// Trigger virtual gpios
    async fn virtual_gpio_trigger(
        &mut self,
        port: LocalPortId,
        edge: trig::Edge,
        cmd: trig::Cmd,
    ) -> Result<ReturnValue, Error<B::Error>> {
        let args = trig::Args { edge, cmd };
        let mut args_buf = [0; trig::ARGS_LEN];

        bincode::encode_into_slice(args, &mut args_buf, config::standard().with_fixed_int_encoding()).unwrap();

        self.execute_command(port, Command::Trig, Some(&args_buf), None).await
    }

    /// Force retimer power on or off
    pub async fn retimer_force_pwr(&mut self, port: LocalPortId, enable: bool) -> Result<(), Error<B::Error>> {
        trace!("retimer_force_pwr: {}", enable);

        let edge = if enable {
            trig::Edge::Rising
        } else {
            trig::Edge::Falling
        };

        self.virtual_gpio_trigger(port, edge, trig::Cmd::RetimerForcePwr)
            .await?;

        embassy_time::Timer::after(Duration::from_millis(50)).await;

        Ok(())
    }

    /// Get retimer fw update state
    pub async fn get_rt_fw_update_status(&mut self, port: LocalPortId) -> Result<bool, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        let rt_fw_update_mode = inner.get_intel_vid_status(port).await?.forced_tbt_mode();
        trace!("rt_fw_update_mode: {}", rt_fw_update_mode);
        Ok(rt_fw_update_mode)
    }

    /// set retimer fw update state
    pub async fn set_rt_fw_update_state(&mut self, port: LocalPortId) -> Result<(), Error<B::Error>> {
        // Force RT Pwr On
        self.retimer_force_pwr(port, true).await?;

        let mut inner = self.lock_inner().await;
        let mut port_control = inner.get_port_control(port).await?;
        port_control.set_retimer_fw_update(true);
        inner.set_port_control(port, port_control).await?;
        Ok(())
    }

    /// clear retimer fw update state
    pub async fn clear_rt_fw_update_state(&mut self, port: LocalPortId) -> Result<(), Error<B::Error>> {
        {
            let mut inner = self.lock_inner().await;
            let mut port_control = inner.get_port_control(port).await?;
            port_control.set_retimer_fw_update(false);
            inner.set_port_control(port, port_control).await?;
        }

        // Force RT Pwr Off
        self.retimer_force_pwr(port, false).await?;

        Ok(())
    }

    /// set retimer compliance
    pub async fn set_rt_compliance(&mut self, port: LocalPortId) -> Result<(), Error<B::Error>> {
        {
            // Force RT Pwr On
            self.retimer_force_pwr(port, true).await?;

            let mut inner = self.lock_inner().await;
            let mut tbt_config = inner.get_tbt_config(port).await?;
            tbt_config.set_retimer_compliance_support(true);
            inner.set_tbt_config(port, tbt_config).await?;
        }

        Ok(())
    }

    /// Execute the [`Command::Dbfg`] command.
    pub async fn execute_dbfg(&mut self, port: LocalPortId) -> Result<ReturnValue, Error<B::Error>> {
        self.execute_command(port, Command::Dbfg, None, None).await
    }

    /// Execute the [`Command::Muxr`] command.
    pub async fn execute_muxr(
        &mut self,
        port: LocalPortId,
        input: muxr::Input,
    ) -> Result<ReturnValue, Error<B::Error>> {
        let indata = input.0.to_le_bytes();
        self.execute_command(port, Command::Muxr, Some(&indata), None).await
    }

    /// Execute the [`Command::VDMs`] command.
    pub async fn send_vdms(&mut self, port: LocalPortId, input: vdms::Input) -> Result<ReturnValue, Error<B::Error>> {
        let indata = input.as_bytes();
        self.execute_command(port, Command::VDMs, Some(indata), None).await
    }

    /// Reset the device.
    pub async fn reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<B::Error>> {
        let _guard = self.disable_all_interrupts_guarded().await;
        let mut inner = self.lock_inner().await;
        inner.reset(delay, &Default::default()).await
    }

    /// Get boot flags
    pub async fn get_boot_flags(&mut self) -> Result<registers::boot_flags::BootFlags, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.get_boot_flags().await
    }

    /// Get DP status
    pub async fn get_dp_status(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::dp_status::DpStatus, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.get_dp_status(port).await
    }

    /// Get Intel VID status
    pub async fn get_intel_vid(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::field_sets::IntelVidStatus, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.get_intel_vid_status(port).await
    }

    /// Get USB status
    pub async fn get_usb_status(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::field_sets::UsbStatus, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.get_usb_status(port).await
    }

    /// Get user VID status
    pub async fn get_user_vid(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::field_sets::UserVidStatus, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.get_user_vid_status(port).await
    }

    /// Get complete alt-mode status
    pub async fn get_alt_mode_status(&mut self, port: LocalPortId) -> Result<AltMode, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.get_alt_mode_status(port).await
    }

    /// Set unconstrained power on a port
    pub async fn set_unconstrained_power(&mut self, port: LocalPortId, enable: bool) -> Result<(), Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.set_unconstrained_power(port, enable).await
    }

    /// Get port config
    pub async fn get_port_config(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::port_config::PortConfig, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.get_port_config(port).await
    }

    /// Set port config
    pub async fn set_port_config(
        &mut self,
        port: LocalPortId,
        config: registers::port_config::PortConfig,
    ) -> Result<(), Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.set_port_config(port, config).await
    }

    /// Get Rx ADO
    pub async fn get_rx_ado(
        &mut self,
        port: LocalPortId,
    ) -> Result<Option<Ado>, DeviceError<B::Error, ado::InvalidType>> {
        let mut inner = self.lock_inner().await;
        let ado_raw = inner.get_rx_ado(port).await.map_err(DeviceError::from)?;

        if ado_raw == registers::field_sets::RxAdo::new_zero() {
            // No ADO available
            Ok(None)
        } else {
            Ok(Some(ado_raw.ado().try_into().map_err(DeviceError::Other)?))
        }
    }

    /// Get Rx Attention Vdm
    pub async fn get_rx_attn_vdm(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::field_sets::RxAttnVdm, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.get_rx_attn_vdm(port).await
    }

    /// Get Rx Other Vdm
    pub async fn get_rx_other_vdm(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::rx_other_vdm::RxOtherVdm, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.get_rx_other_vdm(port).await
    }

    /// Set autonegotiate sink max voltage. This may trigger a renegotiation
    pub async fn set_autonegotiate_sink_max_voltage(
        &mut self,
        port: LocalPortId,
        voltage_mv: Option<u16>,
    ) -> Result<(), Error<B::Error>> {
        self.modify_autonegotiate_sink(port, |settings| {
            if let Some(voltage) = voltage_mv {
                settings.set_auto_compute_sink_max_voltage(AutoComputeSinkMaxVoltage::ProvidedByHost);
                settings.set_auto_neg_max_voltage(voltage);
            } else {
                // Auto neg max voltage is ignored if this value is set
                settings.set_auto_compute_sink_max_voltage(AutoComputeSinkMaxVoltage::ComputedByPdController);
            }

            settings.clone()
        })
        .await?;

        // Trigger autonegotiate sink to apply the new max voltage
        // This will result in a rejection if the port is not a sink, but this is expected
        match self.autonegotiate_sink(port).await {
            Err(Error::Pd(PdError::Rejected)) => Ok(()),
            rest => rest,
        }
    }

    /// Get Rx Source Caps
    ///
    /// Returns (num_standard_pdos, num_epr_pdos).
    pub async fn get_rx_src_caps(&mut self, port: LocalPortId) -> Result<rx_src_caps::RxSrcCaps, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        let mut out_spr_pdos = [pdo::source::Pdo::default(); crate::registers::rx_src_caps::NUM_SPR_PDOS];
        let mut out_epr_pdos = [pdo::source::Pdo::default(); crate::registers::rx_src_caps::NUM_EPR_PDOS];

        let (num_valid_spr, num_valid_epr) = inner
            .get_rx_src_caps(port, &mut out_spr_pdos, &mut out_epr_pdos)
            .await?;

        // These unwraps are safe because we know the sizes of the arrays
        Ok(rx_src_caps::RxSrcCaps {
            spr: heapless::Vec::from_slice(&out_spr_pdos[..num_valid_spr]).unwrap(),
            epr: heapless::Vec::from_slice(&out_epr_pdos[..num_valid_epr]).unwrap(),
        })
    }

    /// Get Tx Identity
    pub async fn get_tx_identity(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::tx_identity::TxIdentity, Error<B::Error>> {
        self.lock_inner().await.get_tx_identity(port).await
    }

    /// Set Tx Identity
    pub async fn set_tx_identity(
        &mut self,
        port: LocalPortId,
        value: registers::tx_identity::TxIdentity,
    ) -> Result<(), Error<B::Error>> {
        self.lock_inner().await.set_tx_identity(port, value).await
    }

    /// Modify the Tx Identity register (`0x47`).
    pub async fn modify_tx_identity(
        &mut self,
        port: LocalPortId,
        f: impl FnOnce(&mut registers::tx_identity::TxIdentity) -> registers::tx_identity::TxIdentity,
    ) -> Result<registers::tx_identity::TxIdentity, Error<B::Error>> {
        self.lock_inner().await.modify_tx_identity(port, f).await
    }

    /// Get DP config
    pub async fn get_dp_config(
        &mut self,
        port: LocalPortId,
    ) -> Result<registers::field_sets::DpConfig, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.get_dp_config(port).await
    }

    /// Set DP config
    pub async fn set_dp_config(
        &mut self,
        port: LocalPortId,
        config: registers::field_sets::DpConfig,
    ) -> Result<(), Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.set_dp_config(port, config).await
    }

    /// Modify DP config settings
    pub async fn modify_dp_config(
        &mut self,
        port: LocalPortId,
        f: impl FnOnce(&mut registers::field_sets::DpConfig) -> registers::field_sets::DpConfig,
    ) -> Result<registers::field_sets::DpConfig, Error<B::Error>> {
        let mut inner = self.lock_inner().await;
        inner.modify_dp_config(port, f).await
    }

    /// Execute the [`Command::Drst`] command.
    pub async fn execute_drst(&mut self, port: LocalPortId) -> Result<ReturnValue, Error<B::Error>> {
        self.execute_command(port, Command::Drst, None, None).await
    }
}

impl<'a, M: RawMutex, B: I2c> interrupt::InterruptController for Tps6699x<'a, M, B> {
    type Guard = InterruptGuard<'a, M, B>;
    type BusError = B::Error;

    async fn interrupts_enabled(&self) -> Result<[bool; MAX_SUPPORTED_PORTS], Error<Self::BusError>> {
        Ok(self.controller.interrupts_enabled())
    }

    async fn enable_interrupts_guarded(
        &mut self,
        enabled: [bool; MAX_SUPPORTED_PORTS],
    ) -> Result<Self::Guard, Error<Self::BusError>> {
        Ok(InterruptGuard::new(self.controller, enabled))
    }
}

pub struct Interrupt<'a, M: RawMutex, B: I2c> {
    controller: &'a controller::Controller<M, B>,
}

impl<'a, M: RawMutex, B: I2c> Interrupt<'a, M, B> {
    async fn lock_inner(&mut self) -> MutexGuard<'_, M, internal::Tps6699x<B>> {
        self.controller.inner.lock().await
    }

    /// Process interrupts
    pub async fn process_interrupt(
        &mut self,
        int: &mut impl InputPin,
    ) -> Result<[IntEventBus1; MAX_SUPPORTED_PORTS], Error<B::Error>> {
        let mut flags = self
            .controller
            .interrupt_waker
            .try_take()
            .unwrap_or([IntEventBus1::new_zero(); MAX_SUPPORTED_PORTS]);

        {
            let interrupts_enabled = self.controller.interrupts_enabled();
            let mut inner = self.lock_inner().await;
            for port in 0..inner.num_ports() {
                let port_id = LocalPortId(port as u8);

                if !interrupts_enabled[port] {
                    trace!("Port{}: Interrupt for disabled", port);
                    continue;
                }

                match int.is_high() {
                    Ok(true) => {
                        // Early exit if checking the last port cleared the interrupt
                        trace!("Interrupt line is high, exiting");
                        continue;
                    }
                    Err(_) => {
                        error!("Failed to read interrupt line");
                        return PdError::Failed.into();
                    }
                    _ => {}
                }

                let result = with_timeout(Duration::from_millis(100), inner.clear_interrupt(port_id)).await;
                match result {
                    Ok(res) => {
                        flags[port] |= res?;
                    }
                    Err(_) => {
                        error!("Port{}: clear_interrupt timeout", port);
                        continue;
                    }
                }
            }
        }

        self.controller.interrupt_waker.signal(flags);
        Ok(flags)
    }
}

/// Restores the original interrupt state when dropped
pub struct InterruptGuard<'a, M: RawMutex, B: I2c> {
    target_state: [bool; MAX_SUPPORTED_PORTS],
    controller: &'a controller::Controller<M, B>,
}

impl<'a, M: RawMutex, B: I2c> InterruptGuard<'a, M, B> {
    fn new(controller: &'a controller::Controller<M, B>, enabled: [bool; MAX_SUPPORTED_PORTS]) -> Self {
        let target_state = controller.interrupts_enabled();
        controller.enable_interrupts(enabled);
        Self {
            target_state,
            controller,
        }
    }
}

impl<M: RawMutex, B: I2c> Drop for InterruptGuard<'_, M, B> {
    fn drop(&mut self) {
        self.controller.enable_interrupts(self.target_state);
    }
}

impl<M: RawMutex, B: I2c> interrupt::InterruptGuard for InterruptGuard<'_, M, B> {}

#[cfg(test)]
mod test {
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use embedded_hal_mock::eh1::i2c::Mock;
    use static_cell::StaticCell;

    use super::*;
    use crate::ADDR0;

    /// Tests `wait_interrupt_any` with a mask for both ports.
    #[tokio::test]
    async fn test_wait_interrupt_any_both() {
        static CONTROLLER: StaticCell<controller::Controller<NoopRawMutex, Mock>> = StaticCell::new();
        let controller = CONTROLLER.init(controller::Controller::new_tps66994(Mock::new(&[]), ADDR0).unwrap());
        let (mut pd, _interrupt) = controller.make_parts();

        let mut port0 = IntEventBus1::new_zero();
        port0.set_new_consumer_contract(true);
        port0.set_sink_ready(true);
        port0.set_cmd_1_completed(true);

        let mut port1 = IntEventBus1::new_zero();
        port1.set_plug_event(true);
        port1.set_alert_message_received(true);

        pd.controller.interrupt_waker.signal([port0, port1]);

        let mut mask0 = IntEventBus1::new_zero();
        mask0.set_cmd_1_completed(true);

        let mut mask1 = IntEventBus1::new_zero();
        mask1.set_plug_event(true);
        mask1.set_alert_message_received(true);

        let flags = pd.wait_interrupt_any(false, [mask0, mask1]).await;
        assert_eq!(flags, [mask0, mask1]);

        let mut unhandled0 = IntEventBus1::new_zero();
        unhandled0.set_new_consumer_contract(true);
        unhandled0.set_sink_ready(true);

        let unhandled1 = IntEventBus1::new_zero();

        // Should already be signaled
        assert_eq!(
            pd.controller.interrupt_waker.try_take().unwrap(),
            [unhandled0, unhandled1]
        );
    }

    /// Tests `wait_interrupt` with a mask for a single port.
    #[tokio::test]
    async fn test_wait_interrupt_any_single() {
        static CONTROLLER: StaticCell<controller::Controller<NoopRawMutex, Mock>> = StaticCell::new();
        let controller = CONTROLLER.init(controller::Controller::new_tps66994(Mock::new(&[]), ADDR0).unwrap());
        let (mut pd, _interrupt) = controller.make_parts();

        let mut port0 = IntEventBus1::new_zero();
        port0.set_new_consumer_contract(true);
        port0.set_sink_ready(true);
        port0.set_cmd_1_completed(true);

        let mut port1 = IntEventBus1::new_zero();
        port1.set_plug_event(true);
        port1.set_alert_message_received(true);

        pd.controller.interrupt_waker.signal([port0, port1]);

        let mut mask0 = IntEventBus1::new_zero();
        mask0.set_cmd_1_completed(true);

        let mask1 = IntEventBus1::new_zero();

        let flags = pd.wait_interrupt_any(false, [mask0, mask1]).await;
        assert_eq!(flags, [mask0, mask1]);

        let mut unhandled0 = IntEventBus1::new_zero();
        unhandled0.set_new_consumer_contract(true);
        unhandled0.set_sink_ready(true);

        let unhandled1 = port1;

        // Should already be signaled
        assert_eq!(
            pd.controller.interrupt_waker.try_take().unwrap(),
            [unhandled0, unhandled1]
        );
    }

    /// Tests `wait_interrupt` with both masks set to zero.
    #[tokio::test]
    async fn test_wait_interrupt_any_zero_masks() {
        static CONTROLLER: StaticCell<controller::Controller<NoopRawMutex, Mock>> = StaticCell::new();
        let controller = CONTROLLER.init(controller::Controller::new_tps66994(Mock::new(&[]), ADDR0).unwrap());
        let (mut pd, _interrupt) = controller.make_parts();

        let mut port0 = IntEventBus1::new_zero();
        port0.set_new_consumer_contract(true);
        port0.set_sink_ready(true);
        port0.set_cmd_1_completed(true);

        let mut port1 = IntEventBus1::new_zero();
        port1.set_plug_event(true);
        port1.set_alert_message_received(true);

        pd.controller.interrupt_waker.signal([port0, port1]);

        let mask0 = IntEventBus1::new_zero();
        let mask1 = IntEventBus1::new_zero();
        let flags = pd.wait_interrupt_any(false, [mask0, mask1]).await;
        assert_eq!(flags, [mask0, mask1]);

        // Should already be signaled with nothing changed
        assert_eq!(pd.controller.interrupt_waker.try_take().unwrap(), [port0, port1]);
    }
}
