//! This module implements functions to access the command register and its associate data register.
//! The data register is larger than what device_driver can handle so access is done directly through the `AsyncRegisterInterface` trait.
use bincode::config;
use device_driver::AsyncRegisterInterface;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use embedded_usb_pd::{Error, PdError, PortId};

use super::Tps6699x;
use crate::command::*;
use crate::{error, info, registers as regs, Mode, PORT0};

impl<B: I2c> Tps6699x<B> {
    /// Sends a command without verifying that it is valid
    pub async fn send_command_unchecked(
        &mut self,
        port: PortId,
        cmd: Command,
        data: Option<&[u8]>,
    ) -> Result<(), Error<B::Error>> {
        let mut registers = self.borrow_port(port)?.into_registers();

        if let Some(data) = data {
            registers
                .interface()
                .write_register(regs::REG_DATA1, (data.len() * 8) as u32, data)
                .await?;
        }

        registers.cmd_1().write_async(|r| r.set_command(cmd as u32)).await?;

        Ok(())
    }

    /// Sends a command, verifying that it is valid
    pub async fn send_command(
        &mut self,
        delay: &mut impl DelayNs,
        port: PortId,
        cmd: Command,
        data: Option<&[u8]>,
    ) -> Result<(), Error<B::Error>> {
        self.send_command_unchecked(port, cmd, data).await?;

        delay.delay_us(cmd.valid_check_delay_us()).await;
        if Command::Invalid
            == self
                .borrow_port(port)?
                .into_registers()
                .cmd_1()
                .read_async()
                .await?
                .command()
        {
            error!("send_command: Unrecognized Command");
            return PdError::UnrecognizedCommand.into();
        }

        Ok(())
    }

    /// Check if the command has completed
    pub async fn check_command_complete(&mut self, port: PortId) -> Result<bool, Error<B::Error>> {
        let mut registers = self.borrow_port(port)?.into_registers();
        let status = registers.cmd_1().read_async().await?.command();

        Ok(Command::Success == status)
    }

    /// Read the result of a command
    pub async fn read_command_result(
        &mut self,
        port: PortId,
        data: Option<&mut [u8]>,
    ) -> Result<ReturnValue, Error<B::Error>> {
        if !self.check_command_complete(port).await? {
            error!("read_command_result: Busy");
            return PdError::Busy.into();
        }

        if let Some(ref data) = data {
            if data.len() > regs::REG_DATA1_LEN - 1 {
                // Data length too long
                error!("read_command_result: Data too long");
                return PdError::InvalidParams.into();
            }
        }

        // Read and return value and data
        let mut buf = [0u8; regs::REG_DATA1_LEN];
        self.borrow_port(port)?
            .into_registers()
            .interface()
            .read_register(regs::REG_DATA1, (regs::REG_DATA1_LEN * 8) as u32, &mut buf)
            .await?;

        info!("read_command_result: buf[0]: {:?}", buf[0]);
        let return_code = buf[0] & 0x0F;
        info!("read_command_result: return_code: {:?}", return_code);
        let ret = ReturnValue::try_from(return_code).map_err(Error::Pd)?;
        info!("read_command_result: ret: {:?}", ret);
        // Overwrite return value
        /*if let Some(data) = data {
            data.copy_from_slice(&buf[1..=data.len()]);
        }*/

        Ok(ret)
    }

    /// Reset the controller
    pub async fn reset(&mut self, delay: &mut impl DelayNs, args: &ResetArgs) -> Result<(), Error<B::Error>> {
        // This is a controller-level command, shouldn't matter which port we use
        let mut arg_bytes = [0u8; RESET_ARGS_LEN];

        bincode::encode_into_slice(args, &mut arg_bytes, config::standard().with_fixed_int_encoding())
            .map_err(|_| Error::Pd(PdError::Serialize))?;
        self.send_command_unchecked(PORT0, Command::Gaid, Some(&arg_bytes))
            .await?;

        delay.delay_ms(RESET_DELAY_MS).await;

        Ok(())
    }

    /// Enter firmware update mode
    pub async fn execute_tfus(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<B::Error>> {
        // This is a controller-level command, shouldn't matter which port we use
        self.send_command_unchecked(PORT0, Command::Tfus, None).await?;

        delay.delay_ms(TFUS_DELAY_MS).await;

        // Confirm we're in the correct mode
        let mode = self.get_mode().await?;
        if mode != Mode::F211 {
            error!("Failed to enter firmware update mode, mode: {:?}", mode);
            return Err(PdError::InvalidMode.into());
        }
        Ok(())
    }

    /// Complete firmware update
    pub async fn execute_tfuc(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<B::Error>> {
        let mut arg_bytes = [0u8; RESET_ARGS_LEN];

        let args = ResetArgs {
            switch_banks: false,
            copy_bank: true,
        };

        bincode::encode_into_slice(args, &mut arg_bytes, config::standard().with_fixed_int_encoding())
            .map_err(|_| Error::Pd(PdError::Serialize))?;

        // This is a controller-level command, shouldn't matter which port we use
        let port = PortId(0);
        self.send_command(delay, port, Command::Tfuc, Some(&arg_bytes)).await?;

        delay.delay_ms(RESET_DELAY_MS).await;

        // Confirm we're in the correct mode
        let mode = self.get_mode().await?;
        if mode != Mode::App0 && mode != Mode::App1 {
            error!("Failed to enter normal mode, mode: {:?}", mode);
            return Err(PdError::InvalidMode.into());
        }

        Ok(())
    }
}

#[cfg(test)]
mod test {
    use embedded_hal_mock::eh1::i2c::Mock;
    use regs::REG_DATA1;

    use crate::asynchronous::internal::Tps6699x;
    use crate::{ADDR0, ADDR1, PORT0};

    extern crate std;
    use std::vec::Vec;

    use super::*;
    use crate::test::*;

    /// Value used for generic command testing, no particular significance
    const TEST_CMD_DATA: u64 = 0x12345678abcdef;

    async fn test_send_command<const N: usize>(
        tps6699x: &mut Tps6699x<Mock>,
        expected_addr: u8,
        expected_cmd: Command,
        expected_data: Option<[u8; N]>,
    ) {
        let mut delay = Delay {};
        let mut transactions = Vec::new();

        // Create data write if supplied
        if let Some(data) = expected_data {
            transactions.push(create_register_write(expected_addr, REG_DATA1, data));
        }

        transactions.push(create_register_write(
            expected_addr,
            0x08,
            (expected_cmd as u32).to_le_bytes(),
        ));
        // Check that the command was valid
        transactions.push(create_register_read(expected_addr, 0x08, [0u8; 4]));
        tps6699x.bus.update_expectations(&transactions);

        if let Some(data) = expected_data {
            tps6699x
                .send_command(&mut delay, PORT0, expected_cmd, Some(&data))
                .await
                .unwrap();
        } else {
            tps6699x
                .send_command(&mut delay, PORT0, expected_cmd, None)
                .await
                .unwrap();
        }

        tps6699x.bus.done();
    }

    #[tokio::test]
    async fn test_send_command_0() {
        let mut tps6699x = Tps6699x::new_tps66994(Mock::new(&[]), ADDR0);
        test_send_command::<0>(&mut tps6699x, PORT0_ADDR0, Command::Invalid, None).await;
        test_send_command(
            &mut tps6699x,
            PORT0_ADDR0,
            Command::Invalid,
            Some(TEST_CMD_DATA.to_le_bytes()),
        )
        .await;
    }

    #[tokio::test]
    async fn test_send_command_1() {
        let mut tps6699x = Tps6699x::new_tps66994(Mock::new(&[]), ADDR1);
        test_send_command::<0>(&mut tps6699x, PORT0_ADDR1, Command::Invalid, None).await;
        test_send_command(
            &mut tps6699x,
            PORT0_ADDR1,
            Command::Invalid,
            Some(TEST_CMD_DATA.to_le_bytes()),
        )
        .await;
    }

    async fn test_reset(tps6699x: &mut Tps6699x<Mock>, expected_addr: u8, expected_args: ResetArgs) {
        let mut delay = Delay {};
        let mut transactions = Vec::new();

        let mut arg_bytes = [0u8; RESET_ARGS_LEN];
        bincode::encode_into_slice(
            &expected_args,
            &mut arg_bytes,
            config::standard().with_fixed_int_encoding(),
        )
        .unwrap();

        transactions.push(create_register_write(expected_addr, REG_DATA1, arg_bytes));
        transactions.push(create_register_write(
            expected_addr,
            0x08,
            (Command::Gaid as u32).to_le_bytes(),
        ));
        tps6699x.bus.update_expectations(&transactions);

        tps6699x.reset(&mut delay, &expected_args).await.unwrap();
        tps6699x.bus.done();
    }

    #[tokio::test]
    async fn test_reset_0() {
        let mut tps6699x = Tps6699x::new_tps66994(Mock::new(&[]), ADDR0);
        test_reset(
            &mut tps6699x,
            PORT0_ADDR0,
            ResetArgs {
                switch_banks: true,
                copy_bank: false,
            },
        )
        .await;
    }

    #[tokio::test]
    async fn test_reset_1() {
        let mut tps6699x = Tps6699x::new_tps66994(Mock::new(&[]), ADDR1);
        test_reset(
            &mut tps6699x,
            PORT0_ADDR1,
            ResetArgs {
                switch_banks: true,
                copy_bank: false,
            },
        )
        .await;
    }

    async fn test_execute_tfus(tps6699x: &mut Tps6699x<Mock>, expected_addr: u8) {
        let mut delay = Delay {};
        let mut transactions = Vec::new();

        transactions.push(create_register_write(
            expected_addr,
            0x08,
            (Command::Tfus as u32).to_le_bytes(),
        ));
        transactions.push(create_register_read(
            expected_addr,
            0x03,
            (Mode::F211 as u32).to_le_bytes(),
        ));
        tps6699x.bus.update_expectations(&transactions);

        tps6699x.execute_tfus(&mut delay).await.unwrap();
        tps6699x.bus.done();
    }

    #[tokio::test]
    async fn test_execute_tfus_0() {
        let mut tps6699x = Tps6699x::new_tps66994(Mock::new(&[]), ADDR0);
        test_execute_tfus(&mut tps6699x, PORT0_ADDR0).await;
    }

    #[tokio::test]
    async fn test_execute_tfus_1() {
        let mut tps6699x = Tps6699x::new_tps66994(Mock::new(&[]), ADDR1);
        test_execute_tfus(&mut tps6699x, PORT0_ADDR1).await;
    }

    async fn test_execute_tfuc(tps6699x: &mut Tps6699x<Mock>, expected_addr: u8) {
        let mut delay = Delay {};
        let mut transactions = Vec::new();

        transactions.push(create_register_write(
            expected_addr,
            REG_DATA1,
            [0, RESET_FEATURE_ENABLE],
        ));
        transactions.push(create_register_write(
            expected_addr,
            0x08,
            (Command::Tfuc as u32).to_le_bytes(),
        ));
        transactions.push(create_register_read(expected_addr, 0x08, [0; 4]));
        transactions.push(create_register_read(
            expected_addr,
            0x03,
            (Mode::App0 as u32).to_le_bytes(),
        ));
        tps6699x.bus.update_expectations(&transactions);

        tps6699x.execute_tfuc(&mut delay).await.unwrap();
        tps6699x.bus.done();
    }

    #[tokio::test]
    async fn test_execute_tfuc_0() {
        let mut tps6699x = Tps6699x::new_tps66994(Mock::new(&[]), ADDR0);
        test_execute_tfuc(&mut tps6699x, PORT0_ADDR0).await;
    }

    #[tokio::test]
    async fn test_execute_tfuc_1() {
        let mut tps6699x = Tps6699x::new_tps66994(Mock::new(&[]), ADDR1);
        test_execute_tfuc(&mut tps6699x, PORT0_ADDR1).await;
    }
}
