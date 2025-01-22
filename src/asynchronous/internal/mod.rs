//! Asynchronous TPS6699x driver
use embedded_hal_async::i2c::I2c;
use embedded_usb_pd::{Error, PdError, PortId};

use crate::registers;

/// Wrapper to allow implementing device_driver traits on our I2C bus
pub struct Port<'a, B: I2c> {
    bus: &'a mut B,
    addr: u8,
}

impl<'a, B: I2c> Port<'a, B> {
    pub fn into_registers(self) -> registers::Registers<Port<'a, B>> {
        registers::Registers::new(self)
    }
}

impl<B: I2c> device_driver::AsyncRegisterInterface for Port<'_, B> {
    type Error = Error<B::Error>;

    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        // Sized to accommodate up to 255 bytes of data
        let mut buf = [0u8; 257];

        // Buffer length is sent as a byte
        if data.len() > 255 {
            return Err(PdError::InvalidParams.into());
        }

        buf[0] = address;
        buf[1] = data.len() as u8;
        let _ = &buf[2..data.len() + 2].copy_from_slice(data);

        self.bus
            .write(self.addr, &buf[..data.len() + 2])
            .await
            .map_err(Error::Bus)
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        // Sized to accommodate length byte + up to 255 bytes of data
        let mut buf = [0u8; 256];
        let full_len = data.len() + 1;
        let reg = [address];

        if data.is_empty() {
            return Err(PdError::InvalidParams.into());
        }

        self.bus
            .write_read(self.addr, &reg, &mut buf[..full_len])
            .await
            .map_err(Error::Bus)?;

        let len = buf[0] as usize;
        if len > data.len() {
            PdError::InvalidParams.into()
        } else if len == 0xff || len == 0 {
            // Controller is busy and can't respond
            PdError::Busy.into()
        } else {
            data.copy_from_slice(&buf[1..len + 1]);
            Ok(())
        }
    }
}

/// Low-level TSP6699x driver, generic over number of ports (N) and I2C bus (B)
pub struct Tps6699x<const N: usize, B: I2c> {
    bus: B,
    /// I2C addresses for ports
    addr: [u8; N],
}

impl<const N: usize, B: I2c> Tps6699x<N, B> {
    pub fn new(bus: B, addr: [u8; N]) -> Self {
        Self { bus, addr }
    }

    /// Get the I2C address for a port
    fn port_addr(&self, port: PortId) -> Result<u8, Error<B::Error>> {
        if port.0 as usize > self.addr.len() {
            PdError::InvalidPort.into()
        } else {
            Ok(self.addr[port.0 as usize])
        }
    }

    /// Borrows the given port, providing exclusive access to it and therefore the underlying bus object
    pub fn borrow_port(&mut self, port: PortId) -> Result<Port<'_, B>, Error<B::Error>> {
        let addr = self.port_addr(port)?;
        Ok(Port {
            bus: &mut self.bus,
            addr,
        })
    }

    /// Clear interrupts on a port, returns asserted interrupts
    pub async fn clear_interrupt(
        &mut self,
        port: PortId,
    ) -> Result<registers::field_sets::IntEventBus1, Error<B::Error>> {
        let p = self.borrow_port(port)?;
        let mut registers = p.into_registers();

        let flags = registers.int_event_bus_1().read_async().await?;
        // Clear interrupt if anything is set
        if flags != registers::field_sets::IntEventBus1::new_zero() {
            registers.int_clear_bus_1().write_async(|r| *r = flags).await?;
        }

        Ok(flags)
    }

    /// Get port status
    pub async fn get_port_status(&mut self, port: PortId) -> Result<registers::field_sets::Status, Error<B::Error>> {
        self.borrow_port(port)?.into_registers().status().read_async().await
    }

    /// Get active PDO contract
    pub async fn get_active_pdo_contract(
        &mut self,
        port: PortId,
    ) -> Result<registers::field_sets::ActivePdoContract, Error<B::Error>> {
        self.borrow_port(port)?
            .into_registers()
            .active_pdo_contract()
            .read_async()
            .await
    }
}

#[cfg(test)]
mod test {
    extern crate std;
    use std::vec::Vec;

    use device_driver::AsyncRegisterInterface;
    use embedded_hal_async::i2c::ErrorType;
    use embedded_hal_mock::eh1::i2c::Mock;

    use super::*;
    use crate::test::*;
    use crate::{ADDR0, ADDR1, TPS66994_NUM_PORTS};

    const PORT0: PortId = PortId(0);
    const PORT1: PortId = PortId(1);

    const PORT0_ADDR0: u8 = ADDR0[0];
    const PORT1_ADDR0: u8 = ADDR0[1];
    const PORT0_ADDR1: u8 = ADDR1[0];
    const PORT1_ADDR1: u8 = ADDR1[1];

    // Use dual-port version to fully test port-specific code
    type Tps66994<B> = Tps6699x<TPS66994_NUM_PORTS, B>;

    #[test]
    fn test() {
        create_register_read(0, 0, [0]);
    }

    async fn test_read_port<const N: usize>(
        tps6699x: &mut Tps66994<Mock>,
        port_id: PortId,
        expected_addr: u8,
        reg: u8,
        expected: [u8; N],
    ) -> Result<(), Error<<Mock as ErrorType>::Error>> {
        tps6699x
            .bus
            .update_expectations(&[create_register_read(expected_addr, reg, expected)]);

        let mut port = tps6699x.borrow_port(port_id)?;
        let mut result = [0; N];
        port.read_register(reg, (expected.len() * 8) as u32, &mut result)
            .await?;
        tps6699x.bus.done();

        Ok(())
    }

    async fn test_write_port<const N: usize>(
        tps6699x: &mut Tps66994<Mock>,
        port_id: PortId,
        expected_addr: u8,
        reg: u8,
        expected: [u8; N],
    ) -> Result<(), Error<<Mock as ErrorType>::Error>> {
        tps6699x
            .bus
            .update_expectations(&[create_register_write(expected_addr, reg, expected)]);

        let mut port = tps6699x.borrow_port(port_id)?;
        port.write_register(reg, (expected.len() * 8) as u32, &expected).await?;
        tps6699x.bus.done();

        Ok(())
    }

    async fn test_rw_port<const N: usize>(
        tps6699x: &mut Tps66994<Mock>,
        port_id: PortId,
        expected_addr: u8,
        reg: u8,
        expected: [u8; N],
    ) -> Result<(), Error<<Mock as ErrorType>::Error>> {
        test_read_port::<N>(tps6699x, port_id, expected_addr, reg, expected).await?;
        test_write_port::<N>(tps6699x, port_id, expected_addr, reg, expected).await
    }

    async fn test_rw_ports(tps6699x: &mut Tps66994<Mock>, port_id: PortId, expected_addr: u8) {
        // No particular signifigance to these values, just testing a mix of values
        test_rw_port(tps6699x, port_id, expected_addr, 0x00, [0x01, 0x02])
            .await
            .unwrap();
        test_rw_port(tps6699x, port_id, expected_addr, 0x20, [0x87, 0x2, 0x7])
            .await
            .unwrap();
        test_rw_port(tps6699x, port_id, expected_addr, 0x05, [0xff, 0xa7, 0x79, 0x87])
            .await
            .unwrap();
    }

    /// Test on the first set of I2C addresses
    #[tokio::test]
    async fn test_rw_ports_0() {
        let mock = Mock::new(&[]);
        let mut tps6699x: Tps66994<Mock> = Tps6699x::new(mock, ADDR0);

        test_rw_ports(&mut tps6699x, PORT0, PORT0_ADDR0).await;
        test_rw_ports(&mut tps6699x, PORT1, PORT1_ADDR0).await;
    }

    /// Test on the second set of I2C addresses
    #[tokio::test]
    async fn test_rw_ports_1() {
        let mock = Mock::new(&[]);
        let mut tps6699x: Tps66994<Mock> = Tps6699x::new(mock, ADDR1);

        test_rw_ports(&mut tps6699x, PORT0, PORT0_ADDR1).await;
        test_rw_ports(&mut tps6699x, PORT1, PORT1_ADDR1).await;
    }

    async fn test_clear_interrupt(tps6699x: &mut Tps66994<Mock>, port: PortId, expected_addr: u8) {
        use registers::field_sets::IntEventBus1;

        // Create a fully asserted interrupt register
        let int = !IntEventBus1::new_zero();
        let mut transactions = Vec::new();

        // Read the interrupt register
        transactions.push(create_register_read(expected_addr, 0x14, int));

        // Write to the interrupt clear register
        transactions.push(create_register_write(expected_addr, 0x18, int));
        tps6699x.bus.update_expectations(&transactions);

        assert_eq!(tps6699x.clear_interrupt(port).await.unwrap(), int);
        tps6699x.bus.done();
    }

    /// Test clearing interrupts with address set 0
    #[tokio::test]
    async fn test_clear_interrupt_0() {
        let mock = Mock::new(&[]);
        let mut tps6699x: Tps66994<Mock> = Tps6699x::new(mock, ADDR0);

        test_clear_interrupt(&mut tps6699x, PORT0, PORT0_ADDR0).await;
        test_clear_interrupt(&mut tps6699x, PORT1, PORT1_ADDR0).await;
    }

    /// Test clearing interrupts with address set 0
    #[tokio::test]
    async fn test_clear_interrupt_1() {
        let mock = Mock::new(&[]);
        let mut tps6699x: Tps66994<Mock> = Tps6699x::new(mock, ADDR1);

        test_clear_interrupt(&mut tps6699x, PORT0, PORT0_ADDR1).await;
        test_clear_interrupt(&mut tps6699x, PORT1, PORT1_ADDR1).await;
    }

    async fn test_get_port_status(tps6699x: &mut Tps66994<Mock>, port: PortId, expected_addr: u8) {
        use registers::field_sets::Status;

        let mut transactions = Vec::new();
        // Read status register
        transactions.push(create_register_read(expected_addr, 0x1A, Status::new_zero()));
        tps6699x.bus.update_expectations(&transactions);

        let status = tps6699x.get_port_status(port).await.unwrap();
        assert_eq!(status, Status::new_zero());
        tps6699x.bus.done();
    }

    /// Test get port status on address set 0
    #[tokio::test]
    async fn test_get_port_status_0() {
        let mock = Mock::new(&[]);
        let mut tps6699x: Tps66994<Mock> = Tps6699x::new(mock, ADDR0);

        test_get_port_status(&mut tps6699x, PORT0, PORT0_ADDR0).await;
        test_get_port_status(&mut tps6699x, PORT1, PORT1_ADDR0).await;
    }

    /// Test get port status on address set 1
    #[tokio::test]
    async fn test_get_port_status_1() {
        let mock = Mock::new(&[]);
        let mut tps6699x: Tps66994<Mock> = Tps6699x::new(mock, ADDR1);

        test_get_port_status(&mut tps6699x, PORT0, PORT0_ADDR1).await;
        test_get_port_status(&mut tps6699x, PORT1, PORT1_ADDR1).await;
    }

    async fn test_get_active_pdo_contract(tps6699x: &mut Tps66994<Mock>, port: PortId, expected_addr: u8) {
        use registers::field_sets::ActivePdoContract;

        let mut transactions = Vec::new();
        // Read status register
        transactions.push(create_register_read(expected_addr, 0x34, ActivePdoContract::new_zero()));
        tps6699x.bus.update_expectations(&transactions);

        let active_pdo_contract = tps6699x.get_active_pdo_contract(port).await.unwrap();
        assert_eq!(active_pdo_contract, ActivePdoContract::new_zero());
        tps6699x.bus.done();
    }

    #[tokio::test]
    async fn test_get_active_pdo_contract_ports_0() {
        let mock = Mock::new(&[]);
        let mut tps6699x: Tps66994<Mock> = Tps6699x::new(mock, ADDR0);

        test_get_active_pdo_contract(&mut tps6699x, PORT0, PORT0_ADDR0).await;
        test_get_active_pdo_contract(&mut tps6699x, PORT1, PORT1_ADDR0).await;
    }

    #[tokio::test]
    async fn test_get_active_pdo_contract_ports_1() {
        let mock = Mock::new(&[]);
        let mut tps6699x: Tps66994<Mock> = Tps6699x::new(mock, ADDR1);

        test_get_active_pdo_contract(&mut tps6699x, PORT0, PORT0_ADDR1).await;
        test_get_active_pdo_contract(&mut tps6699x, PORT1, PORT1_ADDR1).await;
    }
}
