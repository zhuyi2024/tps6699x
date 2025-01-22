#![no_std]

pub mod asynchronous;

/// I2C address set 0
pub const ADDR0: [u8; 2] = [0x20, 0x24];
/// I2C address set 1
pub const ADDR1: [u8; 2] = [0x21, 0x25];

pub const TPS66994_NUM_PORTS: usize = 2;
pub const TPS66993_NUM_PORTS: usize = 1;

pub mod registers {
    use device_driver;
    device_driver::create_device!(
        device_name: Registers,
        manifest: "device.yaml"
    );
}

/// Common unit test functions
#[cfg(test)]
pub(crate) mod test {
    extern crate std;
    use std::vec;
    use std::vec::Vec;

    use embedded_hal_mock::eh1::i2c::Transaction;

    /// Wrapper to easily create a register read transaction
    pub fn create_register_read<const N: usize, R: Into<[u8; N]>>(addr: u8, reg: u8, value: R) -> Transaction {
        // +1 for the length byte
        let mut response = Vec::with_capacity(N + 1);
        response.push(N as u8);
        response.splice(1..1, value.into().iter().cloned());

        Transaction::write_read(addr, vec![reg], response)
    }

    /// Wrapper to easily create a register write transaction
    pub fn create_register_write<const N: usize, R: Into<[u8; N]>>(addr: u8, reg: u8, value: R) -> Transaction {
        // +1 for the register + length byte
        let mut response = Vec::with_capacity(N + 2);
        response.push(reg);
        response.push(N as u8);
        response.splice(2..2, value.into().iter().cloned());

        Transaction::write(addr, response)
    }
}
