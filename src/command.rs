use bincode::enc::Encoder;
use bincode::error::EncodeError;
use bincode::Encode;
use embedded_usb_pd::PdError;

const CMD_LEN: usize = 4;

/// Converts a 4-byte string into a u32
const fn u32_from_str(value: &str) -> u32 {
    if value.len() != CMD_LEN {
        panic!("Invalid command string")
    }

    let bytes = value.as_bytes();
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]).to_le()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u32)]
pub enum Command {
    /// Previous command succeeded
    Success = 0,
    /// Invalid Command
    Invalid = u32_from_str("!CMD"),
    /// Reset command
    Gaid = u32_from_str("GAID"),

    /// Tomcat firmware update mode enter
    Tfus = u32_from_str("TFUs"),
    /// Tomcat firmware update complete
    Tfuc = u32_from_str("TFUc"),
}

impl Command {
    /// Returns the delay in microseconds before checking that the command was valid
    pub fn valid_check_delay_us(self) -> u32 {
        match self {
            // Reset-type commands don't need to be checked
            Command::Success | Command::Invalid | Command::Gaid | Command::Tfus => 0,
            Command::Tfuc => 5000,
        }
    }
}

impl PartialEq<u32> for Command {
    fn eq(&self, other: &u32) -> bool {
        *self as u32 == *other
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ReturnValue {
    /// Success
    Success = 0x00,
    /// Timed-out or aborted with ABRT command
    Abort = 0x01,
    /// Rejected
    Rejected = 0x03,
    /// RX buffer locked
    RxLocked = 0x04,
    /// Task specific result
    Task0 = 0x05,
    /// Task specific result
    Task1 = 0x06,
    /// Task specific result
    Task2 = 0x07,
    /// Task specific result
    Task3 = 0x08,
    /// Task specific result
    Task4 = 0x09,
    /// Task specific result
    Task5 = 0x0A,
    /// Task specific result
    Task6 = 0x0B,
    /// Task specific result
    Task7 = 0x0C,
    /// Task specific result
    Task8 = 0x0D,
    /// Task specific result
    Task9 = 0x0E,
    /// Task specific result
    Task10 = 0x0F,
}

impl TryFrom<u8> for ReturnValue {
    type Error = PdError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(ReturnValue::Success),
            0x01 => Ok(ReturnValue::Abort),
            0x03 => Ok(ReturnValue::Rejected),
            0x04 => Ok(ReturnValue::RxLocked),
            0x05 => Ok(ReturnValue::Task0),
            0x06 => Ok(ReturnValue::Task1),
            0x07 => Ok(ReturnValue::Task2),
            0x08 => Ok(ReturnValue::Task3),
            0x09 => Ok(ReturnValue::Task4),
            0x0A => Ok(ReturnValue::Task5),
            0x0B => Ok(ReturnValue::Task6),
            0x0C => Ok(ReturnValue::Task7),
            0x0D => Ok(ReturnValue::Task8),
            0x0E => Ok(ReturnValue::Task9),
            0x0F => Ok(ReturnValue::Task10),
            _ => Err(PdError::InvalidParams),
        }
    }
}

#[allow(clippy::from_over_into)]
impl Into<Result<(), PdError>> for ReturnValue {
    fn into(self) -> Result<(), PdError> {
        match self {
            ReturnValue::Success => Ok(()),
            _ => Err(PdError::Failed),
        }
    }
}

pub(crate) const RESET_DELAY_MS: u32 = 1600;
pub(crate) const RESET_ARGS_LEN: usize = 2;
pub(crate) const RESET_FEATURE_ENABLE: u8 = 0xAC;

/// Arugments to reset-like commands
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ResetArgs {
    pub switch_banks: bool,
    pub copy_bank: bool,
}
impl Encode for ResetArgs {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let switch_banks = if self.switch_banks { RESET_FEATURE_ENABLE } else { 0 };
        Encode::encode(&switch_banks, encoder)?;
        let copy_bank = if self.copy_bank { RESET_FEATURE_ENABLE } else { 0 };
        Encode::encode(&copy_bank, encoder)
    }
}

/// Delay for completion of TFUs command
pub(crate) const TFUS_DELAY_MS: u32 = 500;

#[cfg(test)]
mod test {
    use bincode::config;

    use super::*;

    fn test_encode_reset_args(args: ResetArgs, expected: [u8; RESET_ARGS_LEN]) {
        let mut buf = [0; RESET_ARGS_LEN];
        bincode::encode_into_slice(args, &mut buf, config::standard().with_fixed_int_encoding()).unwrap();
        assert_eq!(buf, expected);
    }

    #[test]
    fn test_reset_args_encode() {
        test_encode_reset_args(ResetArgs::default(), [0, 0]);
        test_encode_reset_args(
            ResetArgs {
                switch_banks: true,
                copy_bank: false,
            },
            [0xAC, 0],
        );
        test_encode_reset_args(
            ResetArgs {
                switch_banks: false,
                copy_bank: true,
            },
            [0, 0xAC],
        );
        test_encode_reset_args(
            ResetArgs {
                switch_banks: true,
                copy_bank: true,
            },
            [0xAC, 0xAC],
        );
    }
}
