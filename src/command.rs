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
}

impl Command {
    /// Returns the delay in microseconds before checking that the command was valid
    pub fn valid_check_delay_us(self) -> u32 {
        match self {
            Command::Success | Command::Invalid => 0,
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
