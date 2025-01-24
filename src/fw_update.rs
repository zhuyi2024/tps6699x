pub const HEADER_BLOCK_INDEX: usize = 0;
pub const DATA_BLOCK_START_INDEX: usize = 1;
pub const APP_CONFIG_BLOCK_INDEX: usize = 12;

pub const IMAGE_ID_LEN: usize = 4;

pub const APP_IMAGE_SIZE_OFFSET: usize = 0x4F8;

pub const HEADER_METADATA_OFFSET: usize = IMAGE_ID_LEN;
pub const HEADER_METADATA_LEN: usize = 8;
pub const HEADER_BLOCK_OFFSET: usize = HEADER_METADATA_OFFSET + HEADER_METADATA_LEN;
pub const HEADER_BLOCK_LEN: usize = 0x800;

pub const DATA_BLOCK_LEN: usize = 0x4000;
pub const DATA_BLOCK_METADATA_LEN: usize = 8;
pub const APP_CONFIG_METADATA_LEN: usize = 8;
pub const MAX_METADATA_LEN: usize = 8;

pub const TFUI_BURST_WRITE_DELAY_MS: u32 = 250;
pub const TFUD_BURST_WRITE_DELAY_MS: u32 = 150;
pub const BURST_WRITE_SIZE: usize = 256;
