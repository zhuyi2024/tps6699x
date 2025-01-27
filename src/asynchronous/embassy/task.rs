use embassy_sync::blocking_mutex::raw::RawMutex;
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;

use super::Interrupt;
use crate::{error, warn};

/// Task to process all given interrupts
pub async fn interrupt_task<const N: usize, M: RawMutex, B: I2c, INT: Wait + InputPin>(
    int: &mut INT,
    mut interrupts: [&mut Interrupt<'_, M, B>; N],
) {
    loop {
        if int.wait_for_low().await.is_err() {
            error!("Error waiting for interrupt");
            continue;
        }

        for interrupt in &mut interrupts {
            if interrupt.process_interrupt(int).await.is_err() {
                warn!("Error processing interrupt");
            }
        }
    }
}
