#![no_main]
#![no_std]

use stm32_rust_energy_monitor as _; // global logger + panicking-behavior + memory layout

use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use crate::hal::{pac, prelude::*};

#[cortex_m_rt::entry]
fn main() -> ! {

    defmt::println!("Hello, world!");

    let dp = hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Set up the LED. On the BlackPill it's connected to pin PC13.
    let gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output();

    // Set up the system clock. We want to run at 48MHz for this one.
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

    // Create a delay abstraction based on SysTick
    let mut delay = cp.SYST.delay(&clocks);

    loop {
        // On for 1s, off for 1s.
        led.set_high();
        delay.delay_ms(1000_u32);
        led.set_low();
        delay.delay_ms(200_u32);
    }

}
