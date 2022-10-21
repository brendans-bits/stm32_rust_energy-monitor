#![no_main]
#![no_std]

use stm32_rust_energy_monitor as _; // global logger + panicking-behavior + memory layout

use stm32f4xx_hal as hal;

use crate::hal::{
    adc::{
        config::{AdcConfig, SampleTime},
        Adc,
    },
    prelude::*,
    serial::{Config, Serial},
};

use core::fmt::Write;

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Hello, world!");

    let dp = hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Set up the LED on pin PC13.
    let gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output();

    // Set up the system clock to run at 100MHz
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(100.MHz()).freeze();

    // Create a delay abstraction based on SysTick
    let mut delay = cp.SYST.delay(&clocks);

    // Set up serial tx pin
    let gpioa = dp.GPIOA.split();
    let tx_pin = gpioa.pa9.into_alternate();

    // Create a tx abstraction for serial transmission
    let mut tx = Serial::tx(
        dp.USART1,
        tx_pin,
        Config::default()
            .baudrate(115200.bps())
            .wordlength_8()
            .parity_none(),
        &clocks,
    )
    .unwrap();

    // Write a line to serial output
    writeln!(&mut tx, "Starting \r").unwrap();

    // Set up pin for voltage and current measurements
    let pin_voltage = gpioa.pa7.into_analog();
    let pin_current = gpioa.pa1.into_analog();

    // Set up ADC
    let mut adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());

    // Take a sample of the voltage and current measurements
    let sample_voltage = &adc.convert(&pin_voltage, SampleTime::Cycles_480);
    let sample_current = &adc.convert(&pin_current, SampleTime::Cycles_480);

    // Write the sample values to serial output
    writeln!(tx, "{} \r", sample_voltage).unwrap();
    writeln!(tx, "{} \r", sample_current).unwrap();

    loop {
        // On for 0.2s, off for 1s.
        led.set_high();
        delay.delay_ms(1000_u32);
        led.set_low();
        delay.delay_ms(200_u32);
        writeln!(&mut tx, "Blink \r").unwrap();
        
        //Pseudocode derived from emonlib
        // 1. Wait for the voltage sine wave to be roughly mid-cycle
        // We will measure voltage because this will always give a measureable sine wave even if there is little current flowing.
            // a. Start a timer
            // b. Start a free running loop
                // Break out of the loop if the voltage pin is between 0.45 and 0.55 of the ADC measurement range
                // Note: ATM32F4 has a 12-bit ADC (i.e. values between 0 and 4095), so break if the voltage is between 1842 and 2252
                // Break out of the loop if the timer is greater than the timeout
        // 2. Main measurement loop
            // a. Start a timer
            // b. Start a loop that runs until the voltage waveform that we are measuring has crossed zero a set number of times OR a timeout has been reached
                // Read raw ADC values for voltage and current
                // Remove the DC offset
                // Square and sum values for voltage and current
                // Apply phase calibration to voltage (to allow for phase delay in voltage transformer)
                // Calculate instantaneous power
                // update a counter if the voltage sine wave has crossed zero
        // 3. Post loop
            // a. Calculate voltage and current ratio values from calibration constants
            // b. Calculate RMS values for voltage and current
            // c. Calculate real power, apparent power, power factor
            // d. Reset voltage, current, and power accumulators        
    }
}
