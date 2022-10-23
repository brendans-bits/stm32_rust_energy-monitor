#![no_main]
#![no_std]

use stm32_rust_energy_monitor as _; // global logger + panicking-behavior + memory layout

use stm32f4xx_hal::{
    adc::{
        config::{AdcConfig, SampleTime},
        Adc,
    },
    gpio::*,
    //pac::*,
    prelude::*,
    //serial::{Config, Serial},
};

//use core::fmt::Write;

use fugit::*;

//use fugit_timer::*;

use libm::*;

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Hello, world!");

    //---------------------------------------
    // Calibration Constants
    //---------------------------------------
    let mut voltage_phase_calibration_value: f64 = 1.0;
    let mut voltage_calibration_value: f64 = 1.0;
    let mut voltage_correction_ratio: f64 = 1.0;
    let mut current_calibration_value: f64 = 1.0;
    let mut current_correction_ratio: f64 = 1.0;

    //
    // Variable definitions
    //

    // Count how many times the voltage waveform has crossed zero
    let mut zero_crossings_count: u32 = 0;
    // Number of times we want the voltage waveform to cross zero
    let zero_crossings_target: u32 = 100;
    let mut samples_count: u32;

    // Set timeout duration
    let timeout_duration: Duration<u32, 1, 1000000> = ExtU32::millis(2001);

    // Voltage sample - used in step 1
    let mut voltage_sample_initial: u16;

    let mut voltage_sample_measurement: u16;
    let mut voltage_adc_offset: i128;
    let mut voltage_sample_offset_removed: i128;
    let mut voltage_sample_squared: i128;
    let mut voltage_sample_sum: i128;
    let mut voltage_rms: f64;
    let mut voltage_phase_corrected: f64;
    let mut voltage_prior_sample_offset_removed: i128;

    let mut current_sample_measurement: u16;
    let mut current_adc_offset: i128;
    let mut current_sample_offset_removed: i128;
    let mut current_sample_squared: i128;
    let mut current_sample_sum: i128;
    let mut current_rms: f64;


    let mut power_sample_instantaneous: i128;
    let mut power_sample_sum: i128;

    let mut power_real: f64;
    let mut power_apparent: f64;
    let mut power_factor: f64;
    //
    // Set up peripherals
    //

    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Set up the LED on pin PC13.
    //let gpioc = dp.GPIOC.split();
    //let mut led = gpioc.pc13.into_push_pull_output();

    // Set up the system clock to run at 100MHz
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(fugit::RateExtU32::MHz(100)).freeze();

    // Create a delay abstraction based on SysTick
    let mut delay = cp.SYST.delay(&clocks);

    // Set up serial tx pin
    let gpioa = dp.GPIOA.split();
    //let tx_pin = gpioa.pa9.into_alternate();

    // Create a tx abstraction for serial transmission
    //let mut tx = Serial::tx(
    //    dp.USART1,
    //    tx_pin,
    //    Config::default()
    //        .baudrate(115200.bps())
    //        .wordlength_8()
    //        .parity_none(),
    //    &clocks,
    //)
    //.unwrap();

    // Create a timer that we use to measuer whether our measurements have exceeded allowable time
    let mut timeout_timer = dp.TIM5.counter_us(&clocks);

    // Write a line to serial output
    //writeln!(&mut tx, "Starting \r").unwrap();

    // Set up pin for voltage and current measurements
    let pin_voltage = gpioa.pa7.into_analog();
    let pin_current = gpioa.pa1.into_analog();

    // Set up ADC
    let mut adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());

    // Take a sample of the voltage and current measurements
    //let sample_voltage = &adc.convert(&pin_voltage, SampleTime::Cycles_480);
    //let sample_current = &adc.convert(&pin_current, SampleTime::Cycles_480);

    // Write the sample values to serial output
    //writeln!(tx, "{} \r", sample_voltage).unwrap();
    //writeln!(tx, "{} \r", sample_current).unwrap();

    loop {
        defmt::println!("Entering loop...");

        // On for 0.2s, off for 1s.
        //led.set_high();
        delay.delay_ms(1000_u32);
        //led.set_low();
        //delay.delay_ms(200_u32);
        //writeln!(&mut tx, "Blink \r").unwrap();

        // Pseudocode derived from emonlib

        // 1. Wait for the voltage sine wave to be roughly mid-cycle
        // We will measure voltage because this will always give a measureable sine wave even if there is little current flowing.

        // 1a. Start a timer
        timeout_timer.start(timeout_duration).unwrap();
        defmt::println!("Started timer...");

        // 1b. Start a free running loop
        loop {
            // With each iteration of the loop, sample the voltage pin
            voltage_sample_initial = adc.convert(&pin_voltage, SampleTime::Cycles_3);

            // Break out of the loop if the timer is greater than the timeout
            if timeout_timer.now().duration_since_epoch()
                > timeout_duration - fugit::ExtU32::millis(1)
            {
                defmt::println!("Reached timeout...");
                break;
            }

            // Break out of the loop if the voltage pin is between 0.45 and 0.55 of the ADC measurement range
            // Note: ATM32F4 has a 12-bit ADC (i.e. values between 0 and 4095), so break if the voltage is between 1842 and 2252
            if 4095.0 * 0.55 > voltage_sample_initial as f64
                && 4095.0 * 0.45 < voltage_sample_initial as f64
            {
                timeout_timer.cancel().unwrap();
                defmt::println!("Reached midpoint...");
                break;
            }
        }

        // 2. Main measurement loop
        // 2a. Start a timer
        timeout_timer.start(timeout_duration).unwrap();
        defmt::println!("Started timer...");

        voltage_adc_offset = 0;
        current_adc_offset = 0;
        voltage_sample_sum = 0;
        current_sample_sum = 0;
        power_sample_sum = 0;
        samples_count = 0;
        voltage_prior_sample_offset_removed = 0;
        // 2b. Start a loop that runs until the voltage waveform that we are measuring has crossed zero a set number of times OR a timeout has been reached
        while (timeout_timer.now().duration_since_epoch()
            < timeout_duration - fugit::ExtU32::millis(1))
        {
            // Read raw ADC values for voltage and current
            voltage_sample_measurement = adc.convert(&pin_voltage, SampleTime::Cycles_3);
            current_sample_measurement = adc.convert(&pin_current, SampleTime::Cycles_3);
            // Calculate DC offset
            voltage_adc_offset = voltage_adc_offset
                + ((voltage_sample_measurement as i128 - voltage_adc_offset) / 1024);
            current_adc_offset = current_adc_offset
                + ((current_sample_measurement as i128 - current_adc_offset) / 1024);
            // Remove DC offset from voltage and current values
            voltage_sample_offset_removed = voltage_sample_measurement as i128 - voltage_adc_offset;
            current_sample_offset_removed = current_sample_measurement as i128 - current_adc_offset;
            // Square values for voltage and current
            voltage_sample_squared = voltage_sample_offset_removed * voltage_sample_offset_removed;
            current_sample_squared = current_sample_offset_removed * current_sample_offset_removed;
            // Sum values for voltage and current
            voltage_sample_sum = voltage_sample_sum + voltage_sample_squared;
            current_sample_sum = current_sample_sum + current_sample_squared;
            // Apply phase calibration to voltage (to allow for phase delay in voltage transformer)
            voltage_phase_corrected = voltage_prior_sample_offset_removed as f64
                + voltage_phase_calibration_value
                    * (voltage_sample_offset_removed - voltage_prior_sample_offset_removed) as f64;
            // Calculate instantaneous power
            // NEED TO CHANGE THIS FROM voltage_sample_offset_removed to phase corrected voltage sample
            power_sample_instantaneous = voltage_phase_corrected as i128 * current_sample_offset_removed;
            // Sum of instantaneous power
            power_sample_sum = power_sample_sum + power_sample_instantaneous;
            // Count the number of samples we have taken
            samples_count = samples_count + 1;
            // Store previous offset-corrected voltage for use in the next iteration of the loop
            voltage_prior_sample_offset_removed = voltage_sample_offset_removed;
            // update a counter if the voltage sine wave has crossed zero
            // TO DO
        }

        // 3. Post loop
        // 3a. Calculate voltage and current ratio values from calibration constants
        // NEED TO INCLUDE VOLTAGE AND CURRENT CALIBRATION CONSTANTS
        // voltage_correction_ratio = voltage_calibration_value * ((VCC / 1000) / 1<<12);
        // current_correction_ratio = current_calibration_value * ((VCC / 1000) / 1<<12);
        // 3b. Calculate RMS values for voltage and current
        // NEED TO MULTIPLE THESE BY THE VOLTAGE AND CURRENT RATIO VALUES
        voltage_rms = sqrt(voltage_sample_sum as f64 / samples_count as f64);
        //voltage_rms = voltage_correction_ratio * sqrt(voltage_sample_sum as f64 / samples_count as f64);
        current_rms = sqrt(current_sample_sum as f64 / samples_count as f64);
        //current_rms = current_correction_ratio * sqrt(current_sample_sum as f64 / samples_count as f64);
        defmt::println!("RMS Voltage {} \r", voltage_rms);
        defmt::println!("RMS Current {} \r", current_rms);
        // 3c. Calculate real power, apparent power, power factor
        //NEED TO MULTIPLY THIS BY VOLTAGE AND CURRENT CALIBRATION VALUES
        power_real = power_sample_sum as f64 / samples_count as f64;
        // power_real = voltage_correction_ratio * current_correction_ratio * power_sample_sum as f64 / samples_count as f64;
        power_apparent = voltage_rms as f64 * current_rms as f64;
        power_factor = power_real / power_apparent;
        defmt::println!("real power {} \r", power_real);
        defmt::println!("apparent power {} \r", power_apparent);
        defmt::println!("power factor {} \r", power_factor);
    }
}
