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
    // I have removed the need to apply a phase calibration for two reasons
    // 1 - the toroidal transformer I am using has so little phase error that I don't seem to gain anything from applying a correction.
    // 2 - I don't fully understand the equation provided in emonlib
    // NOTE: If you want to use the phase correction, you will need to change some of the variable types
    // i.e. change some variables from i128 to f64
    //let voltage_phase_calibration_value: f64 = 0.00; // Depends on which transformer I am using
    let voltage_calibration_value: f64 = 254.4; // rough value only!
    let current_1_calibration_value: f64 = 18.45; // rough value only!
    let current_2_calibration_value: f64 = 55.35; // rough value only!

    //----------------------------------------
    // Variable definitions
    //----------------------------------------

    // Count how many times the voltage waveform has crossed zero
    //let mut zero_crossings_count: u32 = 0;
    // Number of times we want the voltage waveform to cross zero
    //let zero_crossings_target: u32 = 100;

    let mut samples_count: i128;

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
    //let mut voltage_phase_corrected: f64;
    //let mut voltage_prior_sample_offset_removed: i128;
    let mut voltage_correction_ratio: f64;

    let mut current_1_sample_measurement: u16;
    let mut current_1_adc_offset: i128;
    let mut current_1_sample_offset_removed: i128;
    let mut current_1_sample_squared: i128;
    let mut current_1_sample_sum: i128;
    let mut current_1_rms: f64;
    let mut current_1_correction_ratio: f64;

    let mut power_1_sample_instantaneous: i128;
    let mut power_1_sample_sum: i128;
    let mut power_1_real: f64;
    let mut power_1_apparent: f64;
    let mut power_1_factor: f64;

    let mut current_2_sample_measurement: u16;
    let mut current_2_adc_offset: i128;
    let mut current_2_sample_offset_removed: i128;
    let mut current_2_sample_squared: i128;
    let mut current_2_sample_sum: i128;
    let mut current_2_rms: f64;
    let mut current_2_correction_ratio: f64;

    let mut power_2_sample_instantaneous: i128;
    let mut power_2_sample_sum: i128;
    let mut power_2_real: f64;
    let mut power_2_apparent: f64;
    let mut power_2_factor: f64;

    //----------------------------------
    // Set up peripherals
    //----------------------------------

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
    //        .baudrate(1152000.bps())
    //        .wordlength_8()
    //        .parity_none(),
    //    &clocks,
    //)
    //.unwrap();

    // Create a timer that we use to measuer whether our measurements have exceeded allowable time
    let timer = dp.TIM5;
    let mut timeout_timer = timer.counter_us(&clocks);

    // Write a line to serial output
    // writeln!(&mut tx, "Starting \r").unwrap();

    //writeln!(&mut tx, "counter, current_sample_offset_removed, voltage_sample_offset_removed, voltage_phase_corrected \r").unwrap();

    // Set up pin for voltage and current measurements
    let pin_voltage = gpioa.pa7.into_analog();
    let pin_current_1 = gpioa.pa1.into_analog();
    let pin_current_2 = gpioa.pa4.into_analog();

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

        //-----------------------------------------
        // 1. Wait for the voltage sine wave to be roughly mid-cycle
        //------------------------------------------

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

        //-------------------------------------
        // 2. Main measurement loop
        //-------------------------------------

        // 2a. Reset some variables used in the loop
        voltage_adc_offset = 2048; // This assumes that you are adding a DC offset that is VCC/2 and a 12 bit ADC
        current_1_adc_offset = 2048;
        current_2_adc_offset = 2176; // My voltages are not quite accurate yet
        voltage_sample_sum = 0;
        current_1_sample_sum = 0;
        current_2_sample_sum = 0;
        power_1_sample_sum = 0;
        power_2_sample_sum = 0;
        samples_count = 0;
        //voltage_prior_sample_offset_removed = 0;

        // 2b. Start a timer
        timeout_timer.start(timeout_duration).unwrap();
        //defmt::println!("Started timer...");

        // 2c. Start a loop that runs until a timeout has been reached
        while timeout_timer.now().duration_since_epoch()
            < timeout_duration - fugit::ExtU32::millis(1500)
        {
            //let counter = timeout_timer.now().duration_since_epoch().ticks();
            //defmt::println!("timer count {:?}", counter );
            //let counter = timer.counter_us(&clocks).now().duration_since_epoch().ticks();

            // Read raw ADC values for voltage and current
            voltage_sample_measurement = adc.convert(&pin_voltage, SampleTime::Cycles_3);
            current_1_sample_measurement = adc.convert(&pin_current_1, SampleTime::Cycles_3);
            current_2_sample_measurement = adc.convert(&pin_current_2, SampleTime::Cycles_3);

            // Adjust DC offset
            // Unlike emonlib, we are pre-seeding a value here
            voltage_adc_offset = voltage_adc_offset
                + ((voltage_sample_measurement as i128 - voltage_adc_offset) / 1024);
            current_1_adc_offset = current_1_adc_offset
                + ((current_1_sample_measurement as i128 - current_1_adc_offset) / 1024);
            current_2_adc_offset = current_2_adc_offset
                + ((current_2_sample_measurement as i128 - current_2_adc_offset) / 1024);

            // Remove DC offset from voltage and current values
            voltage_sample_offset_removed = voltage_sample_measurement as i128 - voltage_adc_offset;
            current_1_sample_offset_removed =
                current_1_sample_measurement as i128 - current_1_adc_offset;
            current_2_sample_offset_removed =
                current_2_sample_measurement as i128 - current_2_adc_offset;

            // Square values for voltage and current
            voltage_sample_squared = voltage_sample_offset_removed * voltage_sample_offset_removed;
            current_1_sample_squared =
                current_1_sample_offset_removed * current_1_sample_offset_removed;
            current_2_sample_squared =
                current_2_sample_offset_removed * current_2_sample_offset_removed;

            // Sum values for voltage and current
            voltage_sample_sum = voltage_sample_sum + voltage_sample_squared;
            current_1_sample_sum = current_1_sample_sum + current_1_sample_squared;
            current_2_sample_sum = current_2_sample_sum + current_2_sample_squared;

            // Apply phase calibration to voltage (to allow for phase delay in voltage transformer)
            //voltage_phase_corrected = voltage_prior_sample_offset_removed as f64
            //    + voltage_phase_calibration_value
            //        * (voltage_sample_offset_removed - voltage_prior_sample_offset_removed) as f64;
            // See earlier comments about how I have removed the phase calibration because I don't seem to need it.
            // If it goes back in, you will need to change some variables from i128 to f64

            // Calculate instantaneous power
            //power_sample_instantaneous = voltage_phase_corrected * current_sample_offset_removed as f64;
            power_1_sample_instantaneous =
                voltage_sample_offset_removed * current_1_sample_offset_removed;
            power_2_sample_instantaneous =
                voltage_sample_offset_removed * current_2_sample_offset_removed;
            //The toroid transformer has negligible phase delay.....

            // Sum of instantaneous power
            power_1_sample_sum = power_1_sample_sum + power_1_sample_instantaneous;
            power_2_sample_sum = power_2_sample_sum + power_2_sample_instantaneous;

            // Count the number of samples we have taken
            samples_count = samples_count + 1;

            // Store previous offset-corrected voltage for use in the next iteration of the loop
            //voltage_prior_sample_offset_removed = voltage_sample_offset_removed;

            // update a counter if the voltage sine wave has crossed zero
            // TO DO

            //writeln!(tx, "{}, {}, {}, {} \r", counter, current_sample_offset_removed, voltage_sample_offset_removed, voltage_phase_corrected).unwrap();
            //writeln!(tx, "{}, {}, {} \r", counter, current_sample_offset_removed, voltage_sample_offset_removed).unwrap();
            //writeln!(tx, "{}, {}, {}, {} \r", counter, voltage_sample_offset_removed, current_1_sample_offset_removed, current_2_sample_offset_removed).unwrap();

            //delay.delay_ms(1_u32); // I needed to add thsi delay in order for the phase compensation calculations to be correct
                                   // Too many samples meant that the phase compensation was incorrect, giving me low real power values
                                   // Alternatively, I may need a 'better' voltage transformer...
        }
        //defmt::println!("voltage_sample_sum {} \r", voltage_sample_sum);
        defmt::println!("current_1_sample_sum {} \r", current_1_sample_sum);
        defmt::println!("current_2_sample_sum {} \r", current_2_sample_sum);
        defmt::println!("power_1_sample_sum {} \r", power_1_sample_sum);
        defmt::println!("power_2_sample_sum {} \r", power_2_sample_sum);
        defmt::println!("samples_count {} \r", samples_count);

        //-----------------------
        // 3. Post loop calculations
        //-----------------------

        // 3a. Calculate voltage and current ratio values from calibration constants
        // Assumes that the device is operated from 3.3v
        voltage_correction_ratio = voltage_calibration_value * ((3300.0 / 1000.0) / 4096.0);
        current_1_correction_ratio = current_1_calibration_value * ((3300.0 / 1000.0) / 4096.0);
        current_2_correction_ratio = current_2_calibration_value * ((3300.0 / 1000.0) / 4096.0);
        //defmt::println!("voltage_correction_ratio {} \r", voltage_correction_ratio);
        //defmt::println!("current_correction_ratio {} \r", current_correction_ratio);

        // 3b. Calculate RMS values for voltage and current
        //voltage_rms = sqrt(voltage_sample_sum as f64 / samples_count as f64);
        voltage_rms =
            voltage_correction_ratio * sqrt(voltage_sample_sum as f64 / samples_count as f64);
        //current_rms = sqrt(current_sample_sum as f64 / samples_count as f64);
        current_1_rms =
            current_1_correction_ratio * sqrt(current_1_sample_sum as f64 / samples_count as f64);
        current_2_rms =
            current_2_correction_ratio * sqrt(current_2_sample_sum as f64 / samples_count as f64);
        defmt::println!("RMS Voltage {} \r", voltage_rms);
        defmt::println!("RMS Current 1 {} \r", current_1_rms);
        defmt::println!("RMS Current 2 {} \r", current_1_rms);

        // 3c. Calculate real power, apparent power, power factor
        //power_real = power_sample_sum as f64 / samples_count as f64;
        power_1_real = voltage_correction_ratio
            * current_1_correction_ratio
            * (power_1_sample_sum / samples_count) as f64;
        power_1_apparent = voltage_rms * current_1_rms;
        power_1_factor = power_1_real / power_1_apparent;
        
        power_2_real = voltage_correction_ratio
            * current_2_correction_ratio
            * (power_2_sample_sum / samples_count) as f64;
        power_2_apparent = voltage_rms * current_2_rms;
        power_2_factor = power_2_real / power_2_apparent;
        
        defmt::println!("real power 1 {} \r", power_1_real);
        defmt::println!("real power 2 {} \r", power_2_real);
        defmt::println!("apparent power 1 {} \r", power_1_apparent);
        defmt::println!("apparent power 2 {} \r", power_2_apparent);
        defmt::println!("power factor 1 {} \r", power_1_factor);
        defmt::println!("power factor 2 {} \r", power_2_factor);
    }
}
