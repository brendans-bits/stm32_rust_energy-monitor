//#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

use rtic::app;
use stm32_rust_energy_monitor as _; // global logger + panicking-behavior + memory layout

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {

    use core::fmt::Write;
    use heapless::Vec;
    use libm::sqrt;
    //use rtt_target::{rprintln, rtt_init_print};
    use stm32f4xx_hal::{
        adc::{
            config::{AdcConfig, SampleTime},
            Adc,
        },
        gpio::{self, Analog, Input, Pin},
        pac::*,
        prelude::*,
        serial::{Config, Serial, Tx},
        timer::{CounterUs, Event},
    };

    pub struct SharedADCResources {
        //ADC
        adc: Adc<ADC1>,
        //Timers and counters
        step_2_timer_adc_sampling_loop: CounterUs<TIM3>,
        step_3_timer_energy_sampling_loop: CounterUs<TIM4>,
        count_time: CounterUs<TIM5>,
        count_adc_offset_measurements: u16,
        count_energy_usage_measurements: u16,
        //Pins
        pin_voltage: Pin<'A', 7, Analog>,
        pin_current_1: Pin<'A', 0, Analog>,
        pin_current_2: Pin<'A', 3, Analog>,
        //Buffers
        buffer_voltage: Vec<u16, 10>,
        buffer_current_1: Vec<u16, 10>,
        buffer_current_2: Vec<u16, 10>,
        //Simple moving averages
        simple_moving_average_voltage: u128,
        simple_moving_average_current_1: u128,
        simple_moving_average_current_2: u128,
        // Sum
        sum_voltage: u128,
        sum_current_1: u128,
        sum_current_2: u128,
        // ADC averages
        adc_offset_voltage: u128,
        adc_offset_current_1: u128,
        adc_offset_current_2: u128,
        // ADC offset removed
        offset_removed_voltage: i128,
        offset_removed_current_1: i128,
        offset_removed_current_2: i128,
        // Squared offset removed
        squared_offset_removed_voltage: i128,
        squared_offset_removed_current_1: i128,
        squared_offset_removed_current_2: i128,
        // Sum offset removed
        sum_squared_voltage: i128,
        sum_squared_current_1: i128,
        sum_squared_current_2: i128,
        // RMS values
        rms_voltage: f64,
        rms_current_1: f64,
        rms_current_2: f64,
        // Power - Instantaneous
        instantaneous_power_1: i128,
        instantaneous_power_2: i128,
        // Power - Sum
        sum_power_1: i128,
        sum_power_2: i128,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let device_peripherals = ctx.device;

        let rcc = device_peripherals.RCC.constrain();

        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(100.MHz()).freeze();

        let mut timer_step_1_start_measurement_loop = device_peripherals.TIM2.counter_us(&clocks);

        timer_step_1_start_measurement_loop
            .start(10000.millis())
            .unwrap();
        timer_step_1_start_measurement_loop.listen(Event::Update);

        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM2);
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM3);
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM4);
        }

        let gpioa = device_peripherals.GPIOA.split();

        // Set up Serial
        let tx_pin = gpioa.pa9.into_alternate();
        // Create a tx abstraction for serial transmission
        let serial_tx = Serial::tx(
            device_peripherals.USART1,
            tx_pin,
            Config::default()
                .baudrate(1152000.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();

        let voltage_calibration_value: f64 = 394.9; // rough value only!
        let current_1_calibration_value: f64 = 27.87; // rough value only!
        let current_2_calibration_value: f64 = 85.04; // rough value only!

        let shared_adc_resources = SharedADCResources {
            //ADC
            adc: Adc::adc1(device_peripherals.ADC1, true, AdcConfig::default()),
            //Timers
            step_2_timer_adc_sampling_loop: device_peripherals.TIM3.counter_us(&clocks),
            step_3_timer_energy_sampling_loop: device_peripherals.TIM4.counter_us(&clocks),
            count_time: device_peripherals.TIM5.counter_us(&clocks),
            count_adc_offset_measurements: 0,
            count_energy_usage_measurements: 0,
            //Pins
            pin_voltage: gpioa.pa7.into_analog(),
            pin_current_1: gpioa.pa0.into_analog(),
            pin_current_2: gpioa.pa3.into_analog(),
            //Buffers
            buffer_voltage: Vec::new(),
            buffer_current_1: Vec::new(),
            buffer_current_2: Vec::new(),
            //SMA
            simple_moving_average_voltage: 0,
            simple_moving_average_current_1: 0,
            simple_moving_average_current_2: 0,

            offset_removed_voltage: 0,
            offset_removed_current_1: 0,
            offset_removed_current_2: 0,

            squared_offset_removed_voltage: 0,
            squared_offset_removed_current_1: 0,
            squared_offset_removed_current_2: 0,

            sum_squared_voltage: 0,
            sum_squared_current_1: 0,
            sum_squared_current_2: 0,

            adc_offset_voltage: 0,
            adc_offset_current_1: 0,
            adc_offset_current_2: 0,

            sum_voltage: 0,
            sum_current_1: 0,
            sum_current_2: 0,

            rms_voltage: 0.0,
            rms_current_1: 0.0,
            rms_current_2: 0.0,
            // Power - Instantaneous
            instantaneous_power_1: 0,
            instantaneous_power_2: 0,
            // Power - Sum
            sum_power_1: 0,
            sum_power_2: 0,
        };

        (
            Shared {
                shared_adc_resources,
            },
            Local {
                timer_step_1_start_measurement_loop,
                serial_tx,
                voltage_calibration_value,
                current_1_calibration_value,
                current_2_calibration_value,
            },
            init::Monotonics(),
        )
    }

    #[shared]
    struct Shared {
        shared_adc_resources: SharedADCResources,
    }

    #[local]
    struct Local {
        timer_step_1_start_measurement_loop: CounterUs<TIM2>,
        serial_tx: Tx<USART1>,
        voltage_calibration_value: f64,
        current_1_calibration_value: f64,
        current_2_calibration_value: f64,
    }

    // Task TIM2 is purely to deal with the TIM2 interrupt and call the main measurement loop.
    #[task(binds = TIM2, local = [timer_step_1_start_measurement_loop], shared = [shared_adc_resources])]
    fn step_1_start_measurement_loop(mut ctx: step_1_start_measurement_loop::Context) {
        let step_1_start_measurement_loop::LocalResources {
            //count_measurement,
            timer_step_1_start_measurement_loop,
        } = ctx.local;

        // Clear the interrupt so that the main measurement loop runs again
        timer_step_1_start_measurement_loop.clear_interrupt(Event::Update);

        defmt::println!("Start measurement loop");

        ctx.shared
            .shared_adc_resources
            .lock(|shared_adc_resources| {
                shared_adc_resources.count_adc_offset_measurements = 0;

                shared_adc_resources.sum_voltage = 0;

                shared_adc_resources.sum_current_1 = 0;

                shared_adc_resources.sum_current_2 = 0;

                shared_adc_resources
                    .count_time
                    .start(3000.millis())
                    .unwrap();

                shared_adc_resources
                    .step_2_timer_adc_sampling_loop
                    .start(1000.micros())
                    .unwrap();
                shared_adc_resources
                    .step_2_timer_adc_sampling_loop
                    .listen(Event::Update);
            });
    }

    #[task(binds = TIM3, local = [serial_tx], shared = [shared_adc_resources])]
    fn step_2_get_adc_offset(ctx: step_2_get_adc_offset::Context) {
        let step_2_get_adc_offset::LocalResources { serial_tx } = ctx.local;

        let step_2_get_adc_offset::SharedResources {
            mut shared_adc_resources,
        } = ctx.shared;

        shared_adc_resources.lock(|shared_adc_resources| {
            shared_adc_resources
                .step_2_timer_adc_sampling_loop
                .clear_interrupt(Event::Update);

            shared_adc_resources
                .buffer_voltage
                .push(
                    shared_adc_resources
                        .adc
                        .convert(&shared_adc_resources.pin_voltage, SampleTime::Cycles_3),
                )
                .unwrap();

            shared_adc_resources
                .buffer_current_1
                .push(
                    shared_adc_resources
                        .adc
                        .convert(&shared_adc_resources.pin_current_1, SampleTime::Cycles_3),
                )
                .unwrap();

            shared_adc_resources
                .buffer_current_2
                .push(
                    shared_adc_resources
                        .adc
                        .convert(&shared_adc_resources.pin_current_2, SampleTime::Cycles_3),
                )
                .unwrap();

            if shared_adc_resources.buffer_voltage.len() > 9 {
                shared_adc_resources.simple_moving_average_voltage =
                    shared_adc_resources.buffer_voltage.iter().sum::<u16>() as u128
                        / shared_adc_resources.buffer_voltage.len() as u128;

                shared_adc_resources.simple_moving_average_current_1 =
                    shared_adc_resources.buffer_current_1.iter().sum::<u16>() as u128
                        / shared_adc_resources.buffer_current_1.len() as u128;

                shared_adc_resources.simple_moving_average_current_2 =
                    shared_adc_resources.buffer_current_2.iter().sum::<u16>() as u128
                        / shared_adc_resources.buffer_current_2.len() as u128;

                shared_adc_resources.sum_voltage = shared_adc_resources.sum_voltage
                    + shared_adc_resources.simple_moving_average_voltage;

                shared_adc_resources.sum_current_1 = shared_adc_resources.sum_current_1
                    + shared_adc_resources.simple_moving_average_current_1;

                shared_adc_resources.sum_current_2 = shared_adc_resources.sum_current_2
                    + shared_adc_resources.simple_moving_average_current_2 as u128;

                shared_adc_resources.buffer_voltage.remove(0);
                shared_adc_resources.buffer_current_1.remove(0);
                shared_adc_resources.buffer_current_2.remove(0);

                shared_adc_resources.count_adc_offset_measurements =
                    shared_adc_resources.count_adc_offset_measurements + 1;

                writeln!(
                    serial_tx,
                    "{}, {}, {}, {} \r",
                    shared_adc_resources
                        .count_time
                        .now()
                        .duration_since_epoch()
                        .ticks(),
                    shared_adc_resources.simple_moving_average_voltage,
                    shared_adc_resources.simple_moving_average_current_1,
                    shared_adc_resources.simple_moving_average_current_2
                )
                .unwrap();
            }

            if shared_adc_resources.count_adc_offset_measurements > 2000 {
                shared_adc_resources
                    .step_2_timer_adc_sampling_loop
                    .cancel()
                    .unwrap();

                shared_adc_resources.adc_offset_voltage = shared_adc_resources.sum_voltage
                    / shared_adc_resources.count_adc_offset_measurements as u128;
                //defmt::println!(
                //    "ADC Voltage average {}",
                //    shared_adc_resources.adc_offset_voltage
                //);

                shared_adc_resources.adc_offset_current_1 = shared_adc_resources.sum_current_1
                    / shared_adc_resources.count_adc_offset_measurements as u128;
                //defmt::println!(
                //    "ADC Current 1 average {}",
                //    shared_adc_resources.adc_offset_current_1
                //);

                shared_adc_resources.adc_offset_current_2 = shared_adc_resources.sum_current_2
                    / shared_adc_resources.count_adc_offset_measurements as u128;
                //defmt::println!(
                //    "ADC Current 2 average {}",
                //    shared_adc_resources.adc_offset_current_2
                //);

                defmt::println!(
                    "Time taken for adc offset measurement {}",
                    shared_adc_resources
                        .count_time
                        .now()
                        .duration_since_epoch()
                        .ticks()
                );

                shared_adc_resources.count_time.cancel().unwrap();

                shared_adc_resources.count_energy_usage_measurements = 0;
                shared_adc_resources.buffer_voltage = Vec::new();
                shared_adc_resources.buffer_current_1 = Vec::new();
                shared_adc_resources.buffer_current_2 = Vec::new();
                shared_adc_resources.sum_squared_voltage = 0;
                shared_adc_resources.sum_squared_current_1 = 0;
                shared_adc_resources.sum_squared_current_2 = 0;
                shared_adc_resources.sum_power_1 = 0;
                shared_adc_resources.sum_power_2 = 0;

                shared_adc_resources
                    .count_time
                    .start(3000.millis())
                    .unwrap();

                shared_adc_resources
                    .step_3_timer_energy_sampling_loop
                    .start(1000.micros())
                    .unwrap();

                shared_adc_resources
                    .step_3_timer_energy_sampling_loop
                    .listen(Event::Update);
            };
        })
    }

    #[task(binds = TIM4, shared = [shared_adc_resources])]
    fn step_3_measure_energy_usage(ctx: step_3_measure_energy_usage::Context) {
        let step_3_measure_energy_usage::SharedResources {
            mut shared_adc_resources,
        } = ctx.shared;

        shared_adc_resources.lock(|shared_adc_resources| {
            shared_adc_resources
                .step_3_timer_energy_sampling_loop
                .clear_interrupt(Event::Update);

            // Get ADC values

            shared_adc_resources
                .buffer_voltage
                .push(
                    shared_adc_resources
                        .adc
                        .convert(&shared_adc_resources.pin_voltage, SampleTime::Cycles_3),
                )
                .unwrap();

            shared_adc_resources
                .buffer_current_1
                .push(
                    shared_adc_resources
                        .adc
                        .convert(&shared_adc_resources.pin_current_1, SampleTime::Cycles_3),
                )
                .unwrap();

            shared_adc_resources
                .buffer_current_2
                .push(
                    shared_adc_resources
                        .adc
                        .convert(&shared_adc_resources.pin_current_2, SampleTime::Cycles_3),
                )
                .unwrap();

            if shared_adc_resources.buffer_voltage.len() > 9 {
                
                // Get simple moving averages for ADC values

                shared_adc_resources.simple_moving_average_voltage =
                    shared_adc_resources.buffer_voltage.iter().sum::<u16>() as u128
                        / shared_adc_resources.buffer_voltage.len() as u16 as u128;

                shared_adc_resources.simple_moving_average_current_1 =
                    shared_adc_resources.buffer_current_1.iter().sum::<u16>() as u128
                        / shared_adc_resources.buffer_current_1.len() as u16 as u128;

                shared_adc_resources.simple_moving_average_current_2 =
                    shared_adc_resources.buffer_current_2.iter().sum::<u16>() as u128
                        / shared_adc_resources.buffer_current_2.len() as u16 as u128;

                // Remove the ADC offset from the simple moving average

                shared_adc_resources.offset_removed_voltage =
                    shared_adc_resources.simple_moving_average_voltage as i128
                        - shared_adc_resources.adc_offset_voltage as i128;

                shared_adc_resources.offset_removed_current_1 =
                    shared_adc_resources.simple_moving_average_current_1 as i128
                        - shared_adc_resources.adc_offset_current_1 as i128;

                shared_adc_resources.offset_removed_current_2 =
                    shared_adc_resources.simple_moving_average_current_2 as i128
                        - shared_adc_resources.adc_offset_current_2 as i128;

                // Square the offset removed values

                shared_adc_resources.squared_offset_removed_voltage = shared_adc_resources
                    .offset_removed_voltage
                    * shared_adc_resources.offset_removed_voltage;

                shared_adc_resources.squared_offset_removed_current_1 = shared_adc_resources
                    .offset_removed_current_1
                    * shared_adc_resources.offset_removed_current_1;

                shared_adc_resources.squared_offset_removed_current_2 = shared_adc_resources
                    .offset_removed_current_2
                    * shared_adc_resources.offset_removed_current_2;

                // Sum the squared values

                shared_adc_resources.sum_squared_voltage = shared_adc_resources.sum_squared_voltage
                    + shared_adc_resources.squared_offset_removed_voltage;

                shared_adc_resources.sum_squared_current_1 = shared_adc_resources
                    .sum_squared_current_1
                    + shared_adc_resources.squared_offset_removed_current_1;

                shared_adc_resources.sum_squared_current_2 = shared_adc_resources
                    .sum_squared_current_2
                    + shared_adc_resources.squared_offset_removed_current_2;

                // Calculate instantaneous power

                shared_adc_resources.instantaneous_power_1 = shared_adc_resources
                    .offset_removed_voltage
                    * shared_adc_resources.offset_removed_current_1;

                shared_adc_resources.instantaneous_power_2 = shared_adc_resources
                    .offset_removed_voltage
                    * shared_adc_resources.offset_removed_current_2;

                // Sum instantaneous power

                shared_adc_resources.sum_power_1 =
                    shared_adc_resources.sum_power_1 + shared_adc_resources.instantaneous_power_1;

                shared_adc_resources.sum_power_2 =
                    shared_adc_resources.sum_power_2 + shared_adc_resources.instantaneous_power_2;

                shared_adc_resources.buffer_voltage.remove(0);
                shared_adc_resources.buffer_current_1.remove(0);
                shared_adc_resources.buffer_current_2.remove(0);

                shared_adc_resources.count_energy_usage_measurements =
                    shared_adc_resources.count_energy_usage_measurements + 1;

                if shared_adc_resources.count_energy_usage_measurements > 2000 {
                    shared_adc_resources
                        .step_3_timer_energy_sampling_loop
                        .cancel()
                        .unwrap();

                    //defmt::println!("Finished");

                    defmt::println!(
                        "Time taken for energy usage measurements {}",
                        shared_adc_resources
                            .count_time
                            .now()
                            .duration_since_epoch()
                            .ticks()
                    );

                    shared_adc_resources.count_time.cancel().unwrap();

                    step_4_post_measurement_calculations::spawn().unwrap();
                }
            }
        });
    }

    #[task(
        local = [
        voltage_calibration_value,
        current_1_calibration_value,
        current_2_calibration_value,
    ], 
    shared = [shared_adc_resources])]
    fn step_4_post_measurement_calculations(ctx: step_4_post_measurement_calculations::Context) {
        let step_4_post_measurement_calculations::SharedResources {
            mut shared_adc_resources,
        } = ctx.shared;

        let step_4_post_measurement_calculations::LocalResources {
            voltage_calibration_value,
            current_1_calibration_value,
            current_2_calibration_value,
        } = ctx.local;

        shared_adc_resources.lock(|shared_adc_resources| {
            let voltage_correction_ratio =
                *voltage_calibration_value * ((3300.0 / 1000.0) / 4096.0);
            let current_1_correction_ratio =
                *current_1_calibration_value * ((3300.0 / 1000.0) / 4096.0);
            let current_2_correction_ratio =
                *current_2_calibration_value * ((3300.0 / 1000.0) / 4096.0);

            // Calculate RMS values

            shared_adc_resources.rms_voltage = voltage_correction_ratio
                * sqrt(
                    shared_adc_resources.sum_squared_voltage as f64
                        / shared_adc_resources.count_energy_usage_measurements as f64,
                );

            shared_adc_resources.rms_current_1 = current_1_correction_ratio
                * sqrt(
                    shared_adc_resources.sum_squared_current_1 as f64
                        / shared_adc_resources.count_energy_usage_measurements as f64,
                );

            shared_adc_resources.rms_current_2 = current_2_correction_ratio
                * sqrt(
                    shared_adc_resources.sum_squared_current_2 as f64
                        / shared_adc_resources.count_energy_usage_measurements as f64,
                );

            defmt::println!("RMS Voltage {}", shared_adc_resources.rms_voltage);
            defmt::println!("RMS Current 1 {}", shared_adc_resources.rms_current_1);
            defmt::println!("RMS Current 2 {}", shared_adc_resources.rms_current_2);

            // Calculate real power

            let power_1_real = voltage_correction_ratio
                * current_1_correction_ratio
                * (shared_adc_resources.sum_power_1
                    / shared_adc_resources.count_energy_usage_measurements as i128)
                    as f64;

            let power_2_real = voltage_correction_ratio
                * current_2_correction_ratio
                * (shared_adc_resources.sum_power_2
                    / shared_adc_resources.count_energy_usage_measurements as i128)
                    as f64;

            // Calculate apparent power

            let power_1_apparent =
                shared_adc_resources.rms_voltage * shared_adc_resources.rms_current_1;
            let power_2_apparent =
                shared_adc_resources.rms_voltage * shared_adc_resources.rms_current_2;

            // Calculate power factor

            let power_factor_1 = power_1_real / power_1_apparent;
            let power_factor_2 = power_2_real / power_2_apparent;

            defmt::println!("Real Power 1 {}", power_1_real);
            defmt::println!("Real Power 2 {}", power_2_real);
            defmt::println!("Apparent Power 1 {}", power_1_apparent);
            defmt::println!("Apparent Power 2 {}", power_2_apparent);
            defmt::println!("Power Factor 1 {}", power_factor_1);
            defmt::println!("Power Factor 2 {}", power_factor_2);
            defmt::println!(
                "Energy Usage Measurements Taken {}",
                shared_adc_resources.count_energy_usage_measurements
            );
        })
    }
}
