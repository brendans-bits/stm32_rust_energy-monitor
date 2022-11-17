//#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

use rtic::app;
use stm32_rust_energy_monitor as _; // global logger + panicking-behavior + memory layout

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {

    use core::fmt::Write;
    use core::sync::atomic::{AtomicUsize, Ordering};
    use rtt_target::{rprintln, rtt_init_print};
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
    use heapless::Vec;

    pub struct ADCResources {
        timer_sampling_loop: CounterUs<TIM3>,
        counter_count: CounterUs<TIM5>,
        timer_measurement_loop: CounterUs<TIM4>,
        count_sample: u16,
        sum_voltage: u128,
        adc_average_voltage: u128,
        sum_current_1: u128,
        sum_current_2: u128,
        adc_average_current_1: u128,
        adc_average_current_2: u128,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {

        let device_peripherals = ctx.device;

        let rcc = device_peripherals.RCC.constrain();

        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(100.MHz()).freeze();

        let mut timer_step_1_start_measurement_loop = device_peripherals.TIM2.counter_us(&clocks);

        timer_step_1_start_measurement_loop.start(10000.millis()).unwrap();
        timer_step_1_start_measurement_loop.listen(Event::Update);

        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM2);
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM3);
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM4);
        }

        let gpioa = device_peripherals.GPIOA.split();
        let pin_voltage = gpioa.pa7.into_analog();
        let pin_current_1 = gpioa.pa0.into_analog();
        let pin_current_2 = gpioa.pa3.into_analog();

        // Set up ADC
        let adc = Adc::adc1(device_peripherals.ADC1, true, AdcConfig::default());

        //let count_measurement: u16 = 0;
        //let count_sample: u16 = 0;

        let simple_moving_average_voltage: u16 = 0;
        //let sum_voltage: u128 = 0;
        //let adc_average_voltage: u128 = 0;
        let buffer_voltage: Vec<u16, 10> = Vec::new();

        let simple_moving_average_current_1: u16 = 0;
        //let sum_current_1: u128 = 0;
        //let adc_average_current_1: u128 = 0;
        let buffer_current_1: Vec<u16, 10> = Vec::new();

        let simple_moving_average_current_2: u16 = 0;
        //let sum_current_2: u128 = 0;
        //let adc_average_current_2: u128 = 0;
        let buffer_current_2: Vec<u16, 10> = Vec::new();

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

        let adc_resources = ADCResources {
            timer_sampling_loop: device_peripherals.TIM3.counter_us(&clocks),
            counter_count: device_peripherals.TIM5.counter_us(&clocks),
            timer_measurement_loop: device_peripherals.TIM4.counter_us(&clocks),
            count_sample: 0,
            sum_voltage: 0,
            adc_average_voltage: 0,
            sum_current_1: 0,
            sum_current_2: 0,
            adc_average_current_1: 0,
            adc_average_current_2: 0,
        };

        (
            Shared { adc_resources },
            Local {
                //led,
                pin_voltage,
                pin_current_1,
                pin_current_2,
                adc,
                timer_step_1_start_measurement_loop,
                //count_measurement,
                buffer_voltage,
                simple_moving_average_voltage,
                serial_tx,
                buffer_current_1,
                buffer_current_2,
                simple_moving_average_current_1,
                simple_moving_average_current_2,
            },
            init::Monotonics(),
        )
    }

    #[shared]
    struct Shared {
        adc_resources: ADCResources,
    }

    #[local]
    struct Local {
        pin_voltage: Pin<'A', 7, Analog>,
        pin_current_1: Pin<'A', 0, Analog>,
        pin_current_2: Pin<'A', 3, Analog>,
        adc: Adc<ADC1>,
        timer_step_1_start_measurement_loop: CounterUs<TIM2>,
        //count_measurement: u16,
        buffer_voltage: Vec<u16, 10>,
        simple_moving_average_voltage: u16,
        serial_tx: Tx<USART1>,
        buffer_current_1: Vec<u16, 10>,
        buffer_current_2: Vec<u16, 10>,
        simple_moving_average_current_1: u16,
        simple_moving_average_current_2: u16,
    }

    // Task TIM2 is purely to deal with the TIM2 interrupt and call the main measurement loop.
    #[task(
        binds = TIM2,
        local = [
            timer_step_1_start_measurement_loop,
            //count_measurement,
            //heap_usage
        ],
        shared = [
            adc_resources
        ]
    )]
    fn step_1_start_measurement_loop(mut ctx: step_1_start_measurement_loop::Context) {
        let step_1_start_measurement_loop::LocalResources {
            //count_measurement,
            timer_step_1_start_measurement_loop,
        } = ctx.local;

        // Clear the interrupt so that the main measurement loop runs again
        timer_step_1_start_measurement_loop.clear_interrupt(Event::Update);

        defmt::println!("Start measurement loop");

        ctx.shared.adc_resources.lock(|adc_resources| {
            adc_resources.count_sample = 0;

            adc_resources.sum_voltage = 0;

            adc_resources.sum_current_1 = 0;

            adc_resources.sum_current_2 = 0;

            adc_resources.counter_count.start(3000.millis()).unwrap();

            adc_resources
                .timer_sampling_loop
                .start(1000.micros())
                .unwrap();
            adc_resources.timer_sampling_loop.listen(Event::Update);
        });
    }

    #[task(
        binds = TIM3,
        local = [
             adc,
             pin_voltage,
             pin_current_1,
             pin_current_2,
             buffer_voltage,
             buffer_current_1,
             buffer_current_2,
             simple_moving_average_voltage,
             simple_moving_average_current_1,
             simple_moving_average_current_2,
             serial_tx
         ],
         shared = [
             adc_resources
         ]
     )]
    fn step_2_get_adc_offset(ctx: step_2_get_adc_offset::Context) {
        let step_2_get_adc_offset::LocalResources {
            adc,
            pin_voltage,
            pin_current_1,
            pin_current_2,
            buffer_voltage,
            buffer_current_1,
            buffer_current_2,
            simple_moving_average_voltage,
            simple_moving_average_current_1,
            simple_moving_average_current_2,
            serial_tx,
        } = ctx.local;

        let step_2_get_adc_offset::SharedResources { mut adc_resources } = ctx.shared;

        adc_resources.lock(|adc_resources| {
            adc_resources
                .timer_sampling_loop
                .clear_interrupt(Event::Update);

            buffer_voltage.push(adc.convert(pin_voltage, SampleTime::Cycles_3)).unwrap();

            buffer_current_1.push(adc.convert(pin_current_1, SampleTime::Cycles_3)).unwrap();

            buffer_current_2.push(adc.convert(pin_current_2, SampleTime::Cycles_3)).unwrap();

            if buffer_voltage.len() > 9 {
                *simple_moving_average_voltage =
                    buffer_voltage.iter().sum::<u16>() / buffer_voltage.len() as u16;

                *simple_moving_average_current_1 =
                    buffer_current_1.iter().sum::<u16>() / buffer_current_1.len() as u16;

                *simple_moving_average_current_2 =
                    buffer_current_2.iter().sum::<u16>() / buffer_current_2.len() as u16;

                adc_resources.sum_voltage =
                    adc_resources.sum_voltage + *simple_moving_average_voltage as u128;

                adc_resources.sum_current_1 =
                    adc_resources.sum_current_1 + *simple_moving_average_current_1 as u128;

                adc_resources.sum_current_2 =
                    adc_resources.sum_current_2 + *simple_moving_average_current_2 as u128;

                buffer_voltage.remove(0);
                buffer_current_1.remove(0);
                buffer_current_2.remove(0);

                adc_resources.count_sample = adc_resources.count_sample + 1;

                writeln!(
                    serial_tx,
                    "{}, {}, {}, {} \r",
                    adc_resources
                        .counter_count
                        .now()
                        .duration_since_epoch()
                        .ticks(),
                    simple_moving_average_voltage,
                    simple_moving_average_current_1,
                    simple_moving_average_current_2
                )
                .unwrap();
            }

            if adc_resources.count_sample > 2000 {
                adc_resources.timer_sampling_loop.cancel().unwrap();

                adc_resources.adc_average_voltage =
                    adc_resources.sum_voltage / adc_resources.count_sample as u128;
                defmt::println!("ADC Voltage average {}", adc_resources.adc_average_voltage);

                adc_resources.adc_average_current_1 =
                    adc_resources.sum_current_1 / adc_resources.count_sample as u128;
                defmt::println!(
                    "ADC Current 1 average {}",
                    adc_resources.adc_average_current_1
                );

                adc_resources.adc_average_current_2 =
                    adc_resources.sum_current_2 / adc_resources.count_sample as u128;
                defmt::println!(
                    "ADC Current 2 average {}",
                    adc_resources.adc_average_current_2
                );

                defmt::println!(
                    "Time taken {}",
                    adc_resources
                        .counter_count
                        .now()
                        .duration_since_epoch()
                        .ticks()
                );

                adc_resources
                .timer_measurement_loop
                .start(1000.micros())
                .unwrap();
            
                adc_resources.timer_measurement_loop.listen(Event::Update);

                adc_resources.count_sample = 0;
                
            };
        })
    }

    #[task(binds = TIM4, shared = [adc_resources])]
    fn step_3_measure_energy_usage(ctx: step_3_measure_energy_usage::Context) {

        let step_3_measure_energy_usage::SharedResources { mut adc_resources } = ctx.shared;

        adc_resources.lock(|adc_resources| {

            adc_resources.timer_measurement_loop.clear_interrupt(Event::Update);

            adc_resources.count_sample = adc_resources.count_sample + 1;

            if adc_resources.count_sample > 2000 {
                
                adc_resources.timer_measurement_loop.cancel().unwrap();

                defmt::println!("Finished");

                step_4_post_measurement_calculations::spawn().unwrap();
            
            }
        
        });

    }

    #[task]
    fn step_4_post_measurement_calculations(ctx: step_4_post_measurement_calculations::Context) {
        defmt::println!("Energy usage is....");
    }
}
