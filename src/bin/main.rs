//#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

use rtic::app;
use stm32_rust_energy_monitor as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI0])]
mod app {

    use core::fmt::Write;
    use heapless::Vec;
    use libm::sqrt;
    use stm32f4xx_hal::{
        adc::{
            config::{AdcConfig, Dma, SampleTime, Scan, Sequence},
            Adc,
        },
        dma::{config::DmaConfig, PeripheralToMemory, Stream0, StreamsTuple, Transfer},
        gpio::gpioa,
        pac::*,
        prelude::*,
        serial::{Config, Serial, Tx},
        timer::{CounterHz, CounterUs, Event},
    };

    type DMATransfer =
        Transfer<Stream0<DMA2>, 0, Adc<ADC1>, PeripheralToMemory, &'static mut [u16; 8]>;

    #[derive(Debug, Default, PartialEq)]
    enum MeasurementTypes {
        Voltage,
        #[default]
        Current,
    }

    #[derive(Debug, Default)]
    pub struct MeasurementParameters {
        name: &'static str,
        pin: u16,
        measurement_type: MeasurementTypes,
        calibration_value: f64,
        buffer: u16,
        sum_raw_adc_input: i128,
        adc_offset: i128,
        corrected_value: i128,
        sum_squared_corrected_values: i128,
        sum_power: i128,
        rms_value: f64,
        correction_ratio: f64,
    }

    #[shared]
    struct Shared {
        transfer: DMATransfer,
        adc_timer: CounterHz<TIM3>,
        counter: i32,
        measurements: Vec<MeasurementParameters, 8>,
        serial_tx: Tx<USART1>,
    }

    #[local]
    struct Local {
        dma_buffer: Option<&'static mut [u16; 8]>,
        timer_step_1_start_measurement_loop: CounterUs<TIM2>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let device = cx.device;

        let rcc = device.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(100.MHz()).freeze();

        let gpioa = device.GPIOA.split();
        let pin_7 = gpioa.pa7.into_analog();
        let pin_6 = gpioa.pa6.into_analog();
        let pin_5 = gpioa.pa5.into_analog();
        let pin_4 = gpioa.pa4.into_analog();
        let pin_3 = gpioa.pa3.into_analog();
        let pin_2 = gpioa.pa2.into_analog();
        let pin_1 = gpioa.pa1.into_analog();
        let pin_0 = gpioa.pa0.into_analog();
        //let pin_current_1 = gpioa.pa0.into_analog();
        //let pin_current_2 = gpioa.pa3.into_analog();

        let dma = StreamsTuple::new(device.DMA2);

        let config_dma = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true)
            .double_buffer(false);

        let config_adc = AdcConfig::default()
            .dma(Dma::Continuous)
            .scan(Scan::Enabled);

        let mut adc = Adc::adc1(device.ADC1, true, config_adc);

        adc.configure_channel(&pin_0, Sequence::One, SampleTime::Cycles_480);
        adc.configure_channel(&pin_1, Sequence::Two, SampleTime::Cycles_480);
        adc.configure_channel(&pin_2, Sequence::Three, SampleTime::Cycles_480);
        adc.configure_channel(&pin_3, Sequence::Four, SampleTime::Cycles_480);
        adc.configure_channel(&pin_4, Sequence::Five, SampleTime::Cycles_480);
        adc.configure_channel(&pin_5, Sequence::Six, SampleTime::Cycles_480);
        adc.configure_channel(&pin_6, Sequence::Seven, SampleTime::Cycles_480);
        adc.configure_channel(&pin_7, Sequence::Eight, SampleTime::Cycles_480);

        let first_buffer = cortex_m::singleton!(: [u16; 8] = [0; 8]).unwrap();
        let second_buffer = Some(cortex_m::singleton!(: [u16; 8] = [0; 8]).unwrap());
        let transfer = Transfer::init_peripheral_to_memory(dma.0, adc, first_buffer, None, config_dma);

        // Set up Serial
        let tx_pin = gpioa.pa9.into_alternate();
        // Create a tx abstraction for serial transmission
        let serial_tx = Serial::tx(
            device.USART1,
            tx_pin,
            Config::default()
                .baudrate(1152000.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();

        let adc_timer = device.TIM3.counter_hz(&clocks);

        let counter: i32 = 0;

        let mut measurements: Vec<MeasurementParameters, 8> = Vec::new();

        let current_7 = MeasurementParameters {
            name: "Current_7",
            pin: 7,
            measurement_type: MeasurementTypes::Current,
            calibration_value: 1.0,
            ..Default::default()
        };

        let current_6 = MeasurementParameters {
            name: "Current_6",
            pin: 6,
            measurement_type: MeasurementTypes::Current,
            calibration_value: 57.26,
            ..Default::default()
        };

        let current_5 = MeasurementParameters {
            name: "Current_5",
            pin: 5,
            measurement_type: MeasurementTypes::Current,
            calibration_value: 1.0,
            ..Default::default()
        };

        let current_4 = MeasurementParameters {
            name: "Current_4",
            pin: 4,
            measurement_type: MeasurementTypes::Current,
            calibration_value: 1.0,
            ..Default::default()
        };

        let current_3 = MeasurementParameters {
            name: "Current_3",
            pin: 3,
            measurement_type: MeasurementTypes::Current,
            calibration_value: 1.0,
            ..Default::default()
        };

        let current_2 = MeasurementParameters {
            name: "Current_2",
            pin: 2,
            measurement_type: MeasurementTypes::Current,
            calibration_value: 19.00,
            ..Default::default()
        };

        let current_1 = MeasurementParameters {
            name: "Current_1",
            pin: 1,
            measurement_type: MeasurementTypes::Current,
            calibration_value: 1.0,
            ..Default::default()
        };

        let voltage = MeasurementParameters {
            name: "Voltage",
            pin: 0,
            measurement_type: MeasurementTypes::Voltage,
            calibration_value: 266.67,
            ..Default::default()
        };

        measurements.push(voltage).unwrap();
        measurements.push(current_1).unwrap();
        measurements.push(current_2).unwrap();
        measurements.push(current_3).unwrap();
        measurements.push(current_4).unwrap();
        measurements.push(current_5).unwrap();
        measurements.push(current_6).unwrap();
        measurements.push(current_7).unwrap();

        let mut timer_step_1_start_measurement_loop = device.TIM2.counter_us(&clocks);

        timer_step_1_start_measurement_loop
            .start(10000.millis())
            .unwrap();
        timer_step_1_start_measurement_loop.listen(Event::Update);

        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM2);
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM3);
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM4);
        }

        (
            Shared {
                transfer,
                adc_timer,
                counter,
                measurements,
                serial_tx,
            },
            Local {
                dma_buffer: second_buffer,
                timer_step_1_start_measurement_loop,
            },
            init::Monotonics(),
        )
    }

    //---------
    // Task 1: Start the ADC timer
    //---------

    #[task(binds=TIM2, shared = [adc_timer, measurements, counter], local = [timer_step_1_start_measurement_loop])]
    fn task_1_start_adc(mut cx: task_1_start_adc::Context) {
        // Clear the interrupt so that the main measurement loop runs again
        cx.local
            .timer_step_1_start_measurement_loop
            .clear_interrupt(Event::Update);

        cx.shared.counter.lock(|counter| {
            *counter = 0;
        });

        cx.shared.measurements.lock(|m| {
            for i in 0..m.len() {
                m[i].buffer = 0;
                m[i].sum_raw_adc_input = 0;
                m[i].adc_offset = 0;
                m[i].corrected_value = 0;
                m[i].sum_squared_corrected_values = 0;
                m[i].sum_power = 0;
                m[i].rms_value = 0.0;
                m[i].correction_ratio = 0.0;
            }
        });

        cx.shared.adc_timer.lock(|adc_timer| {
            adc_timer.listen(Event::Update);
            adc_timer.start(1000.Hz()).unwrap();
        });
    }

    //-----------
    // Task 2: Start conversion with ADC timer interrupt is triggered
    //-----------

    #[task(binds = TIM3, shared = [transfer, adc_timer, counter, measurements])]
    fn task_2_start_adc_conversion(mut cx: task_2_start_adc_conversion::Context) {
        cx.shared.transfer.lock(|transfer| {
            transfer.start(|adc| {
                adc.start_conversion();
            });
        });

        cx.shared.adc_timer.lock(|adc_timer| {
            adc_timer.clear_interrupt(Event::Update);
        });
    }

    //---------
    // Task 3: Retrieve ADC data from DMA buffer once it is available
    //---------

    #[task(binds = DMA2_STREAM0, shared = [transfer, adc_timer, counter, measurements, serial_tx], local = [dma_buffer])]
    fn task_3_retrieve_adc_data(mut cx: task_3_retrieve_adc_data::Context) {
        //let dma::Context { mut shared, local } = cx;
        let dma_buffer = cx.shared.transfer.lock(|transfer| {
            let (dma_buffer, _) = transfer
                .next_transfer(cx.local.dma_buffer.take().unwrap())
                .unwrap();
            dma_buffer
        });

        cx.shared.measurements.lock(|m| {
            for i in 0..m.len() {
                let pin = m[i].pin as usize;
                m[i].buffer = dma_buffer[pin];
            }
        });

        cx.shared.counter.lock(|counter| {
            *counter = *counter + 1;

            if counter <= &mut 2000 {
                //defmt::println!("{}", counter);
                step_4_calculate_adc_offset::spawn().unwrap();
            } else {
                step_5_obtain_energy_flow::spawn().unwrap();
            }
        });

        *cx.local.dma_buffer = Some(dma_buffer);
    }

    #[task(shared = [measurements, counter, adc_timer])]
    fn step_4_calculate_adc_offset(mut cx: step_4_calculate_adc_offset::Context) {
        cx.shared.measurements.lock(|m| {
            for i in 0..m.len() {
                m[i].sum_raw_adc_input = m[i].sum_raw_adc_input + m[i].buffer as i128;
            }
        });

        cx.shared.counter.lock(|counter| {
            if counter == &mut 2000 {
                cx.shared.measurements.lock(|m| {
                    for i in 0..m.len() {
                        m[i].adc_offset = m[i].sum_raw_adc_input / *counter as i128;
                        defmt::println!("ADC Offset for {}, {}", m[i].name, m[i].adc_offset)
                    }
                });
                step_5_obtain_energy_flow::spawn().unwrap();
            }
        });
    }

    #[task(shared = [measurements, counter, adc_timer, serial_tx])]
    fn step_5_obtain_energy_flow(mut cx: step_5_obtain_energy_flow::Context) {
        cx.shared.measurements.lock(|m| {
            for i in 0..m.len() {
                m[i].corrected_value = m[i].buffer as i128 - m[i].adc_offset;

                let squared_corrected_value = m[i].corrected_value * m[i].corrected_value;
                m[i].sum_squared_corrected_values =
                    m[i].sum_squared_corrected_values + squared_corrected_value;
            }

            for i in 1..m.len() {
                let instantaneous_power = m[i].corrected_value * m[0].corrected_value;
                m[i].sum_power = m[i].sum_power + instantaneous_power;
            }
        });

        cx.shared.counter.lock(|counter| {
            if counter >= &mut 4000 {
                cx.shared.adc_timer.lock(|adc_timer| {
                    adc_timer.cancel().unwrap();
                });
                defmt::println!("Finished");

                cx.shared.measurements.lock(|m| {
                    for i in 0..m.len() {
                        if m[i].adc_offset > 1000 {
                            m[i].correction_ratio =
                                m[i].calibration_value * ((3300.0 / 1000.0) / 4096.0);

                            m[i].rms_value = m[i].correction_ratio
                                * sqrt(m[i].sum_squared_corrected_values as f64 / 2000.0);

                            defmt::println!("RMS {}: {}", m[i].name, m[i].rms_value);

                            if m[i].measurement_type == MeasurementTypes::Current {
                                let power_real = m[0].correction_ratio
                                    * m[i].correction_ratio
                                    * (m[i].sum_power as f64 / 2000.0);

                                defmt::println!("Real power for {}, {}", m[i].name, power_real);

                                let power_apparent = m[0].rms_value * m[i].rms_value;

                                defmt::println!(
                                    "Apparent power for {}, {}",
                                    m[i].name,
                                    power_apparent
                                );

                                let power_factor = power_real / power_apparent;

                                defmt::println!(
                                    "Apparent power for {}, {}",
                                    m[i].name,
                                    power_factor
                                );
                            };
                        };
                    }
                })
            }
        })

        //defmt::println!("Ready to gather information");
    }
}
