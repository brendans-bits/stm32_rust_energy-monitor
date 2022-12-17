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
    use stm32f4xx_hal::{
        adc::{
            config::{AdcConfig, Dma, SampleTime, Scan, Sequence},
            Adc, Temperature,
        },
        dma::{config::DmaConfig, PeripheralToMemory, Stream0, StreamsTuple, Transfer},
        gpio::gpioa,
        pac::{self, ADC1, DMA2, TIM3, USART1},
        prelude::*,
        serial::{Config, Serial, Tx},
        timer::{CounterHz, Event},
    };

    type DMATransfer =
        Transfer<Stream0<DMA2>, 0, Adc<ADC1>, PeripheralToMemory, &'static mut [u16; 3]>;

    #[derive(Debug, Default)]
    enum MeasurementTypes {
        Voltage,
        #[default]
        Current,
    }

    #[derive(Debug, Default)]
    pub struct MeasurementParameters {
        pin: u8,
        measurement_type: MeasurementTypes,
        calibration_value: f64,
        buffer: Vec<u16, 10>,
        sum_simple_moving_average: u128,
    }

    //impl MeasurementParameters {
    //    fn new() -> Self {
    //        Default::default()
    //    }
    //}

    //impl Default for MeasurementParameters {
    //    fn default() -> Self {
    //        MeasurementParameters {
    //            pin: 0,
    //            measurement_type: MeasurementTypes::Current,
    //            calibration_value: 0.0,
    //           buffer: Vec::new(),
    //        }
    //    }
    //}

    #[shared]
    struct Shared {
        transfer: DMATransfer,
        adc_timer: CounterHz<TIM3>,
        counter: i32,
        //voltage: MeasurementParameters,
        //current: MeasurementParameters,
        adc_value_voltage: u16,
        adc_value_current: u16,
        measurements: Vec<MeasurementParameters, 8>,
    }

    #[local]
    struct Local {
        dma_buffer: Option<&'static mut [u16; 3]>,
        serial_tx: Tx<USART1>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let device: pac::Peripherals = cx.device;

        let rcc = device.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(100.MHz()).freeze();

        let gpioa = device.GPIOA.split();
        let pin_voltage = gpioa.pa7.into_analog();
        let pin_current_1 = gpioa.pa0.into_analog();
        let pin_current_2 = gpioa.pa3.into_analog();

        let dma = StreamsTuple::new(device.DMA2);

        let config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true)
            .double_buffer(false);

        let adc_config = AdcConfig::default()
            .dma(Dma::Continuous)
            .scan(Scan::Enabled);

        let mut adc = Adc::adc1(device.ADC1, true, adc_config);
        adc.configure_channel(&pin_voltage, Sequence::One, SampleTime::Cycles_480);
        adc.configure_channel(&pin_current_1, Sequence::Two, SampleTime::Cycles_480);
        adc.configure_channel(&pin_current_2, Sequence::Three, SampleTime::Cycles_480);

        let first_buffer = cortex_m::singleton!(: [u16; 3] = [0; 3]).unwrap();
        let second_buffer = Some(cortex_m::singleton!(: [u16; 3] = [0; 3]).unwrap());
        let transfer = Transfer::init_peripheral_to_memory(dma.0, adc, first_buffer, None, config);

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

        let mut adc_timer = device.TIM3.counter_hz(&clocks);

        let mut counter: i32 = 0;

        let mut measurements: Vec<MeasurementParameters, 8> = Vec::new();

        let voltage = MeasurementParameters {
            pin: 0,
            measurement_type: MeasurementTypes::Voltage,
            calibration_value: 0.0,
            buffer: Vec::new(),
            sum_simple_moving_average: 0,
        };

        let current = MeasurementParameters {
            pin: 1,
            measurement_type: MeasurementTypes::Current,
            calibration_value: 0.0,
            buffer: Vec::new(),
            sum_simple_moving_average: 0,
        };

        measurements.push(voltage);
        measurements.push(current);

        let mut adc_value_voltage: u16 = 0;
        let mut adc_value_current: u16 = 0;

        task_1::spawn().unwrap();

        (
            Shared {
                transfer,
                adc_timer,
                counter,
                //voltage,
                //current,
                adc_value_current,
                adc_value_voltage,
                measurements,
            },
            Local {
                dma_buffer: second_buffer,
                serial_tx,
            },
            init::Monotonics(),
        )
    }

    #[task(shared = [adc_timer])]
    fn task_1(mut cx: task_1::Context) {
        cx.shared.adc_timer.lock(|adc_timer| {
            adc_timer.listen(Event::Update);
            adc_timer.start(10000.Hz()).unwrap();
        });
    }

    #[task(binds = TIM3, shared = [transfer, adc_timer, counter, measurements])]
    fn start_adc(mut cx: start_adc::Context) {
        cx.shared.transfer.lock(|transfer| {
            transfer.start(|adc| {
                adc.start_conversion();
            });
        });

        cx.shared.adc_timer.lock(|adc_timer| {
            adc_timer.clear_interrupt(Event::Update);
        });
    }

    #[task(binds = DMA2_STREAM0, shared = [transfer, adc_timer, counter, measurements], local = [dma_buffer, serial_tx])]
    fn dma(mut cx: dma::Context) {
        //let dma::Context { mut shared, local } = cx;
        let dma_buffer = cx.shared.transfer.lock(|transfer| {
            let (dma_buffer, _) = transfer
                .next_transfer(cx.local.dma_buffer.take().unwrap())
                .unwrap();

            dma_buffer
        });

        cx.shared.measurements.lock(|measurements| {
            for i in 0..measurements.len() {
                measurements[i].buffer.push(dma_buffer[i]);

                if measurements[i].buffer.len() > 9 {
                    calculate_adc_offset::spawn();
                }
            }
        });

        *cx.local.dma_buffer = Some(dma_buffer);

        cx.shared.counter.lock(|counter| {
            *counter = *counter + 1;
        });
    }

    #[task(shared = [measurements, counter, adc_timer])]
    fn calculate_adc_offset(mut cx: calculate_adc_offset::Context) {
        cx.shared.measurements.lock(|measurements| {
            for i in 0..measurements.len() {
                measurements[i].sum_simple_moving_average = 
                    measurements[i].sum_simple_moving_average +
                        (measurements[i].buffer.iter().sum::<u16>() as u128
                        / measurements[i].buffer.len() as u128);
                    measurements[i].buffer.remove(0);
            }
        });
        cx.shared.counter.lock(|counter| {
            if counter == &mut 1999 {
                cx.shared.adc_timer.lock(|adc_timer| {
                    adc_timer.cancel().unwrap();
                });
                cx.shared.measurements.lock(|measurements| {
                    for i in 0..measurements.len() {
                        defmt::println!("{}", measurements[i].sum_simple_moving_average/2000)
                    }
                })
            }
        });

    }

}
