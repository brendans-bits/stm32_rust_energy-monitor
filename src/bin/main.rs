//#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

use rtic::app;
use stm32_rust_energy_monitor as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI0])]
mod app {

    use core::fmt::Write;
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

    #[shared]
    struct Shared {
        transfer: DMATransfer,
        adc_timer: CounterHz<TIM3>,
        counter: i32,
    }

    #[local]
    struct Local {
        buffer: Option<&'static mut [u16; 3]>,
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

        //polling::spawn_after(1.secs()).ok();

        task_1::spawn().unwrap();

        (
            Shared {
                transfer,
                adc_timer,
                counter,
            },
            Local {
                buffer: second_buffer,
                serial_tx,
            },
            init::Monotonics(),
        )
    }

    #[task(shared = [adc_timer])]
    fn task_1(mut cx: task_1::Context) {
        cx.shared.adc_timer.lock(|adc_timer| {
            adc_timer.listen(Event::Update);
            adc_timer.start(1000.Hz()).unwrap();
        });
    }

    #[task(binds = TIM3, shared = [transfer, adc_timer, counter])]
    fn start_adc(mut cx: start_adc::Context) {
        cx.shared.transfer.lock(|transfer| {
            transfer.start(|adc| {
                adc.start_conversion();
            });
        });

        cx.shared.counter.lock(|counter| {
            if counter == &mut 1999 {
                cx.shared.adc_timer.lock(|adc_timer| {
                    adc_timer.cancel().unwrap();
                });
            }
        });

        cx.shared.adc_timer.lock(|adc_timer| {
            adc_timer.clear_interrupt(Event::Update);
        });
    }

    #[task(binds = DMA2_STREAM0, shared = [transfer, adc_timer, counter], local = [buffer, serial_tx])]
    fn dma(mut cx: dma::Context) {
        //let dma::Context { mut shared, local } = cx;
        let buffer = cx.shared.transfer.lock(|transfer| {
            let (buffer, _) = transfer
                .next_transfer(cx.local.buffer.take().unwrap())
                .unwrap();

            buffer
        });

        let voltage = buffer[0];
        let current_1 = buffer[1];
        let current_2 = buffer[2];

        *cx.local.buffer = Some(buffer);

        cx.shared.counter.lock(|counter| {
            *counter = *counter + 1;
            writeln!(
                cx.local.serial_tx,
                "{}, {}, {}, {} \r",
                counter, voltage, current_1, current_2
            )
            .unwrap();
        });
    }
}
