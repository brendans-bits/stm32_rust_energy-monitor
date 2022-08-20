# A Rust based energy monitor for the STM32 platform

This repository will host the code for running an mains energy monitor on the STM32 platform, written in Rust.

As of August 2020:
- this code is written for the WeAct Black Pill (v2) which contains the STM32F411CEU6 microcontroller, but should be easily adaptable for other similar microcontrollers, and
- it currently only contains a 'hello world' code example to get up and running.

## Acknowledgements

Inspiration for the code to work out energy comes from [EmonLib](https://github.com/openenergymonitor/EmonLib).

The code is written on top of the [app-template](https://github.com/knurling-rs/app-template) developed by [Knurling-rs](https://github.com/knurling-rs) and makes use of pieces of example code for the STM32 platform in the [STM32F4xx-hal crate](https://github.com/stm32-rs/stm32f4xx-hal).



