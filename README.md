# A Rust based energy monitor for the STM32 platform

This repository will host the code for running an mains energy monitor on the STM32 platform, written in Rust.

As of October 2022:
- this code is written for the WeAct Black Pill (v2) which contains the STM32F411CEU6 microcontroller, but should be easily adaptable for other similar microcontrollers, and
- it contains minimally working code that is calibrated for my physical set up.

## Acknowledgements

Inspiration for the code to work out power flow comes from [EmonLib](https://github.com/openenergymonitor/EmonLib).
There are three slight differences to the original EmonLib:
- The code provides an inital preseeded starting point for the DC offset which is removed from the raw ADC values that are obtained. This starting point assumes that the DC offset is VCC/2 and the ADC has 12 bits of resolution. The original EmonLib uses a digital low-pass filter formula to slowly increase the offset from 0. This allows the early 'filtered' values for voltage and current value to be roughly correct instead of having to wait until the low-pass filter formula settles.
- The main measurement loop runs for a set amount of time (nominally 500ms) instead of running for a set number of wavelengths. This removes a set of calculations from the code.
- There is no correction for any phase error coming from the voltage transformer. The toroidal transformer I am using produced negligible phase error.

The code is written on top of the [app-template](https://github.com/knurling-rs/app-template) developed by [Knurling-rs](https://github.com/knurling-rs) and makes use of pieces of example code for the STM32 platform in the [STM32F4xx-hal crate](https://github.com/stm32-rs/stm32f4xx-hal).



