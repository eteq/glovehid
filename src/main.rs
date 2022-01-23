#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m::asm;
use cortex_m_rt::entry;

use nrf52840_hal as hal;
use nrf52840_hal::prelude::*;
use nrf52840_hal::gpio::Level;

#[entry]
fn main() -> ! {
    let p = hal::pac::Peripherals::take().unwrap();
    let core = hal::pac::CorePeripherals::take().unwrap();
    let mut port0 = hal::gpio::p0::Parts::new(p.P0);

    let mut delay = hal::Delay::new(core.SYST);

    let mut led = port0.p0_06.into_push_pull_output(Level::Low);


    loop {
        led.set_high().unwrap();
        delay.delay_ms(250_u16);
        led.set_low().unwrap();
        delay.delay_ms(250_u16); 
    }
}
