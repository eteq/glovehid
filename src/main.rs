#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m;
use cortex_m_rt::entry;

use nrf52840_hal as hal;
use nrf52840_hal::prelude::*;
use nrf52840_hal::gpio::{Pin, Output, PushPull, Level};
use nrf52840_hal::twim::{Twim, Pins, Frequency};

#[entry]
fn main() -> ! {
    let p = hal::pac::Peripherals::take().unwrap();
    let core = hal::pac::CorePeripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(p.P0);
    let port1 = hal::gpio::p1::Parts::new(p.P1);

    let mut delay = hal::Delay::new(core.SYST);

    let mut led = port0.p0_06.into_push_pull_output(Level::Low).degrade();
    
    let dot_dat: &mut Pin<Output<PushPull>> = &mut port0.p0_08.into_push_pull_output(Level::Low).degrade();
    let dot_clk: &mut Pin<Output<PushPull>> = &mut port1.p1_09.into_push_pull_output(Level::Low).degrade();
    
    let sw2 = port0.p0_29.into_pullup_input();

    let scl = port0.p0_14.into_floating_input().degrade();
    let sda = port0.p0_16.into_floating_input().degrade();
    let mut twim = Twim::new(p.TWIM0, Pins { scl, sda }, Frequency::K100);

    const LIS3MDL_ADDR:u8 = 0x1c;
    const LSM6DSOX_ADDR:u8 = 0x6a;

    let wr_buffer = [0x0F];
    let mut rd_buffer = [0 ; 1];

    twim.copy_write_then_read(LIS3MDL_ADDR, &wr_buffer, &mut rd_buffer).ok();

    loop {
        //blink_byte(rd_buffer[0], &mut led, &mut delay);
        dotstar_byte(rd_buffer[0], 0.05, dot_dat, dot_clk, &mut delay);
        delay.delay_ms(1000_u16);
    }
}


fn set_dotstar_color(r:f32, g:f32, b:f32, brightness:f32, 
    dat: &mut Pin<Output<PushPull>>, clk: &mut Pin<Output<PushPull>>) {
    let rbyte = (r * 255.) as u8;
    let gbyte = (g * 255.) as u8;
    let bbyte = (b * 255.) as u8;
    let firstbyte = ((brightness * 31.) as u8) + 0b1110000;
    
    let txbuffer = [0, 0, 0, 0,
                    firstbyte, bbyte, gbyte, rbyte,
                    0xff, 0xff, 0xff, 0xff
                    ];

    clk.set_high().ok();
    cortex_m::asm::delay(200_u32);


    for b in txbuffer.iter() {
        for i in 0..8 {
            clk.set_low().ok();
            if 0b10000000 & (b << i) == 0 { 
                dat.set_low().ok();
            } else {
                dat.set_high().ok();
            }
            clk.set_high().ok();
        }
    }
    clk.set_low().ok();
}

fn dotstar_byte(val: u8, brightness: f32,
                dat: &mut Pin<Output<PushPull>>, clk: &mut Pin<Output<PushPull>>, 
                delay: &mut hal::Delay) {
    for i in 0..8 {
        if (val << i & 0b10000000) == 0 {
            set_dotstar_color(1., 0., 0.1, brightness, dat, clk);
        } else {
            set_dotstar_color(0., 1., 0.1, brightness, dat, clk);
        }
        delay.delay_ms(300_u16);
        set_dotstar_color(0., 0., 0., 0., dat, clk);
        delay.delay_ms(300_u16);
    }
}


// use core::panic::PanicInfo;
// //use core::sync::atomic::{self, Ordering};
// #[inline(never)]
// #[panic_handler]
// fn panic(_info: &PanicInfo) -> ! {
//     let p = hal::pac::Peripherals::take().unwrap();
//     let port0 = hal::gpio::p0::Parts::new(p.P0);
//     let port1 = hal::gpio::p1::Parts::new(p.P1);
//     let dot_dat: &mut Pin<Output<PushPull>> = &mut port0.p0_08.into_push_pull_output(Level::Low).degrade();
//     let dot_clk: &mut Pin<Output<PushPull>> = &mut port1.p1_09.into_push_pull_output(Level::Low).degrade();
    
//     set_dotstar_color(1., 0., 0., 0.5, dot_dat, dot_clk);

//     loop {
//         for i in 0..31 {
//             set_dotstar_color(1., 0., 0., (i as f32)/31., dot_dat, dot_clk);
//             cortex_m::asm::delay(64_000_000/31 as u32);
//         }
//     }
// }
