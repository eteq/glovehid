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
//use cortex_m::peripheral::SYST;

#[entry]
fn main() -> ! {
    let p = hal::pac::Peripherals::take().unwrap();
    let core = hal::pac::CorePeripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(p.P0);
    let port1 = hal::gpio::p1::Parts::new(p.P1);

    let mut delay = hal::Delay::new(core.SYST);

    let mut led = port0.p0_06.into_push_pull_output(Level::Low);
    
    let dot_data = port0.p0_08.into_push_pull_output(Level::Low);
    let dot_clk = port1.p1_09.into_push_pull_output(Level::Low);

    let spimiso = port1.p1_01.into_floating_input(); // unused
    let cs = port1.p1_03.into_push_pull_output(Level::Low); // unused
    
    let sw2 = port0.p0_29.into_pullup_input();

     //set_dotstar_color(0.5, 0.2, 0.3, 0.4, 
      //                 &mut dot_data.degrade(), &mut dot_clk.degrade());

    let pins = hal::spim::Pins {
        sck: dot_clk.degrade(),
        mosi: Some(dot_data.degrade()),
        miso: Some(spimiso.degrade()),
    };
    let mut spim = hal::Spim::new(
        p.SPIM0,
        pins,
        hal::spim::Frequency::M4,
        hal::spim::MODE_0,
        0,
    );
    

    let mut color_cycle: usize = 0;
    let mut switch_down = false;
    let csref: &mut Pin<Output<PushPull>> = &mut cs.degrade();
    loop {
        if sw2.is_low().unwrap() {
            switch_down = true;
        }  else if switch_down {
            // treat this as a press
            switch_down = false;
            color_cycle += 1;
        }
        led.set_high().unwrap();
        delay.delay_ms(50_u16);
        led.set_low().unwrap();
        delay.delay_ms(50_u16);
        
        set_dotstar_color_spi(0.5 * (color_cycle%3 ==0) as u8 as f32, 
                              0.5 * (color_cycle%3 ==1) as u8 as f32, 
                              0.5 * (color_cycle%3 ==2) as u8 as f32, 
                              0.04, &mut spim, csref);
    }
}

fn set_dotstar_color(r:f32, g:f32, b:f32, brightness:f32, 
                     data_pin: &mut Pin<Output<PushPull>>, 
                     clock_pin: &mut Pin<Output<PushPull>>) {
    // not sure if this is necesary but it's in the timing diagram..
    clock_pin.set_high().unwrap();

    let rbyte = (r * 255.) as u8;
    let gbyte = (g * 255.) as u8;
    let bbyte = (b * 255.) as u8;
    let brightnessbyte = (brightness * 32.) as u8;
    
    cortex_m::interrupt::free(|_| {
        // 32 bits of 1
        data_pin.set_low().unwrap();
        for _ in 0..32 {
            clock_pin.set_low().unwrap();
            cortex_m::asm::delay(8_u32);
            clock_pin.set_high().unwrap();
            cortex_m::asm::delay(8_u32);
        }

        // leading 3 bits
        data_pin.set_high().unwrap();
        for _ in 0..3 {
            clock_pin.set_low().unwrap();
            cortex_m::asm::delay(8_u32);
            clock_pin.set_high().unwrap();
            cortex_m::asm::delay(8_u32);
        }

        // brightness
        for i in 0..5 {

            if (brightnessbyte << i) & 0b10000 == 0 
            { data_pin.set_low().unwrap(); }
            else {  data_pin.set_low().unwrap(); }

            clock_pin.set_low().unwrap();
            cortex_m::asm::delay(8_u32);
            clock_pin.set_high().unwrap();
            cortex_m::asm::delay(8_u32);
        }

        // r
        for i in 0..8 {
            if (rbyte << i) & 0b10000000 == 0 
            { data_pin.set_low().unwrap(); }
            else {  data_pin.set_low().unwrap(); }

            clock_pin.set_low().unwrap();
            cortex_m::asm::delay(8_u32);
            clock_pin.set_high().unwrap();
            cortex_m::asm::delay(8_u32);
        }

        // g
        for i in 0..8 {
            if (gbyte << i) & 0b10000000 == 0 
            { data_pin.set_low().unwrap(); }
            else {  data_pin.set_low().unwrap(); }

            clock_pin.set_low().unwrap();
            cortex_m::asm::delay(8_u32);
            clock_pin.set_high().unwrap();
            cortex_m::asm::delay(8_u32);
        }

        // b
        for i in 0..8 {
            if (bbyte << i) & 0b10000000 == 0 
            { data_pin.set_low().unwrap(); }
            else {  data_pin.set_low().unwrap(); }

            clock_pin.set_low().unwrap();
            cortex_m::asm::delay(8_u32);
            clock_pin.set_high().unwrap();
            cortex_m::asm::delay(8_u32);
        }

        
        // 32 bits of 1
        data_pin.set_high().unwrap();
        for _ in 0..32 {
            clock_pin.set_low().unwrap();
            cortex_m::asm::delay(8_u32);
            clock_pin.set_high().unwrap();
            cortex_m::asm::delay(8_u32);
        }
        clock_pin.set_low().unwrap();
    });
}

fn set_dotstar_color_spi(r:f32, g:f32, b:f32, brightness:f32, 
                            spim: &mut hal::Spim<hal::pac::SPIM0>,
                            cs: &mut Pin<Output<PushPull>>) {
    let rbyte = (r * 255.) as u8;
    let gbyte = (g * 255.) as u8;
    let bbyte = (b * 255.) as u8;
    let firstbyte = ((brightness * 31.) as u8) + 0b1110000;

    let txbuffer = [0, 0, 0, 0,
                    firstbyte, bbyte, gbyte, rbyte,
                    0xff, 0xff, 0xff, 0xff
                    ];
    
    spim.write(cs, &txbuffer).expect("SPI write failed");
}