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

fn set_dotstar_color_spi<T:hal::spim::Instance>(r:f32, g:f32, b:f32, brightness:f32, 
                            spim: &mut hal::Spim<T>,
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