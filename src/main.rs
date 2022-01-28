#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m;
use cortex_m_rt::entry;

use heapless::String;
use core::fmt::Write;

use nrf52840_hal as hal;
use nrf52840_hal::prelude::*;
use nrf52840_hal::gpio::{Pin, Output, PushPull, Level};
use nrf52840_hal::twim::{Twim, Pins, Frequency};

use nrf52840_hal::usbd::{UsbPeripheral, Usbd};
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC, UsbError};

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

    // set up USB for serial communication
    let clocks = hal::Clocks::new(p.CLOCK);
    let clocks = clocks.enable_ext_hfosc();
    let usb_bus = Usbd::new(UsbPeripheral::new(p.USBD, &clocks));
    let mut usb_serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Erik's debug company")
        .product("Serial port")
        .serial_number("DEBUGGER-1")
        .device_class(USB_CLASS_CDC)
        .max_packet_size_0(64) // (makes control transfers 8x faster)
        .build();


    // set up the accelerometers
    let scl = port0.p0_14.into_floating_input().degrade();
    let sda = port0.p0_16.into_floating_input().degrade();
    let mut twim = Twim::new(p.TWIM0, Pins { scl, sda }, Frequency::K400);

    const LIS3MDL_ADDR:u8 = 0x1c;
    //const LSM6DSOX_ADDR:u8 = 0x6a;

    let read_mag_lis3mdl = [0x28 + 0b10000000];

    let mut inclick : bool = false;
    let mut clicks = 0;
    let mut lis3mdl_scale : LIS3MDLFullScale = LIS3MDLFullScale::G4;
    init_lis3mdl(&mut twim, LIS3MDL_ADDR, lis3mdl_scale).expect("lis3mdl init failed");

    loop {
        usb_dev.poll(&mut [&mut usb_serial]);

        if sw2.is_low().unwrap() {
            inclick = true;
        } else if inclick {
            inclick = false;

            clicks += 1;

            set_dotstar_color(0., 0., 0., 0., dot_dat, dot_clk);
            for _ in 0..clicks {
                led.set_high().ok();
                delay.delay_ms(200_u16);
                led.set_low().ok();
                delay.delay_ms(200_u16);
            }

            lis3mdl_scale = match clicks % 4 { 0 => LIS3MDLFullScale::G4,
                1 => LIS3MDLFullScale::G8,
                2 => LIS3MDLFullScale::G12,
                _ => LIS3MDLFullScale::G16};

            init_lis3mdl(&mut twim, LIS3MDL_ADDR, lis3mdl_scale).expect("lis3mdl scale-set failed");
        
            let wstr = match lis3mdl_scale {
                LIS3MDLFullScale::G4 => {"4 Gauss Range\n"},
                LIS3MDLFullScale::G8 => {"8 Gauss Range\n"},
                LIS3MDLFullScale::G12 => {"12 Gauss Range\n"},
                LIS3MDLFullScale::G16 => {"16 Gauss Range\n"}
            };
            usb_serial.write(wstr.as_bytes()).ok();
        }

        let mut rd_buffer = [0; 6];
        twim.write_then_read(LIS3MDL_ADDR, &read_mag_lis3mdl, &mut rd_buffer).ok();

        let scale: f32 = match lis3mdl_scale {LIS3MDLFullScale::G4=>6842.,
                                              LIS3MDLFullScale::G8=>3421.,
                                              LIS3MDLFullScale::G12=>2281.,
                                              LIS3MDLFullScale::G16=>1711.,};

        let x_mag: f32 = (rd_buffer[0] as i16 | (rd_buffer[1] as i16) << 8) as f32 / scale;
        let y_mag: f32 = (rd_buffer[2] as i16 | (rd_buffer[3] as i16) << 8) as f32 / scale;
        let z_mag: f32 = (rd_buffer[4] as i16 | (rd_buffer[5] as i16) << 8) as f32 / scale;

        let mut buf = [0u8; 64];
        let write_ser = match usb_serial.read(&mut buf[..]) {
            Ok(_count) => { true },
            Err(UsbError::WouldBlock) => { false },// No data received
            Err(_err) => { false }// An error occurred
        };
        if write_ser {
            let mut s: String<1024> = String::new();
            write!(s, "M-x: {}, y: {}, z: {}\n.,", (x_mag + 16.)/32., (y_mag + 16.)/32., (z_mag + 16.)/32.).expect("Can't write");
            usb_serial.write(s.as_bytes()).ok();
        }

        set_dotstar_color((x_mag + 16.)/32., (y_mag + 16.)/32., (z_mag + 16.)/32., 0.5, dot_dat, dot_clk);

        //blink_byte(rd_buffer[0], &mut led, &mut delay);
        //dotstar_byte(rd_buffer[0], 0.05, dot_dat, dot_clk, &mut delay);
        //delay.delay_ms(1000_u16);
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


// fn dotstar_byte(val: u8, brightness: f32,
//                 dat: &mut Pin<Output<PushPull>>, clk: &mut Pin<Output<PushPull>>, 
//                 delay: &mut hal::Delay) {
//     for i in 0..8 {
//         if (val << i & 0b10000000) == 0 {
//             set_dotstar_color(1., 0., 0.1, brightness, dat, clk);
//         } else {
//             set_dotstar_color(0., 1., 0.1, brightness, dat, clk);
//         }
//         delay.delay_ms(300_u16);
//         set_dotstar_color(0., 0., 0., 0., dat, clk);
//         delay.delay_ms(300_u16);
//     }
// }

#[derive(Clone, Copy)]
enum LIS3MDLFullScale {
    G4, G8, G12, G16
}

fn init_lis3mdl<T: hal::twim::Instance>(twim: &mut Twim<T>, addr: u8,
                                        fullscale: LIS3MDLFullScale)
                                        -> Result<(), hal::twim::Error> {
    let fullscale_reg : u8  = match fullscale {
        LIS3MDLFullScale::G4 => 0b00000000,
        LIS3MDLFullScale::G8 => 0b00100000,
        LIS3MDLFullScale::G12 => 0b01000000,
        LIS3MDLFullScale::G16 => 0b01100000,
    };

    let init_buffer = [0x20 + 0b10000000,  // high bit means repeated write
                       0b01111100, //CTRL_REG1 -> 80Hz, UHP, No FAST, Temp off
                       0b00000000 | fullscale_reg, //CTRL_REG2
                       0b00000000, //CTRL_REG3 -> continuous conversion mode
                       0b00001100, //CTRL_REG4 -> high perf z
                       //0b00000000, //CTRL_REG5
                       ];
    twim.write(addr, &init_buffer)
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
