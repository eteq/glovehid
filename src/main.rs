#![no_std]
#![no_main]

// pick a panicking behavior
// use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use panic_persist;

use cortex_m;
use cortex_m_rt::entry;

use heapless::Vec;

use nrf52840_hal as hal;
use nrf52840_hal::prelude::*;
use nrf52840_hal::gpio::{Pin, Output, PushPull, Level};
use nrf52840_hal::twim::{Twim, Pins, Frequency};
use core::fmt::Write;

// use nrf52840_hal::usbd::{UsbPeripheral, Usbd};
// use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
// use usbd_serial::{SerialPort, USB_CLASS_CDC, UsbError};


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

    // Set up uart on the regular tx/rx labeled pins
    let (uart0, uart_pins) = {
        (
            p.UARTE0,
            hal::uarte::Pins {
                txd: port0.p0_24.into_push_pull_output(Level::High).degrade(),
                rxd: port0.p0_25.into_floating_input().degrade(),
                cts: None,
                rts: None,
            },
        )
    };
    let mut uarte = hal::uarte::Uarte::new(
        uart0,
        uart_pins,
        hal::uarte::Parity::EXCLUDED,
        hal::uarte::Baudrate::BAUD115200,
    );

    write!(uarte, "Started glovehid!\r\n"); 
    // Check if there was a panic message, if so, send to UART
    if let Some(msg) = panic_persist::get_panic_message_utf8() {
        write!(uarte, "There was a panic last boot, reset to clear:\r\n"); 
        write!(uarte, "{}\r\n", msg); 
        loop {
            led.set_high().ok();
            delay.delay_ms(50_u8);
            led.set_low().ok();
            delay.delay_ms(50_u8);
        }
    }


    // set up the i2c bus
    let scl = port0.p0_14.into_floating_input().degrade();
    let sda = port0.p0_16.into_floating_input().degrade();
    let mut twim = Twim::new(p.TWIM0, Pins { scl, sda }, Frequency::K400);

    //const LIS3MDL_ADDR:u8 = 0x1c;
    //const LSM6DSOX_ADDR:u8 = 0x6a;
    const MPU6050_ADDR1:u8 = 0x68;
    const MPU6050_ADDR2:u8 = 0x69;

    let mpu_accel_scale = MPU6050AccelScale::G4; 
    let mpu_gyro_scale = MPU6050GyroScale::DPS500; 
    
    init_mpu6050(&mut twim, MPU6050_ADDR1, mpu_accel_scale, mpu_gyro_scale).expect("mpu6050 init failed");
    
    delay.delay_ms(100_u16); // let the thing catch up

    let mut i = 0;
    loop {
        delay.delay_us(7500_u16); // make sure we have at least one sample-and-a-half

        let (mut xaccel, mut yaccel, mut zaccel, mut xgyro, mut ygyro, mut zgyro) = {
            let fifo_dump = dump_fifo_mpu6050(&mut twim, MPU6050_ADDR1).expect("Fifo dump failure");
            fifo_dump_to_data_mpu6050(fifo_dump,  mpu_accel_scale, mpu_gyro_scale)
        };
        
        
        write!(uarte, "acceldump done, n={}\r\n", xaccel.len());
        
        if xaccel.len() > 0 {
            set_dotstar_color((255.*floatabs(xaccel.pop().unwrap())/4.0) as u8,
                            (255.*floatabs(yaccel.pop().unwrap())/4.0) as u8, 
                            (255.*floatabs(zaccel.pop().unwrap())/4.0) as u8, 
                            16, dot_dat, dot_clk);
        }
        i +=1 ;
    }
}

fn set_dotstar_color(rbyte:u8, gbyte:u8, bbyte:u8, brightness:u8, 
    dat: &mut Pin<Output<PushPull>>, clk: &mut Pin<Output<PushPull>>) {
    let firstbyte = brightness | 0b11100000;  // anything > 31 is effectively treated as max
    
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
            cortex_m::asm::delay(3_u32);
            clk.set_high().ok();
            cortex_m::asm::delay(3_u32);
        }
    }
    clk.set_low().ok();
}

#[derive(Clone, Copy)]
enum MPU6050AccelScale {
    G2, G4, G8, G16 
}
impl MPU6050AccelScale {
    fn lsb_per_g(&self) -> u16 {
        match self {
            MPU6050AccelScale::G2 => { return 16_384; },
            MPU6050AccelScale::G4 => { return 8_192; },
            MPU6050AccelScale::G8 => { return 4_096; },
            MPU6050AccelScale::G16 => { return 2_048; },
        }
    }
}

#[derive(Clone, Copy)]
enum MPU6050GyroScale {
    DPS250, DPS500, DPS1000, DPS2000 
}
impl MPU6050GyroScale {
    fn lsb_per_dps(&self) -> f32 {
        match self {
            MPU6050GyroScale::DPS250 => { return 131.; },
            MPU6050GyroScale::DPS500 => { return 65.5; },
            MPU6050GyroScale::DPS1000 => { return 32.8; },
            MPU6050GyroScale::DPS2000 => { return 16.4; },
        }
    }
}

fn init_mpu6050<T: hal::twim::Instance>(twim: &mut Twim<T>, addr: u8,
                                        ascale: MPU6050AccelScale,
                                        gscale: MPU6050GyroScale)
                                        -> Result<(), hal::twim::Error> {
    let gscale_reg27 : u8  = match gscale {
        MPU6050GyroScale::DPS250 => 0b00000000,
        MPU6050GyroScale::DPS500 => 0b00001000,
        MPU6050GyroScale::DPS1000 => 0b00010000,
        MPU6050GyroScale::DPS2000 => 0b00011000,
    };
    let ascale_reg28 : u8  = match ascale {
        MPU6050AccelScale::G2 =>  0b00000000,
        MPU6050AccelScale::G4 => 0b00001000,
        MPU6050AccelScale::G8 => 0b00010000,
        MPU6050AccelScale::G16 => 0b00011000,
    };

    // Make sure this is really an MPU 6050 by checking the WHO_AM_I reguster
    let whoami_reg_addr = [117_u8];
    let mut whoami_buffer = [0_u8];
    twim.write_then_read(addr, &whoami_reg_addr, &mut whoami_buffer)?;
    if whoami_buffer[0] != 0b01101000 {
        return Err(hal::twim::Error::Overrun); // No other really meaningful flag...
    }
    

    // // reset the device, wait for the reset bit to clear
    // let reset_buffer = [107, 0b10000000];
    // twim.write(addr, &reset_buffer)?;
    // while reset_buffer[1] & 0b1000000 != 0 {
    //     let mut read_buffer = [1_u8];
    //     twim.write_then_read(addr, &reset_buffer[0..1], &mut read_buffer)?;
    // }
    // This makes it not work for unclear reasons.  Maybe a wait is required?  But don't really care

    // these are all register, setting pairs, so should be written 2-at-a-time 
    let init_buffer = [26, 0b00000001, //CONFIG -> 184 Hz bandwidth, Fs = 1khz for everything
                       // Desired sample rate is 200 Hz.  According to register map, SR = Gyro_rate / (1+SMPLRT_DIV), and gyro rate = 1000, so:
                       25, 4, //SMPRT_DIV 
                       27, gscale_reg27, //GYRO_CONFIG
                       28, ascale_reg28, //ACCEL_CONFIG
                       35, 0b01111000, //FIFO_EN -> xg,yg,zg,accel
                       106, 0b01000000, //USER_CTRL -> FIFO_EN
                       107, 0b00000001, //PWR_MGMT_1 -> power on, x-gyro-axis as clock source
                       //108, 0b00000000, //PWR_MGMT_2 -> defaults all 0 already
                       ];
                       
    for i in (0..init_buffer.len()).step_by(2) {
        twim.write(addr, &init_buffer[i..i+2])?;
    }
    Ok(())
}

fn reset_fifo_mpu6050<T: hal::twim::Instance>(twim: &mut Twim<T>, addr: u8)
                                             -> Result<(), hal::twim::Error>  {
    let buffer = [106, 0b01000100];  //USER_CTRL -> FIFO_EN, FIFO_RESET
    twim.write(addr, &buffer)
}

fn dump_fifo_mpu6050<T: hal::twim::Instance>(twim: &mut Twim<T>, addr: u8)
                                             -> Result<Vec<u8, 1024>, hal::twim::Error>  {
    // Note this only dumps multiples of 12.  But if the FIFO is full, it will clear it to make sure we start the sequence again in the right place.
    let register_addresses = [114, 116];  // FIFO_COUNTH, FIFO_RW
    let mut count_buffer = [0_u8; 2];
    twim.write_then_read(addr, &register_addresses[0..1], &mut count_buffer)?;
    let counts : u16 = count_buffer[1] as u16 + (count_buffer[0] as u16) << 8;
    let reads = counts/12;

    // vector that's as big as the FIFO can be
    let mut fifo_output: Vec<u8, 1024> = Vec::new();

    let mut rd_buffer = [0_u8; 12];
    for _ in 0..reads {
        twim.write_then_read(addr, &register_addresses[1..2], &mut rd_buffer)?;
        for read in rd_buffer {
            fifo_output.push(read).expect("vector full, but this should be impossible unless FIFO broken");
        }
    }

    if counts == 1024 {
        // the fifo is out-of-sync with the length-12 streams, so clear it now
        reset_fifo_mpu6050(twim, addr)?;
    }
    return Ok(fifo_output)
}

fn fifo_dump_to_data_mpu6050(mut fifo_output: Vec<u8, 1024>, 
                             accel_scale: MPU6050AccelScale, 
                             gyro_scale: MPU6050GyroScale) 
                             -> (Vec<f32, 85>, Vec<f32, 85>, Vec<f32, 85>, Vec<f32, 85>, Vec<f32, 85>, Vec<f32, 85>) {
    // 85 = floor(1024 / 14)
    let mut xaccel: Vec<f32, 85> = Vec::new();
    let mut yaccel: Vec<f32, 85> = Vec::new();
    let mut zaccel: Vec<f32, 85> = Vec::new();
    let mut xgyro: Vec<f32, 85> = Vec::new();
    let mut ygyro: Vec<f32, 85> = Vec::new();
    let mut zgyro: Vec<f32, 85> = Vec::new();

    let accel_scale_factor = accel_scale.lsb_per_g() as f32;
    let gyro_scale_factor = gyro_scale.lsb_per_dps();

    // we go from the top index down because that's easier to use with step_by and not fall off the end
    for i in (12..fifo_output.len()+1).step_by(12) {
        xaccel.push(regval_to_data(fifo_output[i-12], fifo_output[i-11], accel_scale_factor)).expect("data vector full");
        yaccel.push(regval_to_data(fifo_output[i-10], fifo_output[i-9], accel_scale_factor)).expect("data vector full");
        zaccel.push(regval_to_data(fifo_output[i-8], fifo_output[i-7], accel_scale_factor)).expect("data vector full");

        xgyro.push(regval_to_data(fifo_output[i-6], fifo_output[i-5], gyro_scale_factor)).expect("data vector full");
        ygyro.push(regval_to_data(fifo_output[i-4], fifo_output[i-3], gyro_scale_factor)).expect("data vector full");
        zgyro.push(regval_to_data(fifo_output[i-2], fifo_output[i-1], gyro_scale_factor)).expect("data vector full");
    }

    (xaccel, yaccel, zaccel, xgyro, ygyro, zgyro)
}

fn regval_to_data(hregval: u8, lregval: u8, scale:f32) -> f32{
    let ival = (hregval as i16) << 8 | lregval as i16;
    (ival as f32) / scale
}

fn floatabs(f:f32) -> f32{
    const LOWER_31_BITS : u32 = 0x7fffffff;
    f32::from_bits(f.to_bits() & LOWER_31_BITS)
}