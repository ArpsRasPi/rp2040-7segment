//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::convert::Infallible;

use bsp::{entry, hal::gpio::AnyPin};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{OutputPin, PinState};
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

const ONE: u8 = 2 + 4;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut srclk = pins.gpio13.into_push_pull_output_in_state(PinState::Low);
    let mut rclk = pins.gpio14.into_push_pull_output_in_state(PinState::Low);
    let mut serial = pins.gpio15.into_push_pull_output_in_state(PinState::Low);
    let value: u8 = 1;

    set_display(value, &mut rclk, &mut srclk, &mut serial);

    {
        // Pull rclk Low
        // For each bit
        //  Set serial to value
        //  Strobe srclk
        // Push rclk High
        rclk.set_low().unwrap();
        srclk.set_low().unwrap();

        for i in 0..8 {
            match value & (1 << i) > 0 {
                true => serial.set_low().unwrap(),
                false => serial.set_high().unwrap(),
            }
            srclk.set_high().unwrap();
            srclk.set_low().unwrap();
        }
        rclk.set_high().unwrap();
        rclk.set_low().unwrap();
    }

    let mut led_pin = pins.led.into_push_pull_output();
    loop {
        info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

fn set_display<PRCLK, PSRCLK, PSERIAL>(
    value: u8,
    rclk: &mut PRCLK,
    srclk: &mut PSRCLK,
    serial: &mut PSERIAL,
) where
    PRCLK: OutputPin<Error = Infallible>,
    PSRCLK: OutputPin<Error = Infallible>,
    PSERIAL: OutputPin<Error = Infallible>,
{
    info!("Setting display");
    // Pull rclk Low
    // For each bit
    //  Set serial to value
    //  Strobe srclk
    // Push rclk High
    rclk.set_low().unwrap();
    srclk.set_low().unwrap();

    for i in 0..8 {
        match value & (1 << i) > 0 {
            true => serial.set_low().unwrap(),
            false => serial.set_high().unwrap(),
        }
        srclk.set_high().unwrap();
        srclk.set_low().unwrap();
    }
    rclk.set_high().unwrap();
    rclk.set_low().unwrap();
}
