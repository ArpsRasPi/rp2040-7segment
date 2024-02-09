//! Uses a Pico to drive a 74xx595 to illumiate digits on a 7-segment display.
//!
//! This code assumes that the outputs Qa-Qg on the shift register are wired to segments a-g respectively on the 7-segment
//! display. Typically the segments are labled 'a' to the top segment, then assigned alphabetically in a clockwise
//! order until finally assigning 'g' to the centre segment.
//!
//! In the code, SRCLK, RCLK and SERIAL lines are connected toi GPIO pins 13, 14 and 15
//! respectively.
//!
#![no_std]
#![no_main]

use core::convert::Infallible;

use bsp::entry;
use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{OutputPin, PinState};
// use panic_probe as _;
extern crate panic_probe;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

extern crate panic_halt;

use pio_proc::pio_file;

use hal::pio::PIOExt;
use rp2040_hal as hal;

// Constants defining how digits are displayed on the 7-segment display using bits of a u8 to
// represent segments. Here 1 = a, 2 = b, 4 = c ... etc
const ONE: u8 = 2 + 4;
const TWO: u8 = 1 + 2 + 64 + 16 + 8;
const THREE: u8 = 1 + 2 + 64 + 4 + 8;
const FOUR: u8 = 32 + 2 + 64 + 4;
const FIVE: u8 = 1 + 32 + 64 + 4 + 8;
const SIX: u8 = 1 + 32 + 64 + 4 + 8 + 16;
const SEVEN: u8 = 1 + 2 + 4;
const EIGHT: u8 = 1 + 2 + 4 + 8 + 16 + 32 + 64;
const NINE: u8 = 1 + 2 + 32 + 64 + 4 + 8;
const ZERO: u8 = 1 + 2 + 4 + 8 + 16 + 32;

// Convinience array for mapping numeric value (the index of the array) to display value
const DIGITS: [u8; 10] = [ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE];

/// Entry point for embedded no_std applications.
///
/// Must not exit.
#[entry]
fn main() -> ! {
    info!("Program start");

    // Standard device set up
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

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Device set up ends

    // Change the GPIO pin assignments here
    let mut srclk = pins.gpio13.into_push_pull_output_in_state(PinState::Low);
    let mut rclk = pins.gpio14.into_push_pull_output_in_state(PinState::Low);
    let mut serial = pins.gpio15.into_push_pull_output_in_state(PinState::Low);

    let program_with_defines = pio_file!(
        "./src/shift_reg.pio",
        select_program("shift_reg"), // Optional if only one program in the file
        options(max_program_size = 32)  // Optional, defaults to 32
    );

    let program = program_with_defines.program;

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let (int, frac) = (0, 0); // as slow as possible (0 is interpreted as 65536)
    let (mut sm, _, _) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
        .set_pins(serial.id(), 1)
        .side_set_pin_base(srclk.id())
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);

    sm.start();

    // Endless loop - main method dies not exit.
    loop {
        // Iterate over digits and display them
        for digit in DIGITS {
            sm.write(digit as u32);
            delay.delay_ms(200);
        }
    }
}

/// Sets the state of the 7-segment display
///
/// Each segment is represented by a bit in a u8
///
/// Setting the state of the segments is via a 74xx595 shift register.
///
/// We only use 3 input pins on the 74xx595:
/// rclk: The storage register clock. On the rising edge, will transfer the shift register contents
/// to the storage, and therefore the output pins driving the segmets (if output enable is tied to
/// on)
/// srclk: The shift register clock. On the rising edge, will transfer the value of the serial pin
/// to the first register, shifting what was in there to the second, and so on.
/// serial: The input pin for the value to set to the first register on the rising edge of srclk
///
/// An outline of how these pins are manipulated to set the segments is therefore:
///
/// Pull rclk and srclk Low
/// For each bit in value
///  Set serial to !bit (as the display is common anode)
///  Strobe srclk
/// Push rclk High
fn set_display<PRCLK, PSRCLK, PSERIAL>(
    value: u8,
    rclk: &mut PRCLK,
    srclk: &mut PSRCLK,
    serial: &mut PSERIAL,
    delay: &mut Delay,
) where
    PRCLK: OutputPin<Error = Infallible>,
    PSRCLK: OutputPin<Error = Infallible>,
    PSERIAL: OutputPin<Error = Infallible>,
{
    info!("Setting display");
    set_pin(rclk, PinState::Low, delay);
    set_pin(srclk, PinState::Low, delay);

    // We push in the highest value first because it'll get shifted into the highest register by the
    // time we have pushed in the final bit
    for i in (0..8).rev() {
        match value & (1 << i) > 0 {
            true => set_pin(serial, PinState::Low, delay),
            false => set_pin(serial, PinState::High, delay),
        }
        set_pin(srclk, PinState::High, delay);
        set_pin(srclk, PinState::Low, delay);
    }
    set_pin(rclk, PinState::High, delay);
    set_pin(rclk, PinState::Low, delay);
}

/// Convinence function to set a pin state that allows a short delay to be added
///
/// This was added because the 74HC595N I was using when I wrote this would start to misbehave at
/// 3.3v and full speed. Even a delay of 0us was enough to correct this! This was done without an
/// oscilloscope to make sure that was actually the problem.
///
/// In a next iteration I'll use PIO which _should_ allow specific timings
fn set_pin<P: OutputPin<Error = Infallible>>(pin: &mut P, state: PinState, delay: &mut Delay) {
    info!("Setting pin");
    pin.set_state(state).unwrap();
    delay.delay_us(0);
}
