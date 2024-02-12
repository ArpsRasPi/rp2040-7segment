//! Uses a Pico to drive a 74xx595 to illumiate digits on a 7-segment display.
//!
//! This code assumes that the outputs Qa-Qh on the shift register are wired to segments a-h respectively on the 7-segment
//! display. Typically the segments are labled 'a' to the top segment, then assigned alphabetically in a clockwise
//! order until finally assigning 'g' to the centre segment and 'h' to a decimal (if present).
//!
//! In the code, SRCLK, RCLK and SERIAL lines are connected toi GPIO pins 13, 14 and 15
//! respectively.
//!
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{
        gpio::{FunctionPio0, Pin},
        pio::PinDir,
    },
};
use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::PinState;
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

use pio_proc::pio_file;

use bsp::hal::pio::PIOExt;

// Constants defining how digits are displayed on the 7-segment display using bits of a u8 to
// represent segments. Here 1 = a, 2 = b, 4 = c ... etc
// Note: this ordering is helpful because the pins are set in the PIO MSB first, so the
// 'h' bit gets pushed first. i.e. HGFEDCBA
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
    let srclk: Pin<_, FunctionPio0, _> = pins.gpio13.into_function();
    let rclk: Pin<_, FunctionPio0, _> = pins.gpio14.into_function();
    let serial: Pin<_, FunctionPio0, _> = pins.gpio15.into_function();

    // LED to get some feedback that words are being transmitted to the PIO SM
    let mut led_pin = pins.led.into_push_pull_output_in_state(PinState::Low);

    let program_with_defines = pio_file!(
        "./src/shift_reg.pio",
        select_program("shift_reg"), // Optional if only one program in the file
        options(max_program_size = 32)  // Optional, defaults to 32
    );

    let program = program_with_defines.program;

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    let (int, frac) = (0, 0); // as slow as possible (0 is interpreted as 65536)
    let (mut sm, mut _rx, mut tx) = bsp::hal::pio::PIOBuilder::from_program(installed)
        .out_pins(serial.id().num, 1)
        .side_set_pin_base(srclk.id().num)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);

    // Make sure all the pins are set to outputs
    sm.set_pindirs([
        (srclk.id().num, PinDir::Output),
        (rclk.id().num, PinDir::Output),
        (serial.id().num, PinDir::Output),
    ]);

    sm.start();

    // Endless loop - main method dies not exit.
    loop {
        // Iterate over digits and display them
        for digit in DIGITS {
            led_pin.set_low().unwrap();
            delay.delay_ms(200);
            // In the PIO assembly, OUT PINS, 1 shifts the MSB to the output pin
            // so we shift the 8 bits we need into the top byte of the word.
            // The word is then HGFEDCBA_00000000_00000000_00000000
            // Note: this is configurable using out_shift_direction in PIOBuilder
            let result = tx.write((!digit as u32) << 24);

            // If we were able to tx the word, flash the LED on.
            if result {
                led_pin.set_high().unwrap();
            }
            delay.delay_ms(800);
        }
    }
}
