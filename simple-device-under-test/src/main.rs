#![no_main]
#![no_std]

extern crate panic_rtt_target;

use lpc8xx_hal::{
    cortex_m_rt::entry, delay::Delay, gpio::Level, prelude::*, CorePeripherals,
    Peripherals,
};
#[allow(unused_imports)]
use rtt_target::rprintln; // for debug messages

/// Simple example app for a test target.
/// The device behaves as follows
///
/// ----------
/// |         |
/// |     [PIN 31] <~~~ in ~~~   Low/High signal
/// |      ↓  |
/// |     [PIN 29] ~~~ out ~~~>  ¬signal read at PIN 31
/// |         |
/// |     [PIN 30] ~~~ out ~~~>  ¬signal read at PIN 31
/// |         |
/// -----------

#[entry]
fn main() -> ! {
    const PIN_READ_DELAY : u16 = 700_u16;
    rtt_target::rtt_init_print!();

    // Get access to the device's peripherals.
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();

    // Initialize the APIs of the peripherals we need.
    let mut delay = Delay::new(cp.SYST);
    let gpio = {
        let mut syscon = p.SYSCON.split();
        p.GPIO.enable(&mut syscon.handle)
    };

    // Select pins we want to use
    let (pio1_0, pio1_0_token) = (p.pins.pio1_0, gpio.tokens.pio1_0); // green led
    let (pio1_2, pio1_2_token) = (p.pins.pio1_2, gpio.tokens.pio1_2); // red   led
    let (pio1_1, pio1_1_token) = (p.pins.pio1_1, gpio.tokens.pio1_1); // blue led

    // Configure the pin directions
    let pin_31 = pio1_0.into_input_pin(pio1_0_token);
    let mut pin_29 = pio1_2.into_output_pin(pio1_2_token, Level::High); // red led off
    let mut pin_30 = pio1_1.into_output_pin(pio1_1_token, Level::High);

    // check / update our pins every PIN_READ_DELAY ms
    // (we're polling instead of listening on interrupts here to keep things simple)
    loop {
        // pin 29 always outputs the inverse of the signal that's read on pin 31
        match pin_31.is_low() {
            true  => pin_29.set_high(),
            false => pin_29.set_low(),
        }

        // pin 1 gets toggled periodically
        match pin_30.is_set_high() {
            true  => pin_30.set_low(),
            false => pin_30.set_high(),
        }

        // wait until the next turn
        delay.delay_ms(PIN_READ_DELAY);
    }
}
