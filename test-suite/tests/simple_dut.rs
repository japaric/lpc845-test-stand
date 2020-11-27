//! Test Suite to demonstrate how to check if the simple-deice-under-test firmware
//! behaves as expected.
//!
//! This test suite communicates with hardware.
//! TODO add wiring instructions

use std::thread::sleep;
use std::time;

use lpc845_messages::{pin::Level};
use lpc845_test_suite::{Result, TestStand};

#[test]
fn dut_pin_30_should_output_inverse_signal_on_pin_31_within_1_second() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    let mut test_assistant_pin_31 = test_stand
        .assistant
        // output pin because we're feeding signals into the device under test
        .create_gpio_output_pin(31, Level::Low)?;

    let mut test_assistant_pin_29 = test_stand
        .assistant
        // output pin because we're reading signals from the device under test
        .create_gpio_input_pin(29)?;

    // TODO dont forget to rewire!

    // TEST
    test_assistant_pin_31.set_high();

    // give Device Under Test some time to react
    sleep(time::Duration::from_secs(1));

    // ASSERT POSTCONDITION
    assert!(test_assistant_pin_29.is_low()?);
    //                            ^^^^^^^👀

    Ok (())
}