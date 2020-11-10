//! Test suite for the test-assitant "user interface" for pin configuration during tests

use lpc845_messages::{Level, PinNumber};
use lpc845_test_suite::{Result, TestStand};

const TEST_PIN: PinNumber = 29; // this is the red led

#[test]
#[allow(unused_assignments)] // to silence the last conversion
fn in_out_changes_should_be_consuming() -> Result {
    // SETUP
    let test_assistant = TestStand::new()?.assistant;

    // RUN TEST
    let mut in_pin = test_assistant.create_gpio_input_pin(TEST_PIN)?;
    let out_pin = in_pin.to_output_pin(Level::Low)?;

    // Note: calling
    // in_pin.is_low();
    // here would lead to a compile error, because the `InputPin` has been converted to an `OutputPin`
    // by the call to `to_output_pin()`

    // ..and convert back
    in_pin = out_pin.to_input_pin()?;

    Ok(())
}

#[test]
fn in_pin_should_not_be_creatable_twice() -> Result {
    // SETUP
    let test_assistant = TestStand::new()?.assistant;
    let in_pin_1 = test_assistant.create_gpio_input_pin(TEST_PIN)?;

    // RUN TEST
    let in_pin_2 = test_assistant.create_gpio_input_pin(TEST_PIN);

    // ASSERT POSTCONDITION
    assert!(in_pin_2.is_err());

    Ok(())
}

#[test]
fn out_pin_should_not_be_creatable_twice() -> Result {
    // SETUP
    let test_assistant = TestStand::new()?.assistant;
    let out_pin_1 = test_assistant.create_gpio_output_pin(TEST_PIN)?;

    // RUN TEST
    let out_pin_2 = test_assistant.create_gpio_output_pin(TEST_PIN);

    // ASSERT POSTCONDITION
    assert!(out_pin_2.is_err());

    Ok(())
}
