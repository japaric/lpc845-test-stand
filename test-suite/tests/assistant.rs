//! Test suite for the test-assitant "user interface" for pin configuration during tests

use lpc845_messages::{PinNumber, VoltageLevel};
use lpc845_test_suite::{Result, TestStand};

const TEST_PIN: PinNumber = 29; // this is the red led

#[test]
fn in_out_changes_should_be_consuming() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;

    // RUN TEST
    let in_pin = test_stand.assistant.create_gpio_input_pin(TEST_PIN)?;
    let _out_pin = in_pin.to_output_pin(VoltageLevel::Low)?;

    // Note: calling
    // in_pin.is_low();
    // here would lead to a compile error, because the `InputPin` has been converted to an `OutputPin`
    // by the call to `to_output_pin()`

    Ok(())
}

#[test]
fn pin_should_not_be_creatable_twice() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    let in_pin_1 = test_stand.assistant.create_gpio_input_pin(TEST_PIN);
    assert!(in_pin_1.is_ok());

    // RUN TEST
    let in_pin_2 = test_stand.assistant.create_gpio_input_pin(TEST_PIN);

    // ASSERT POSTCONDITION
    assert!(in_pin_2.is_err());

    Ok(())
}
