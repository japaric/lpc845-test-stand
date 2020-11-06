//! Test Suite for the GPIO API in LPC8xx HAL
//!
//! This test suite communicates with hardware. See top-level README.md for
//! wiring instructions.

use std::thread::sleep;
use std::time;

use lpc845_messages::{DynamicPin, PinNumber, VoltageLevel};
use lpc845_test_suite::{Result, TestStand};
use lpc845_test_suite::assistant::{Assistant, InputPin2};

const RED_LED_PIN: PinNumber = 29;
const GRN_LED_PIN: PinNumber = 31;

const GRN_LED: DynamicPin = DynamicPin::GPIO(31);

/*
#[test]
fn it_should_set_pin_level() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;

    let mut in_pin = test_stand.assistant.create_gpio_input_pin(GRN_LED_PIN)?;

    // TEST & ASSERT POSTCONDITION
    test_stand.target.set_pin_low()?;
    assert!(test_stand.assistant.is_low(&mut in_pin)?);

    // TEST & ASSERT POSTCONDITION
    //test_stand.target.set_pin_high()?;
    //assert!(test_stand.assistant.input_pin_is_high(GRN_LED)?);

    Ok(())
}
*/

#[test]
fn target_should_read_input_level() -> Result {
    // SETUP
    let mut test_stand =  TestStand::new()?;
    let mut out_pin = test_stand.assistant.create_gpio_output_pin(RED_LED_PIN)?;

    // RUN TEST
    out_pin.set_low()?;
    assert!(test_stand.target.pin_is_low()?);

    out_pin.set_high()?;
    assert!(test_stand.target.pin_is_high()?);

    Ok(())
}

#[test]
#[allow(unused_assignments)] // to silence the last conversion
fn assistant_red_led_should_be_toggleable_by_pin_direction() -> Result {
    // SETUP
    let test_stand = TestStand::new()?;
    // ensure pin is low (-> red led is on) when we start
    let mut out_pin = test_stand.assistant.create_gpio_output_pin(RED_LED_PIN)?;
    let mut in_pin: InputPin2<Assistant>; // we'll need this during the loop
    out_pin.set_low()?;

    // RUN TEST
    // TODO: figure out how to do this in a loop without move trouble

    in_pin = out_pin.to_input_pin()?;
    sleep(time::Duration::from_secs(2));
    out_pin = in_pin.to_output_pin(VoltageLevel::Low)?;
    sleep(time::Duration::from_secs(2));

    in_pin = out_pin.to_input_pin()?;
    sleep(time::Duration::from_secs(2));
    out_pin = in_pin.to_output_pin(VoltageLevel::Low)?;
    sleep(time::Duration::from_secs(2));

    in_pin = out_pin.to_input_pin()?;
    sleep(time::Duration::from_secs(2));
    out_pin = in_pin.to_output_pin(VoltageLevel::Low)?;
    sleep(time::Duration::from_secs(2));

    // ASSERT POSTCONDITION
    // 👀  manually assert that led is toggling on/off every 2 secs

    Ok(())
}

#[test]
fn assistant_red_led_should_be_toggleable_by_level() -> Result {
    // SETUP
    let test_stand = TestStand::new()?;
    let mut out_pin = test_stand.assistant.create_gpio_output_pin(RED_LED_PIN)?;
    out_pin.set_high()?;

    // RUN TEST
    sleep(time::Duration::from_secs(2));
    out_pin.set_low()?;
    sleep(time::Duration::from_secs(2));
    out_pin.set_high()?;

    sleep(time::Duration::from_secs(2));
    out_pin.set_low()?;
    sleep(time::Duration::from_secs(2));
    out_pin.set_high()?;

    sleep(time::Duration::from_secs(2));
    out_pin.set_low()?;
    sleep(time::Duration::from_secs(2));
    out_pin.set_high()?;

    // ASSERT POSTCONDITION
    // 👀  manually assert that led is toggling on/off every 2 secs

    Ok(())
}

/*
// TODO add tests checking that:
// - what happens if I call dyn read/write funtions on uninitialized pin?
// - I didn't break measure_gpio_period works , receive_from_target_usart_sync
#[test]
fn dynamic_interrupt_handlers_should_be_ignored_after_direction_switch() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    test_stand.assistant.set_pin_direction_input(GRN_LED)?;
    test_stand.target.set_pin_low()?;
    test_stand.assistant.set_pin_direction_output(GRN_LED)?;

    // RUN TEST
    let read_result = test_stand.assistant.input_pin_is_low(GRN_LED);

    // ASSERT POSTCONDITION
    assert!(read_result.is_err());

    Ok(())
}

#[test]
fn dynamic_input_calls_on_output_direction_should_yield_useful_error() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    test_stand.assistant.set_pin_direction_output(GRN_LED)?;

    // RUN TEST
    let read_result = test_stand.assistant.input_pin_is_low(GRN_LED);

    // ASSERT POSTCONDITION
    // TODO. make error more useful here
    assert!(read_result.is_err());

    Ok(())
}
*/