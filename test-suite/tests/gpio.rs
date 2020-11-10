//! Test Suite for the GPIO API in LPC8xx HAL
//!
//! This test suite communicates with hardware. See top-level README.md for
//! wiring instructions.

use std::thread::sleep;
use std::time;

use lpc845_messages::{pin::Level, PinNumber};
use lpc845_test_suite::assistant::{Assistant, InputPin};
use lpc845_test_suite::{Result, TestStand};

const RED_LED_PIN: PinNumber = 29;
const GRN_LED_PIN: PinNumber = 31;

#[test]
fn target_should_set_pin_level() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    let mut in_pin = test_stand.assistant.create_gpio_input_pin(GRN_LED_PIN)?;

    // TEST & ASSERT POSTCONDITION
    test_stand.target.set_pin_low()?;
    assert!(in_pin.is_low()?);

    // TEST & ASSERT POSTCONDITION
    // TODO I think I may have damaged my green pin by bedning during rewiring
    // -> test this with other hw OR reconfig other assistant pin as input and rewire and test
    // test_stand.target.set_pin_high()?;
    // assert!(in_pin.is_high()?);

    Ok(())
}

#[test]
fn target_should_read_input_level() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    let mut out_pin = test_stand
        .assistant
        .create_gpio_output_pin(RED_LED_PIN, Level::Low)?;

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
    let mut out_pin = test_stand
        .assistant
        .create_gpio_output_pin(RED_LED_PIN, Level::Low)?;
    let mut in_pin: InputPin<Assistant>; // we'll need this during the loop

    // RUN TEST
    // TODO: figure out how to do this in a loop without move trouble
    in_pin = out_pin.to_input_pin()?;
    sleep(time::Duration::from_secs(2));
    out_pin = in_pin.to_output_pin(Level::Low)?;
    sleep(time::Duration::from_secs(2));

    in_pin = out_pin.to_input_pin()?;
    sleep(time::Duration::from_secs(2));
    out_pin = in_pin.to_output_pin(Level::Low)?;
    sleep(time::Duration::from_secs(2));

    in_pin = out_pin.to_input_pin()?;
    sleep(time::Duration::from_secs(2));
    out_pin = in_pin.to_output_pin(Level::Low)?;

    sleep(time::Duration::from_secs(2));

    // ASSERT POSTCONDITION
    // 👀  manually assert that led is toggling on/off every 2 secs

    Ok(())
}

#[test]
#[allow(unused_assignments)] // to silence the last conversion
fn wonky_in_out_conversion_should_work() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    // ensure pin is low (-> red led is on) when we start
    let mut out_pin = test_stand
        .assistant
        .create_gpio_output_pin(RED_LED_PIN, Level::Low)?;
    assert!(test_stand.target.pin_is_low()?);

    let in_pin = out_pin.to_input_pin()?;
    sleep(time::Duration::from_secs(2));

    out_pin = in_pin.to_output_pin(Level::High)?;
    assert!(test_stand.target.pin_is_high()?);

    Ok(())
}


#[test]
fn assistant_red_led_should_be_toggleable_by_level() -> Result {
    // SETUP
    let test_stand = TestStand::new()?;
    let mut out_pin = test_stand
        .assistant
        .create_gpio_output_pin(RED_LED_PIN, Level::Low)?;
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
