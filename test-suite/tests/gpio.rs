//! Test Suite for the GPIO API in LPC8xx HAL
//!
//! This test suite communicates with hardware. See top-level README.md for
//! wiring instructions.

use std::thread::sleep;
use std::time;

use lpc845_test_suite::{
    Result,
    TestStand,
};

#[test]
fn it_should_set_pin_level() -> Result {
    let mut test_stand = TestStand::new()?;

    test_stand.target.set_pin_low()?;
    assert!(test_stand.assistant.pin_is_low()?);

    test_stand.target.set_pin_high()?;
    assert!(test_stand.assistant.pin_is_high()?);

    Ok(())
}

#[test]
fn it_should_read_input_level() -> Result {
    let mut test_stand = TestStand::new()?;

    test_stand.assistant.set_pin_low()?;
    assert!(test_stand.target.pin_is_low()?);

    test_stand.assistant.set_pin_high()?;
    assert!(test_stand.target.pin_is_high()?);

    Ok(())
}

#[test]
fn red_should_light_up_on_low() -> Result {
    let mut test_stand = TestStand::new()?;

    test_stand.assistant.set_pin_low()?;

    // 👀  manually assert that on-board led is red after 5 secs

    Ok(())
}

#[test]
fn toggle_direction_should_turn_off_red_led() -> Result {
    let mut test_stand = TestStand::new()?;

    // setup: ensure pin is low (-> red led is on) when we start
    test_stand.assistant.set_pin_low()?;
    sleep(time::Duration::from_secs(5));

    // run test
    test_stand.assistant.set_pin_direction_input()?;

    // 👀  manually assert that on-board led is off after 5 secs

    Ok(())
}
