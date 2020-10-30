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
    assert!(test_stand.assistant.pin_is_low()?); // TODO fix this test

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
fn red_led_should_be_toggleable_by_pin_direction() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    // ensure pin is low (-> red led is on) when we start
    test_stand.assistant.set_pin_low()?;

    // RUN TEST
    for n in 0..10 {
        sleep(time::Duration::from_secs(2));

        // toggle back and forth between in/output
        if n % 2 == 0 {
            test_stand.assistant.set_pin_direction_input()?;
        } else {
            test_stand.assistant.set_pin_direction_output()?;
        }
    }

    // ASSERT POSTCONDITION
    // 👀  manually assert that led is toggling on/off 10 times every 2 secs

    Ok(())
}

#[test]
fn red_led_should_be_toggleable_by_level() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    test_stand.assistant.set_pin_direction_input()?;

    // RUN TEST
    for n in 0..10 {
        sleep(time::Duration::from_secs(2));

        // toggle back and forth between high/low
        if n % 2 == 0 {
            test_stand.assistant.set_pin_low()?;
            // TODO: why does this check blue led Levels and not red?
            assert!(test_stand.assistant.pin_is_low()?);
        } else {
            test_stand.assistant.set_pin_high()?;
            //assert!(test_stand.target.pin_is_high()?);
        }
    }

    // ASSERT POSTCONDITION
    // 👀  manually assert that led is toggling on/off 10 times every 2 secs

    Ok(())
}