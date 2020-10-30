//! Test Suite for the GPIO API in LPC8xx HAL
//!
//! This test suite communicates with hardware. See top-level README.md for
//! wiring instructions.

use std::thread::sleep;
use std::time;

use lpc845_messages::DynamicPin;
use lpc845_test_suite::{
    Result,
    TestStand,
};


// TODO: this test and `it_should_read_input_level()` only run green if they are executed
// *first* because the `dynamic_...` tests mess with red. => prevent double pin configs
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
fn dynamic_red_led_should_light_up_on_low() -> Result {
     // SETUP
    let mut test_stand = TestStand::new()?;
    test_stand.assistant.set_pin_direction_output( DynamicPin::Red )?;
    test_stand.assistant.set_output_pin_high( DynamicPin::Red )?;

    // RUN TEST
    sleep(time::Duration::from_secs(2));
    test_stand.assistant.set_output_pin_low( DynamicPin::Red )?;

    // 👀  manually assert that on-board led is red after 2 secs

    Ok(())
}

#[test]
fn dynamic_red_led_should_be_toggleable_by_pin_direction() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    // ensure pin is low (-> red led is on) when we start
    test_stand.assistant.set_output_pin_low( DynamicPin::Red )?;

    // RUN TEST
    for n in 0..10 {
        sleep(time::Duration::from_secs(2));

        // toggle back and forth between in/output
        if n % 2 == 0 {
            test_stand.assistant.set_pin_direction_input( DynamicPin::Red )?;
        } else {
            test_stand.assistant.set_pin_direction_output( DynamicPin::Red )?;
        }
    }

    // ASSERT POSTCONDITION
    // 👀  manually assert that led is toggling on/off 10 times every 2 secs

    Ok(())
}

#[test]
fn dynamic_red_led_should_be_toggleable_by_level() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    test_stand.assistant.set_pin_direction_output( DynamicPin::Red )?;

    // RUN TEST
    for n in 0..10 {
        sleep(time::Duration::from_secs(2));

        // toggle back and forth between high/low
        if n % 2 == 0 {
            test_stand.assistant.set_output_pin_low( DynamicPin::Red )?;
            // TODO: why does this check blue led Levels and not red?
            //assert!(test_stand.assistant.pin_is_low()?);
        } else {
            test_stand.assistant.set_output_pin_high( DynamicPin::Red )?;
            //assert!(test_stand.target.pin_is_high()?);
        }
    }

    // ASSERT POSTCONDITION
    // 👀  manually assert that led is toggling on/off 10 times every 2 secs

    Ok(())
}

#[test]
fn dynamic_red_led_should_be_readable() -> Result {
    // SETUP
    //let mut test_stand = TestStand::new()?;
    // configure target red as output
    //test_stand.assistant.set_pin_direction_output( DynamicPin::Red )?;
    // assistant red as input
    //test_stand.assistant.set_pin_direction_input()?;

    // RUN TEST
    // TODO
    // set target red high
    // read + assert assistant red high

    Ok(())
}

// TODO add tests checking that:
// - attempting to call Input methods on a dynamic Pin set to Output (and vice versa) causes understandable errors