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

const RED_LED : DynamicPin = DynamicPin::PIO1_2;
const GRN_LED : DynamicPin = DynamicPin::PIO1_0;


#[test]
fn it_should_set_pin_level() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    test_stand.assistant.set_pin_direction_input( GRN_LED )?;

    // TEST & ASSERT POSTCONDITION
    test_stand.target.set_pin_low()?;
    assert!(test_stand.assistant.input_pin_is_low( GRN_LED )?);

    // TEST & ASSERT POSTCONDITION
    test_stand.target.set_pin_high()?;
    assert!(test_stand.assistant.input_pin_is_high( GRN_LED )?);

    Ok(())
}

#[test]
fn it_should_read_input_level() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    test_stand.assistant.set_pin_direction_output( RED_LED )?;

    test_stand.assistant.set_output_pin_low( RED_LED )?;
    assert!(test_stand.target.pin_is_low()?);

    test_stand.assistant.set_output_pin_high( RED_LED )?;
    assert!(test_stand.target.pin_is_high()?);

    Ok(())
}

#[test]
fn dynamic_red_led_should_light_up_on_low() -> Result {
     // SETUP
    let mut test_stand = TestStand::new()?;
    test_stand.assistant.set_pin_direction_output( RED_LED )?;
    test_stand.assistant.set_output_pin_high( RED_LED )?;

    // RUN TEST
    sleep(time::Duration::from_secs(2));
    test_stand.assistant.set_output_pin_low( RED_LED )?;

    // 👀  manually assert that on-board led is red after 2 secs

    Ok(())
}

#[test]
fn dynamic_red_led_should_be_toggleable_by_pin_direction() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    // ensure pin is low (-> red led is on) when we start
    test_stand.assistant.set_output_pin_low( RED_LED )?;

    // RUN TEST
    for n in 0..5 {
        sleep(time::Duration::from_secs(2));

        // toggle back and forth between in/output
        if n % 2 == 0 {
            test_stand.assistant.set_pin_direction_input( RED_LED )?;
        } else {
            test_stand.assistant.set_pin_direction_output( RED_LED )?;
        }
    }

    // ASSERT POSTCONDITION
    // 👀  manually assert that led is toggling on/off 5 times every 2 secs

    Ok(())
}

#[test]
fn dynamic_red_led_should_be_toggleable_by_level() -> Result {
    // SETUP
    let mut test_stand = TestStand::new()?;
    test_stand.assistant.set_pin_direction_output( RED_LED )?;

    // RUN TEST
    for n in 0..5 {
        sleep(time::Duration::from_secs(2));

        // toggle back and forth between high/low
        if n % 2 == 0 {
            test_stand.assistant.set_output_pin_low( RED_LED )?;
            assert!(test_stand.target.pin_is_low()?);
        } else {
            test_stand.assistant.set_output_pin_high( RED_LED )?;
            assert!(test_stand.target.pin_is_high()?);
        }
    }

    // ASSERT POSTCONDITION
    // 👀  manually assert that led is toggling on/off 5 times every 2 secs

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