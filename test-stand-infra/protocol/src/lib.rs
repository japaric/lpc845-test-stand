#![no_std]


pub mod pin;


use core::convert::TryFrom;

use serde::{
    Deserialize,
    Serialize,
};


/// A message from the test suite on the host to the test assistant
#[derive(Debug, Deserialize, Serialize)]
pub enum HostToAssistant<'r> {
    /// Instruct the assistant to send data to the target via USART
    SendUsart {
        mode: UsartMode,
        data: &'r [u8],
    },

    /// Instruct the assistant to change level of the target's input pin
    SetPin(pin::SetLevel<OutputPin>),

    /// Ask the assistant for the current level of a pin
    ReadPin(pin::ReadLevel<InputPin>),

    /// Instruct the assistant to change level of the target's dynamic pin if direction is input
    SetDynamicPin(pin::SetLevel<DynamicPin>),

    /// Ask the assistant for the current level of a dynamic pin if direction is output
    ReadDynamicPin(pin::ReadLevel<DynamicPin>),

    /// Instruct the assistant to change the direction of `DynamicPin`
    SetDirection(pin::SetDirection<DynamicPin>),
}

impl From<pin::SetLevel<OutputPin>> for HostToAssistant<'_> {
    fn from(set_level: pin::SetLevel<OutputPin>) -> Self {
        Self::SetPin(set_level)
    }
}

impl From<pin::ReadLevel<InputPin>> for HostToAssistant<'_> {
    fn from(read_level: pin::ReadLevel<InputPin>) -> Self {
        Self::ReadPin(read_level)
    }
}

impl From<pin::SetDirection<DynamicPin>> for HostToAssistant<'_> {
    fn from(set_direction: pin::SetDirection<DynamicPin>) -> Self {
        Self::SetDirection(set_direction)
    }
}

impl From<pin::SetLevel<DynamicPin>> for HostToAssistant<'_> {
    fn from(set_level: pin::SetLevel<DynamicPin>) -> Self {
        Self::SetDynamicPin(set_level)
    }
}

impl From<pin::ReadLevel<DynamicPin>> for HostToAssistant<'_> {
    fn from(read_level: pin::ReadLevel<DynamicPin>) -> Self {
        Self::ReadDynamicPin(read_level)
    }
}

/// A message from the test assistant to the test suite on the host
#[derive(Debug, Deserialize, Serialize)]
pub enum AssistantToHost<'r> {
    /// Notify the host that data has been received from the target via USART
    UsartReceive {
        mode: UsartMode,
        data: &'r [u8],
    },

    /// Notify the host that the level of a pin has changed
    ReadPinResult(Option<pin::ReadLevelResult<InputPin>>),

    /// Notify the host that the level of a dynamic pin has changed
    ReadPinResultDynamic(Option<pin::ReadLevelResult<DynamicPin>>),
}

impl<'r> TryFrom<AssistantToHost<'r>> for pin::ReadLevelResult<InputPin> {
    type Error = AssistantToHost<'r>;

    fn try_from(value: AssistantToHost<'r>) -> Result<Self, Self::Error> {
        match value {
            AssistantToHost::ReadPinResult(Some(result)) => {
                Ok(result)
            }
            _ => {
                Err(value)
            }
        }
    }
}

impl<'r> TryFrom<AssistantToHost<'r>> for pin::ReadLevelResult<DynamicPin> {
    type Error = AssistantToHost<'r>;

    fn try_from(value: AssistantToHost<'r>) -> Result<Self, Self::Error> {
        match value {
            AssistantToHost::ReadPinResultDynamic(Some(result)) => {
                Ok(result)
            }
            _ => {
                Err(value)
            }
        }
    }
}

pub type PinNumber = u8;

/// Specifies which mode a USART transmission uses
#[derive(Clone, Copy, Debug, Deserialize, Serialize, Eq, PartialEq)]
pub enum UsartMode {
    Regular,
    Dma,
    FlowControl,
    Sync,
}


/// Represents one of the pins that the assistant is monitoring
#[derive(Clone, Copy, Debug, Deserialize, Serialize, Eq, PartialEq)]
pub enum InputPin {
    Blue  = 0,
    Green = 1,
    Rts   = 2,
    Pwm   = 3,
}

/// Represents one of the pins that the assistant can set
#[derive(Clone, Copy, Debug, Deserialize, Serialize, Eq, PartialEq)]
pub enum OutputPin {
    Pin5,
    Cts,
    Red,
}

/// Represents one of the pins that the assistant can (re)configure at runtime
/// Note that the pin numbers used here correspond to the LPC845 breakout board pinouts counted
/// from top left counterclockwise to top right
/// (see https://www.nxp.com/assets/images/en/block-diagrams/LPC845-BRK-BD2.png )
#[derive(Clone, Copy, Debug, Deserialize, Serialize, Eq, PartialEq)]
pub enum DynamicPin {
    // TODO:
    // - maybe prevent illegal pin numbers (e.g. GND/20 should not be used for GPIO)
    GPIO(PinNumber),
    UART, // TODO this is just a dummy, adjust to add info needed for uart
}

impl DynamicPin {
    pub fn get_pin_number(&self) -> Option<PinNumber> {
        match self {
            DynamicPin::GPIO(number) => { Some(*number) }
            DynamicPin::UART => { None }
        }
    }
}