use std::sync::RwLock;
use std::time::{Duration, Instant};

use std::collections::HashMap;

use protocol::{
    AssistantToHost,
    HostToAssistant,
    InputPin,
    OutputPin,
    UsartMode,
    pin,
};

use crate::{
    conn::{
        Conn,
        ConnReceiveError,
        ConnSendError,
    },
    pin::{
        Pin,
        ReadLevelError,
    },
};

use lpc845_messages::{pin, AssistantToHost, DynamicPin, HostToAssistant, PinNumber, UsartMode};

// TODO find a place to share them with t-a and t-t?
/// some commonly used pin numbers
const RTS_PIN_NUMBER: PinNumber = 18;
const CTS_PIN_NUMBER: PinNumber = 19;

/// A wrapper around the test-assistant for easy pin configuration.
pub struct AssistantInterface<Assistant> {
    real_assistant: RwLock<Assistant>,
}

/// The connection to the test assistant
pub struct Assistant {
    /// connection between test-assistant and host
    conn: Conn,
    /// all of the assitant's GPIO pins, keyed by pin number (Arduino style)
    pins: HashMap<PinNumber, Pin<DynamicPin>>,
}

/// A dynamically reconfigurable Pin whose current direction is input.
pub struct InputPin<'assistant, Assistant> {
    /// Note that the pin numbers used here correspond to the LPC845 breakout board pinouts counted
    /// from top left counterclockwise to top right
    /// (see https://www.nxp.com/assets/images/en/block-diagrams/LPC845-BRK-BD2.png )
    pin_number: PinNumber,
    pin: Pin<DynamicPin>,
    /// The test-assistant instance that manages this pin (needed to access conn)
    assistant: &'assistant RwLock<Assistant>,
}

/// A dynamically reconfigurable Pin whose current direction is output.
pub struct OutputPin<'assistant, Assistant> {
    /// Note that the pin numbers used here correspond to the LPC845 breakout board pinouts counted
    /// from top left counterclockwise to top right
    /// (see https://www.nxp.com/assets/images/en/block-diagrams/LPC845-BRK-BD2.png )
    pin_number: PinNumber,
    pin: Pin<DynamicPin>,
    /// The test-assistant instance that manages this pin (needed to access conn)
    assistant: &'assistant RwLock<Assistant>,
}

// TODO add docs
impl AssistantInterface<Assistant> {
    pub fn new(conn: Conn, num_pins: u8) -> Self {
        let assistant = Assistant::new(conn, num_pins);

        AssistantInterface {
            real_assistant: RwLock::new(assistant),
        }
    }

    // TODO: create set_pin_5_high() / set_pin_5_low() wrappers somewhere

    /// Retrieve an InputPin instance that we can use to (re)configure the test-assistant's pin with
    /// number `pin_number` at test runtime.
    pub fn create_gpio_input_pin(
        &self,
        pin_number: u8,
    ) -> Result<InputPin<Assistant>, AssistantError> {
        // TODO untangle match statement below
        // TODO add coherence check to ensure we don't
        // - assign pins that can't be dynamic
        // - assign more dynamic pins than possible
        // -> and then return Err(AssistantPinOperationError::IllegalPinNumber(PinNumber))

        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock {
            // TODO make this a match instead?
            // pull pin out so it can't be reassigned
            match assistant.pins.remove(&pin_number) {
                Some(mut pin) => {
                    pin.set_direction_input::<HostToAssistant>(&mut assistant.conn)
                        .map_err(|err| AssistantPinOperationError::SetPinDirectionInputError(err))
                        .unwrap();

                    return Ok(InputPin {
                        assistant: &self.real_assistant,
                        pin_number: pin_number,
                        pin: pin,
                    });
                }
                None => return Err(AssistantPinOperationError::IllegalPinNumber(pin_number))),
            }
        }
        Err(AssistantPinOperationError::AssistantLockedError)
    }

    /// Retrieve an OutputPin instance that we can use to (re)configure the test-assistant's pin with
    /// number `pin_number` at test runtime.
    /// If this function returns without Error, the pin's voltage has been set to `level`.
    pub fn create_gpio_output_pin(
        &self,
        pin_number: u8,
        level: pin::Level,
    ) -> Result<OutputPin<Assistant>, AssistantError> {
        // TODO untangle match statement below
        // TODO add coherence check to ensure we don't
        // - assign more dynamic pins than possible
        // - assign pins that can't be dynamic
        // -> and then return Err(AssistantPinOperationError::IllegalPinNumber(PinNumber))

        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock {
            // TODO make this a match instead?
            // pull pin out so it can't be reassigned
            match assistant.pins.remove(&pin_number) {
                Some(mut pin) => {
                    pin.set_direction_output::<HostToAssistant>(level, &mut assistant.conn)
                        .map_err(|err| AssistantError::PinOperation(AssistantPinOperationError::SetPinDirectionInputError(err)))
                        .unwrap();

                    return Ok(OutputPin {
                        assistant: &self.real_assistant,
                        pin_number: pin_number,
                        pin: pin,
                    });
                }
                None => return Err(AssistantError::PinOperation(AssistantPinOperationError::IllegalPinNumber(pin_number))),
            }
        }
        Err(AssistantPinOperationError::AssistantLockedError)
    }

    pub fn measure_gpio_period(
        &mut self,
        samples: u32,
        timeout: Duration,
    ) -> Result<GpioPeriodMeasurement, AssistantError> {
        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock {
            return assistant.measure_gpio_period(samples, timeout);
        }

        // TODO more helpful error
        Err(AssistantError::PinReadError(ReadLevelError::Timeout))
    }

    /// Wait to receive the provided data via USART
    ///
    /// Returns the receive buffer, once the data was received. Returns an
    /// error, if it times out before that, or an I/O error occurs.
    pub fn receive_from_target_usart(
        &mut self,
        data: &[u8],
        timeout: Duration,
    ) -> Result<Vec<u8>, AssistantError> {
        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock {
            return assistant.receive_from_target_usart(data, timeout);
        }

        // TODO more helpful error
        Err(AssistantError::UsartWait(AssistantUsartWaitError::Timeout))
    }

    /// Wait to receive the provided data via USART in synchronous mode
    ///
    /// Returns the receive buffer, once the data was received. Returns an
    /// error, if it times out before that, or an I/O error occurs.
    pub fn receive_from_target_usart_sync(
        &mut self,
        data: &[u8],
        timeout: Duration,
    ) -> Result<Vec<u8>, AssistantError> {
        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock {
            return assistant.receive_from_target_usart(data, timeout);
        }

        // TODO more helpful error
        Err(AssistantError::UsartWait(AssistantUsartWaitError::Timeout))
    }

    /// Instruct assistant to send this message to the target via USART
    pub fn send_to_target_usart(&mut self, data: &[u8]) -> Result<(), AssistantError> {
        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock {
            return assistant.send_to_target_usart(data);
        }

        // TODO more helpful error
        Err(AssistantError::UsartSend(ConnSendError(
            host_lib::error::Error::AssistantLockedError,
        )))
    }

    /// Instruct assistant to send this message to the target's USART/DMA
    pub fn send_to_target_usart_dma(&mut self, data: &[u8]) -> Result<(), AssistantError> {
        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock {
            return assistant.send_to_target_usart_dma(data);
        }

        // TODO more helpful error
        Err(AssistantError::UsartSend(ConnSendError(
            host_lib::error::Error::AssistantLockedError,
        )))
    }

    /// Instruct assistant to send this message to the target's sync USART
    pub fn send_to_target_usart_sync(
        &mut self,
        data: &[u8],
    ) -> Result<(), AssistantError> {
        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock {
            return assistant.send_to_target_usart_sync(data);
        }

        // TODO more helpful error
        Err(AssistantError::UsartSend(ConnSendError(
            host_lib::error::Error::AssistantLockedError,
        )))
    }

    /// Instruct the assistant to disable CTS
    pub fn disable_cts(&mut self) -> Result<(), AssistantError> {
        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock {
            return assistant.disable_cts();
        }

        // TODO more helpful error
        Err(AssistantError::SetPinHigh(ConnSendError(
            host_lib::error::Error::AssistantLockedError,
        )))
    }

    /// Wait for RTS signal to be enabled
    pub fn wait_for_rts(&mut self) -> Result<bool, AssistantError> {
        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock {
            return assistant.wait_for_rts();
        }
        // TODO more helpful error
        Err(AssistantError::PinRead(ReadLevelError::Timeout))
    }

    /// Expect to hear nothing from the target within the given timeout period
    pub fn expect_nothing_from_target(
        &mut self,
        timeout: Duration,
    ) -> Result<(), AssistantError> {
        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock {
            return assistant.expect_nothing_from_target(timeout);
        }

        // TODO more helpful error
        Err(AssistantError::ExpectNothing(ConnReceiveError(
            host_lib::error::Error::AssistantLockedError,
        )))
    }

    /// Instruct the assistant to enable CTS
    pub fn enable_cts(&mut self) -> Result<(), AssistantError> {
        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock {
            return assistant.enable_cts();
        }

        // TODO more helpful error
        Err(AssistantError::SetPinLow(ConnSendError(
            host_lib::error::Error::AssistantLockedError,
        )))
    }
}

impl<'assistant> InputPin<'assistant, Assistant> {
    /// Convert this pin into an Output pin with initial voltage `level`.
    pub fn into_output_pin(
        mut self,
        level: pin::Level,
    ) -> Result<OutputPin<'assistant, Assistant>, AssistantError> {
        // note to self: loop until we get the lock?
        let lock = self.assistant.try_write();

        match lock {
            Ok(mut assistant) => {
                assistant
                    .pin_direction_to_output(&mut self.pin, level)
                    .unwrap();

                Ok(OutputPin {
                    pin_number: self.pin_number,
                    pin: self.pin,
                    assistant: self.assistant,
                })
            }
            Err(_) => Err(AssistantError::PinOperation(AssistantPinOperationError::AssistantLockedError)),
        }
    }

    /// Indicates whether this pin receives a **Low** signal from the test target
    pub fn is_low(&mut self) -> Result<bool, AssistantError> {
        // TODO handle lock getting failures better
        let lock = self.assistant.try_write();
        match lock {
            Ok(mut assistant) => {
                let pin_state = self
                    .pin
                    .read_level::<HostToAssistant, AssistantToHost>(
                        Duration::from_millis(10),
                        &mut assistant.conn,
                    )
                    .map_err(|err| AssistantError::ReadPin(err))?;

                Ok(pin_state.0 == pin::Level::Low)
            }
            Err(_) => Err(AssistantError::PinOperation(AssistantPinOperationError::AssistantLockedError)),
        }
    }

    /// Indicates whether this pin receives a **High** signal from the test target
    pub fn is_high(&mut self) -> Result<bool, AssistantError> {
        match self.is_low() {
            Ok(is_low) => Ok(!is_low),
            Err(err) => Err(err),
        }
    }
}

impl<'assistant> OutputPin<'assistant, Assistant> {
    /// Convert this pin into an Input pin
    pub fn into_input_pin(
        mut self,
    ) -> Result<InputPin<'assistant, Assistant>, AssistantError> {
        // note to self: loop until we get the lock?
        let lock = self.assistant.try_write();

        match lock {
            Ok(mut assistant) => {
                assistant.pin_direction_to_input(&mut self.pin).unwrap();

                Ok(InputPin {
                    pin_number: self.pin_number,
                    pin: self.pin,
                    assistant: self.assistant,
                })
            }
            Err(_) => Err(AssistantError::PinOperation(AssistantPinOperationError::AssistantLockedError)),
        }
    }

    /// Set this pin's level to Low.
    pub fn set_low(&mut self) -> Result<(), AssistantError> {
        // TODO handle lock getting failures better
        let lock = self.assistant.try_write();
        match lock {
            Ok(mut assistant) => self
                .pin
                .set_level::<HostToAssistant>(pin::Level::Low, &mut assistant.conn)
                .map_err(|err| AssistantError::SetPinLow(err)),
            Err(_) => Err(AssistantError::PinOperation(AssistantPinOperationError::AssistantLockedError)),
        }
    }

    /// Set this pin's level to High.
    pub fn set_high(&mut self) -> Result<(), AssistantError> {
        // TODO handle lock getting failures better
        let lock = self.assistant.try_write();
        match lock {
            Ok(mut assistant) => self
                .pin
                .set_level::<HostToAssistant>(pin::Level::High, &mut assistant.conn)
                .map_err(|err| AssistantError::SetPinLow(err)),
            Err(_) => Err(AssistantError::PinOperation(AssistantPinOperationError::AssistantLockedError)),
        }
    }
}

impl Assistant {
    pub(crate) fn new(conn: Conn, num_pins: u8) -> Self {
        let mut s = Self {
            conn,
            pins: HashMap::new(),
        };

        // init all pins
        for pin_number in 1..=num_pins {
            s.pins
                .insert(pin_number, Pin::new(DynamicPin::GPIO(pin_number)));
        }

        // make sure rts and cts have the right direction
        s.set_pin_direction_input(DynamicPin::GPIO(RTS_PIN_NUMBER))
            .unwrap();
        // TODO double check with old code if this is the correct default direction?
        s.set_pin_direction_output(DynamicPin::GPIO(CTS_PIN_NUMBER), pin::Level::Low)
            .unwrap();

        return s;
    }

    // internal helper
    fn pin_direction_to_output(
        &mut self,
        pin: &mut Pin<DynamicPin>,
        level: pin::Level,
    ) -> Result<(), AssistantError> {
        pin.set_direction_output::<HostToAssistant>(level, &mut self.conn)
            .map_err(|err| AssistantError::PinOperation(AssistantPinOperationError::SetPinDirectionInputError(err)))
    }

    // internal helper
    fn pin_direction_to_input(
        &mut self,
        pin: &mut Pin<DynamicPin>,
    ) -> Result<(), AssistantError> {
        pin.set_direction_input::<HostToAssistant>(&mut self.conn)
            .map_err(|err| AssistantError::PinOperation(AssistantPinOperationError::SetPinDirectionInputError(err)))
    }

    /// Make the test-assistant's pin with number `pin` an Input pin.
    /// Note: this is a legacy function and should probably be refactored out (TODO)
    fn set_pin_direction_input(
        &mut self,
        pin: DynamicPin,
    ) -> Result<(), AssistantError> {
        match pin {
            DynamicPin::GPIO(pin_number) => self
                .pins
                .get_mut(&pin_number)
                .unwrap()
                .set_direction_input::<HostToAssistant>(&mut self.conn)
                .map_err(|err| AssistantError::PinOperation(AssistantPinOperationError::SetPinDirectionInputError(err))),
            _ => todo!(),
        }
    }

    /// Make the test-assistant's `pin` an Output pin.
    pub fn set_pin_direction_output(
        &mut self,
        pin: DynamicPin,
        level: pin::Level,
    ) -> Result<(), AssistantError> {
        match pin {
            DynamicPin::GPIO(pin_number) => self
                .pins
                .get_mut(&pin_number)
                .unwrap()
                .set_direction_output::<HostToAssistant>(level, &mut self.conn)
                .map_err(|err| AssistantError::PinOperation(AssistantPinOperationError::SetPinDirectionOutput(err))),
            _ => todo!(),
        }
    }

    /// Instruct the assistant to disable CTS
    pub fn disable_cts(&mut self) -> Result<(), AssistantError> {
        self.pins
            .get_mut(&CTS_PIN_NUMBER)
            .unwrap()
            .set_level::<HostToAssistant>(pin::Level::High, &mut self.conn)
            .map_err(|err| AssistantError::SetPinHigh(err))
    }

    /// Instruct the assistant to enable CTS
    pub fn enable_cts(&mut self) -> Result<(), AssistantError> {
        self.pins
            .get_mut(&CTS_PIN_NUMBER)
            .unwrap()
            .set_level::<HostToAssistant>(pin::Level::Low, &mut self.conn)
            .map_err(|err| AssistantError::SetPinLow(err))
    }

    /// Wait for RTS signal to be enabled
    pub fn wait_for_rts(&mut self) -> Result<bool, AssistantError> {
        let pin_state = self
            .pins
            .get_mut(&RTS_PIN_NUMBER)
            .unwrap()
            .read_level::<HostToAssistant, AssistantToHost>(
                Duration::from_millis(10),
                &mut self.conn,
            )?;
        Ok(pin_state.0 == pin::Level::Low)
    }

    pub fn receive_from_target_usart_inner(
        &mut self,
        data: &[u8],
        timeout: Duration,
        expected_mode: UsartMode,
    ) -> Result<Vec<u8>, AssistantUsartWaitError> {
        let mut buf = Vec::new();
        let start = Instant::now();

        loop {
            if buf.windows(data.len()).any(|window| window == data) {
                return Ok(buf);
            }
            if start.elapsed() > timeout {
                return Err(AssistantUsartWaitError::Timeout);
            }

            let mut tmp = Vec::new();
            let message = self
                .conn
                .receive::<AssistantToHost>(timeout, &mut tmp)
                .map_err(|err| AssistantUsartWaitError::Receive(err))?;

            match message {
                AssistantToHost::UsartReceive { mode, data } if mode == expected_mode => {
                    buf.extend(data)
                }
                _ => {
                    return Err(AssistantUsartWaitError::UnexpectedMessage(format!(
                        "{:?}",
                        message
                    )));
                }
            }
        }
    }

    /// Wait to receive the provided data via USART
    ///
    /// Returns the receive buffer, once the data was received. Returns an
    /// error, if it times out before that, or an I/O error occurs.
    pub fn receive_from_target_usart(
        &mut self,
        data: &[u8],
        timeout: Duration,
    ) -> Result<Vec<u8>, AssistantUsartWaitError> {
        self.receive_from_target_usart_inner(data, timeout, UsartMode::Regular)
    }

    /// Wait to receive the provided data via USART in synchronous mode
    ///
    /// Returns the receive buffer, once the data was received. Returns an
    /// error, if it times out before that, or an I/O error occurs.
    pub fn receive_from_target_usart_sync(
        &mut self,
        data: &[u8],
        timeout: Duration,
    ) -> Result<Vec<u8>, AssistantUsartWaitError> {
        self.receive_from_target_usart_inner(data, timeout, UsartMode::Sync)
    }

    /// Measures the period of changes triggered by the target Timer interrupt signal
    /// on pin number 30 / PIO1_1
    ///
    /// Waits for changes in the GPIO signal until the given number of samples
    /// has been measured. Returns the minimum and maximum period measured, in
    /// milliseconds.
    ///
    /// # Panics
    ///
    /// `samples` must be at least `1`. This method will panic, if this is not
    /// the case.
    pub fn measure_gpio_period(
        &mut self,
        samples: u32,
        timeout: Duration,
    ) -> Result<GpioPeriodMeasurement, AssistantPinReadError> {
        assert!(samples > 0);

        let target_timer_pin_number = 30;
        let mut measurement: Option<GpioPeriodMeasurement> = None;

        let (mut state, _) = self
            .pins
            .get_mut(&target_timer_pin_number)
            .unwrap()
            .read_level::<HostToAssistant, AssistantToHost>(timeout, &mut self.conn)?;

        for _ in 0..samples {
            let (new_state, period_ms) =
                self.pins
                    .get_mut(&target_timer_pin_number)
                    .unwrap()
                    .read_level::<HostToAssistant, AssistantToHost>(timeout, &mut self.conn)?;
            print!("{:?}, {:?}\n", new_state, period_ms);

            if new_state == state {
                continue;
            }

            state = new_state;

            let period = match period_ms {
                Some(period_ms) => Duration::from_millis(period_ms as u64),
                None => continue,
            };

            match &mut measurement {
                Some(measurement) => {
                    measurement.min = Ord::min(measurement.min, period);
                    measurement.max = Ord::max(measurement.max, period);
                }
                None => {
                    measurement = Some(GpioPeriodMeasurement {
                        min: period,
                        max: period,
                    })
                }
            }
        }

        // Due to the assertion above, we know that samples is at least `1` and
        // therefore, that the loop ran at least once. `measurement` must be
        // `Some`.
        Ok(measurement.unwrap())
    }

    /// Expect to hear nothing from the target within the given timeout period
    pub fn expect_nothing_from_target(
        &mut self,
        timeout: Duration,
    ) -> Result<(), AssistantExpectNothingError> {
        loop {
            let mut tmp = Vec::new();
            let message = self.conn.receive::<AssistantToHost>(timeout, &mut tmp);

            match message {
                Ok(message) => {
                    return Err(AssistantExpectNothingError::UnexpectedMessage(format!(
                        "{:?}",
                        message
                    )));
                }
                Err(err) if err.is_timeout() => {
                    break;
                }
                Err(err) => {
                    return Err(AssistantExpectNothingError::Receive(err));
                }
            }
        }

        Ok(())
    }
}

#[derive(Debug)]
pub struct GpioPeriodMeasurement {
    pub min: Duration,
    pub max: Duration,
}

/// All the errors that can be returned by this API
#[derive(Debug)]
pub enum AssistantError {
    ExpectNothing(AssistantExpectNothingError),
    PinRead(ReadLevelError),
    SetPinHigh(ConnSendError),
    SetPinLow(ConnSendError),
    UsartSend(ConnSendError),
    UsartWait(AssistantUsartWaitError),
    PinOperation(AssistantPinOperationError)
}

impl From<ReadLevelError> for AssistantError {
    fn from(err: ReadLevelError) -> Self {
        Self::PinRead(err)
    }
}

impl From<AssistantUsartWaitError> for AssistantError {
    fn from(err: AssistantUsartWaitError) -> Self {
        Self::UsartWait(err)
    }
}


#[derive(Debug)]
pub enum AssistantUsartWaitError {
    Receive(ConnReceiveError),
    Timeout,
    UnexpectedMessage(String),
}

#[derive(Debug)]
pub enum AssistantExpectNothingError {
    Receive(ConnReceiveError),
    UnexpectedMessage(String),
}

#[derive(Debug)]
pub enum AssistantPinOperationError {
    /// This pin cannot be configured, for example because it is reserved for internal use
    /// Or has already been created earlier
    IllegalPinNumber(PinNumber),
    SetPinDirectionInputError(ConnSendError), // TODO rm Error from name
    SetPinDirectionOutput(ConnSendError),
    ReadPinError(ReadLevelError), // TODO rm Error from name
    AssistantLockedError, // TOOD move out of this enum?
}
