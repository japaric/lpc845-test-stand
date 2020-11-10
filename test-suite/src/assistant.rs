use std::sync::RwLock;
use std::time::{Duration, Instant};

use std::collections::HashMap;

use host_lib::{
    conn::{Conn, ConnReceiveError, ConnSendError},
    pin::{Pin, ReadLevelError},
};
use lpc845_messages::{
    pin, AssistantToHost, DynamicPin, HostToAssistant, PinNumber, UsartMode, VoltageLevel,
};

// TODO find a place to share them with t-a and t-t?
/// some commonly used pin numbers
const RTS_PIN_NUMBER: PinNumber = 18;
const CTS_PIN_NUMBER: PinNumber = 19;

/// A wrapper around the test-assistant for easy pin configuration.
pub struct AssistantInterface<Assistant> {
    real_assistant: RwLock<Assistant>,
}

/// The connection to the test assistant
// TODO: doesn't have to be pub after refactoring #3?
pub struct Assistant {
    conn: Conn,
    /// all of the assitant's GPIO pins, keyed by pin number (Arduino style)
    pins: HashMap<PinNumber, Pin<DynamicPin>>,
}

pub struct InputPin<'assistant, Assistant> {
    /// Note that the pin numbers used here correspond to the LPC845 breakout board pinouts counted
    /// from top left counterclockwise to top right
    /// (see https://www.nxp.com/assets/images/en/block-diagrams/LPC845-BRK-BD2.png )
    pin_number: PinNumber,
    pin: Pin<DynamicPin>,
    assistant: &'assistant RwLock<Assistant>, // needed to access conn
}

pub struct OutputPin<'assistant, Assistant> {
    /// Note that the pin numbers used here correspond to the LPC845 breakout board pinouts counted
    /// from top left counterclockwise to top right
    /// (see https://www.nxp.com/assets/images/en/block-diagrams/LPC845-BRK-BD2.png )
    pin_number: PinNumber,
    pin: Pin<DynamicPin>,
    assistant: &'assistant RwLock<Assistant>, // needed to access conn
}

impl AssistantInterface<Assistant> {
    pub fn new(conn: Conn, num_pins: u8) -> Self {
        let assistant = Assistant::new(conn, num_pins);

        AssistantInterface {
            real_assistant: RwLock::new(assistant),
        }
    }

    pub fn create_gpio_input_pin(
        &self,
        pin_number: u8,
    ) -> Result<InputPin<Assistant>, AssistantPinOperationError> {
        // TODO untangle match statement below
        // TODO add coherence check to ensure we don't
        // - assign pins that can't be dynamic
        // - assign more dynamic pins than possible
        // -> and then return Err(AssistantPinOperationError::IllegalPinNumber(PinNumber))

        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock { // TODO make this a match instead?
            // pull pin out so it can't be reassigned
            match assistant.pins.remove(&pin_number) {
                Some(mut pin) => {
                    pin.set_direction::<HostToAssistant>(
                        pin::Direction::Input,
                        &mut assistant.conn,
                    )
                    .map_err(|err| AssistantPinOperationError::SetPinDirectionInputError(err)).unwrap();

                    return Ok(InputPin {
                        assistant: &self.real_assistant,
                        pin_number: pin_number,
                        pin: pin,
                    });
                }
                None => return Err(AssistantPinOperationError::IllegalPinNumber(pin_number)),
            }
        }
        Err(AssistantPinOperationError::AssistantLockedError)
    }

    pub fn create_gpio_output_pin(
        &self,
        pin_number: u8,
    ) -> Result<OutputPin<Assistant>, AssistantPinOperationError> {
        // TODO untangle match statement below
        // TODO add coherence check to ensure we don't
        // - assign more dynamic pins than possible
        // - assign pins that can't be dynamic
        // -> and then return Err(AssistantPinOperationError::IllegalPinNumber(PinNumber))

        let lock = self.real_assistant.try_write();
        // note to self: loop until we get the lock?
        if let Ok(mut assistant) = lock { // TODO make this a match instead?
            // pull pin out so it can't be reassigned
            match assistant.pins.remove(&pin_number) {
                Some(mut pin) => {
                    pin.set_direction::<HostToAssistant>(
                        pin::Direction::Output,
                        &mut assistant.conn,
                    )
                    .map_err(|err| AssistantPinOperationError::SetPinDirectionInputError(err)).unwrap();

                    return Ok(OutputPin {
                        assistant: &self.real_assistant,
                        pin_number: pin_number,
                        pin: pin,
                    });
                }
                None => return Err(AssistantPinOperationError::IllegalPinNumber(pin_number)),
            }
        }
        Err(AssistantPinOperationError::AssistantLockedError)
    }
}

// TODO rm 2s
impl<'assistant> InputPin<'assistant, Assistant> {

    /// Convert this pin into an Output pin with initial voltage `voltage_level`
    /// NOTE: `voltage_level` is not passed to test-assistant yet; pin is always `Low`
    pub fn to_output_pin(
        mut self,
        _voltage_level: VoltageLevel,
    ) -> Result<OutputPin<'assistant, Assistant>, AssistantPinOperationError> {

        // note to self: loop until we get the lock?
        let lock = self.assistant.try_write();

        match lock {
            Ok(mut assistant) => {
                assistant.pin_direction_to_output(&mut self.pin).unwrap();
                // TODO pass voltage_level on to t-a

                Ok(OutputPin {
                    pin_number: self.pin_number,
                    pin: self.pin,
                    assistant: self.assistant,
                })
            }
            Err(_) => Err(AssistantPinOperationError::AssistantLockedError)
        }
    }

    /// Indicates whether this pin receives a **Low** signal from the test target
    pub fn is_low(&mut self) -> Result< bool, AssistantPinOperationError> {
        // TODO handle lock getting failures better
        let lock = self.assistant.try_write();
        match lock {
            Ok(mut assistant) => {
                let pin_state = self.pin
                    .read_level::<HostToAssistant, AssistantToHost>(
                        Duration::from_millis(10),
                        &mut assistant.conn)
                    .map_err(|err| AssistantPinOperationError::ReadPinError(err))?;

                    Ok(pin_state.0 == pin::Level::Low)
            }
            Err(_) => Err(AssistantPinOperationError::AssistantLockedError)
        }
    }

    /// Indicates whether this pin receives a **High** signal from the test target
    pub fn is_high(&mut self) -> Result<bool, AssistantPinOperationError> {
        match self.is_low() {
            Ok(is_low) => { Ok(!is_low) }
            Err(err) => {Err(err)}
        }
    }
}

impl<'assistant> OutputPin<'assistant, Assistant> {

    /// Convert this pin into an Input pin
    pub fn to_input_pin(
        mut self,
    ) -> Result<InputPin<'assistant, Assistant>, AssistantPinOperationError> {

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
            Err(_) => Err(AssistantPinOperationError::AssistantLockedError)
        }
    }

    /// Set this pin's level to Low.
    pub fn set_low(&mut self) -> Result<(), AssistantPinOperationError> {
         // TODO handle lock getting failures better
        let lock = self.assistant.try_write();
        match lock {
            Ok(mut assistant) => {
                self.pin
                    .set_level::<HostToAssistant>(pin::Level::Low, &mut assistant.conn)
                    .map_err(|err| AssistantPinOperationError::SetPinLowError(err))
            }
            Err(_) => Err(AssistantPinOperationError::AssistantLockedError)
        }
    }

    /// Set this pin's level to High.
    pub fn set_high(&mut self) -> Result<(), AssistantPinOperationError> {
        // TODO handle lock getting failures better
        let lock = self.assistant.try_write();
        match lock {
            Ok(mut assistant) => {
                self.pin
                    .set_level::<HostToAssistant>(pin::Level::High, &mut assistant.conn)
                    .map_err(|err| AssistantPinOperationError::SetPinLowError(err))
            }
            Err(_) => Err(AssistantPinOperationError::AssistantLockedError)
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
        s.set_pin_direction_output(DynamicPin::GPIO(CTS_PIN_NUMBER))
            .unwrap();

        return s;
    }

    // internal helper
    fn pin_direction_to_output(
        &mut self,
        pin: &mut Pin<DynamicPin>,
    ) -> Result<(), AssistantPinOperationError> {
        pin.set_direction::<HostToAssistant>(pin::Direction::Output, &mut self.conn)
            .map_err(|err| AssistantPinOperationError::SetPinDirectionInputError(err))
    }

    // internal helper
    fn pin_direction_to_input(
        &mut self,
        pin: &mut Pin<DynamicPin>,
    ) -> Result<(), AssistantPinOperationError> {
        pin.set_direction::<HostToAssistant>(pin::Direction::Input, &mut self.conn)
            .map_err(|err| AssistantPinOperationError::SetPinDirectionInputError(err))
    }

    /// Make the test-assistant's pin with number `pin` an Input pin.
    /// Note: this is a legacy function and should probably be refactored out (TODO)
    fn set_pin_direction_input(
        &mut self,
        pin: DynamicPin,
    ) -> Result<(), AssistantSetPinDirectionInputError> {
        match pin {
            DynamicPin::GPIO(pin_number) => self
                .pins
                .get_mut(&pin_number)
                .unwrap()
                .set_direction::<HostToAssistant>(pin::Direction::Input, &mut self.conn)
                .map_err(|err| AssistantSetPinDirectionInputError(err)),
            _ => todo!(),
        }
    }

    /// Make the test-assistant's `pin` an Output pin.
    pub fn set_pin_direction_output(
        &mut self,
        pin: DynamicPin,
    ) -> Result<(), AssistantSetPinDirectionOutputError> {
        match pin {
            DynamicPin::GPIO(pin_number) => self
                .pins
                .get_mut(&pin_number)
                .unwrap()
                .set_direction::<HostToAssistant>(pin::Direction::Output, &mut self.conn)
                .map_err(|err| AssistantSetPinDirectionOutputError(err)),
            _ => todo!(),
        }
    }

    /// Set the test-assistant's `pin` level to Low.
    /// Note that the direction of `pin` must be set to Output first!
    /// Use `set_pin_direction_output()` for this.
    pub fn set_output_pin_low(&mut self, pin: DynamicPin) -> Result<(), AssistantSetPinLowError> {
        match pin {
            DynamicPin::GPIO(pin_number) => self
                .pins
                .get_mut(&pin_number)
                .unwrap()
                .set_level::<HostToAssistant>(pin::Level::Low, &mut self.conn)
                .map_err(|err| AssistantSetPinLowError(err)),
            _ => todo!(),
        }
    }

    /// Instruct the assistant to disable CTS
    pub fn disable_cts(&mut self) -> Result<(), AssistantSetPinHighError> {
        self.pins
            .get_mut(&CTS_PIN_NUMBER)
            .unwrap()
            .set_level::<HostToAssistant>(pin::Level::High, &mut self.conn)
            .map_err(|err| AssistantSetPinHighError(err))
    }

    /// Instruct the assistant to enable CTS
    pub fn enable_cts(&mut self) -> Result<(), AssistantSetPinLowError> {
        self.pins
            .get_mut(&CTS_PIN_NUMBER)
            .unwrap()
            .set_level::<HostToAssistant>(pin::Level::Low, &mut self.conn)
            .map_err(|err| AssistantSetPinLowError(err))
    }

    /// Wait for RTS signal to be enabled
    pub fn wait_for_rts(&mut self) -> Result<bool, AssistantPinReadError> {
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

    /// Instruct assistant to send this message to the target via USART
    pub fn send_to_target_usart(&mut self, data: &[u8]) -> Result<(), AssistantUsartSendError> {
        self.conn
            .send(&HostToAssistant::SendUsart {
                mode: UsartMode::Regular,
                data,
            })
            .map_err(|err| AssistantUsartSendError(err))
    }

    /// Instruct assistant to send this message to the target's USART/DMA
    pub fn send_to_target_usart_dma(&mut self, data: &[u8]) -> Result<(), AssistantUsartSendError> {
        self.conn
            .send(&HostToAssistant::SendUsart {
                mode: UsartMode::Dma,
                data,
            })
            .map_err(|err| AssistantUsartSendError(err))
    }

    /// Instruct assistant to send this message to the target's sync USART
    pub fn send_to_target_usart_sync(
        &mut self,
        data: &[u8],
    ) -> Result<(), AssistantUsartSendError> {
        self.conn
            .send(&HostToAssistant::SendUsart {
                mode: UsartMode::Sync,
                data,
            })
            .map_err(|err| AssistantUsartSendError(err))
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

#[derive(Debug)]
pub struct AssistantSetPinHighError(ConnSendError);

#[derive(Debug)]
pub struct AssistantSetPinLowError(ConnSendError);

#[derive(Debug)]
pub struct AssistantSetPinDirectionInputError(ConnSendError);

#[derive(Debug)]
pub struct AssistantSetPinDirectionOutputError(ConnSendError);

#[derive(Debug)]
pub struct AssistantPinReadError(ReadLevelError);

impl From<ReadLevelError> for AssistantPinReadError {
    fn from(err: ReadLevelError) -> Self {
        Self(err)
    }
}

#[derive(Debug)]
pub struct AssistantUsartSendError(ConnSendError);

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
    SetPinDirectionInputError(ConnSendError),
    SetPinLowError(ConnSendError),
    ReadPinError(ReadLevelError),
    AssistantLockedError,
}
