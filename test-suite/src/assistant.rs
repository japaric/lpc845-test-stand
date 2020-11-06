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
pub const RTS_PIN_NUMBER: PinNumber = 18;
pub const CTS_PIN_NUMBER: PinNumber = 19;

/// The connection to the test assistant
pub struct Assistant {
    conn: Conn,
    /// all of the assitant's GPIO pins, keyed by pin number (Arduino style)
    pins: HashMap<PinNumber, Pin<DynamicPin>>,
}

pub struct InputPin {
    /// Note that the pin numbers used here correspond to the LPC845 breakout board pinouts counted
    /// from top left counterclockwise to top right
    /// (see https://www.nxp.com/assets/images/en/block-diagrams/LPC845-BRK-BD2.png )
    pin_number: PinNumber,
    pin: Pin<DynamicPin>,
}

pub struct OutputPin {
    /// Note that the pin numbers used here correspond to the LPC845 breakout board pinouts counted
    /// from top left counterclockwise to top right
    /// (see https://www.nxp.com/assets/images/en/block-diagrams/LPC845-BRK-BD2.png )
    pin_number: PinNumber,
    pin: Pin<DynamicPin>,
}

// TODO remember what james said about not having to reimplement everything three times
impl InputPin {
    pub fn is_low(&self) {
        todo!()
    }

    /// Convert this pin into an Output pin with initial voltage `voltage_level`
    pub fn to_output_pin(
        self,
        _voltage_level: VoltageLevel,
    ) -> Result<OutputPin, AssistantPinOperationError> {
        // TODO actually set pin direction :D

        // TODO pass voltage_level on to t-a

        Ok(OutputPin {
            pin_number: self.pin_number,
            pin: self.pin,
        })
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

    /// TODO add docs
    pub fn create_gpio_input_pin(
        &mut self,
        pin_number: PinNumber,
    ) -> Result<InputPin, AssistantPinOperationError> {
        // TODO add coherence check to ensure we don't
        // - assign more dynamic pins than possible
        // -> and then return Err(AssistantPinOperationError::IllegalPinNumber(PinNumber))

        match self.pins.remove(&pin_number) {
            Some(mut pin) => {
                pin.set_direction::<HostToAssistant>(pin::Direction::Input, &mut self.conn)
                    .map_err(|err| {
                        AssistantPinOperationError::SetPinDirectionInputError(pin_number, err)
                    })?;

                Ok(InputPin {
                    pin_number: pin_number,
                    pin: pin,
                })
            }
            None => Err(AssistantPinOperationError::IllegalPinNumber(pin_number)),
        }
    }

    /// Make the test-assistant's pin with number `pin` an Input pin.
    pub fn set_pin_direction_input(
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

    /// Set the test-assistant's `pin` level to High.
    /// Note that the direction of `pin` must be set to Output first!
    /// Use `set_pin_direction_output()` for this.
    pub fn set_output_pin_high(&mut self, pin: DynamicPin) -> Result<(), AssistantSetPinHighError> {
        // TODO assert that pin is in output direction (note: we don't store pin state for this here)
        match pin {
            DynamicPin::GPIO(pin_number) => self
                .pins
                .get_mut(&pin_number)
                .unwrap()
                .set_level::<HostToAssistant>(pin::Level::High, &mut self.conn)
                .map_err(|err| AssistantSetPinHighError(err)),
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

    /// Indicates whether the GPIO pin `pin` receives a **High** signal from the test target
    /// Note that the direction of `pin` must be set to Input first!
    /// Use `set_pin_direction_input()` for this.
    ///
    /// Uses `pin_state` internally.
    pub fn input_pin_is_high(&mut self, pin: DynamicPin) -> Result<bool, AssistantPinReadError> {
        let pin_state: (pin::Level, Option<u32>);

        match pin {
            DynamicPin::GPIO(pin_number) => {
                pin_state = self
                    .pins
                    .get_mut(&pin_number)
                    .unwrap()
                    .read_level::<HostToAssistant, AssistantToHost>(
                        Duration::from_millis(10),
                        &mut self.conn,
                    )?;
            }
            _ => todo!(),
        }
        Ok(pin_state.0 == pin::Level::High)
    }

    /// Indicates whether the GPIO pin `pin` receives a **Low** signal from the test target
    /// Note that the direction of `pin` must be set to Input first!
    /// Use `set_pin_direction_input()` for this.
    ///
    /// Uses `pin_state` internally.
    pub fn input_pin_is_low(&mut self, pin: DynamicPin) -> Result<bool, AssistantPinReadError> {
        let pin_state: (pin::Level, Option<u32>);
        match pin {
            DynamicPin::GPIO(pin_number) => {
                pin_state = self
                    .pins
                    .get_mut(&pin_number)
                    .unwrap()
                    .read_level::<HostToAssistant, AssistantToHost>(
                        Duration::from_millis(10),
                        &mut self.conn,
                    )?;
            }
            _ => todo!(),
        }
        Ok(pin_state.0 == pin::Level::Low)
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
    SetPinDirectionInputError(PinNumber, ConnSendError),
}
