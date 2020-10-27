use heapless::{
    consts::U4,
    spsc::{Consumer, Producer, Queue},
};
use lpc8xx_hal::{gpio};

/// Represents a pin interrupt
pub struct TimerInterrupt<E> {
    queue: Queue<E, QueueCap>,
}

impl<E> TimerInterrupt<E> {
    /// Create a new instance of `PinInterrupt`
    ///
    /// Can be called in a const context, which means it can be used to
    /// initialize a `static`.
    pub const fn new() -> Self {
        Self {
            queue: Queue(heapless::i::Queue::new()),
        }
    }

    /// TODO add docs
    pub fn init(
        &mut self,
    ) -> (
        Producer<'_, E, QueueCap>,
        Consumer<'_, E, QueueCap>,
    ) {
        // TODO init systick here?
        self.queue.split()
    }
}

/// A timer interrupt event
#[derive(Debug)]
pub struct PinMeasurementEvent {
    /// The pin number associated with this event (if any)
    // TODO use lpc845_messages::PinNumber instead (move type somewhere more universal?)
    pub pin_number: u8,
    /// The Level of the pin during measurement
    pub level: gpio::Level,
}

// It would be nice to make the queue capacity configurable, but that would
// require a generic with trait bound on all the structs. As of this writing,
// `const fn`s with trait bounds are unstable, so we can't do it yet.
// note: capacity lowered from U256 to make space for another interruptable pin
type QueueCap = U4;
