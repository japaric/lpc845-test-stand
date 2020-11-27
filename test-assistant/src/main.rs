//! Test assistant firmware
//!
//! Used to assist the test suite in interfacing with the test target. Needs to
//! be downloaded to an LPC845-BRK board before the test cases can be run.


#![no_main]
#![no_std]


extern crate panic_rtt_target;


use core::marker::PhantomData;

use heapless::{
    FnvIndexMap,
    consts::U4,
};
use lpc8xx_hal::{
    prelude::*,
    Peripherals,
    cortex_m::interrupt,
    gpio::{
        self,
        GpioPin,
        DynamicGpioPin,
        direction::{Dynamic},
    },
    i2c,
    init_state::Enabled,
    mrt::{
        MRT0,
        MRT1,
        MRT2,
        MRT3,
    },
    nb::{
        self,
        block,
    },
    pac::{
        I2C0,
        SPI0,
        USART0,
        USART1,
        USART2,
        USART3,
    },
    pinint::{
        PININT0,
        PININT1,
        PININT2,
        PININT3,
    },
    pins::{
        PIO0_8,
        PIO0_9,
        PIO1_1,
    },
    spi::{
        self,
        SPI,
    },
    syscon::{
        IOSC,
        frg,
    },
    usart::{
        self,
        state::{
            AsyncMode,
            SyncMode,
        },
    },
};
use rtt_target::rprintln;

#[cfg(feature = "sleep")]
use lpc8xx_hal::cortex_m::asm;

use firmware_lib::{
    pin_interrupt::{
        self,
        PinInterrupt,
    },
    usart::{
        RxIdle,
        RxInt,
        Tx,
        Usart,
    },
};
use lpc845_messages::{
    AssistantToHost,
    HostToAssistant,
    InputPin,
    DynamicPin,
    UsartMode,
    pin,
};

// By default (and we haven't changed that setting)
// the SysTick timer runs at half the system
// frequency. The system frequency runs at 12 MHz by
// default (again, we haven't changed it), meaning
// the SysTick timer runs at 6 MHz.
//
// At 6 MHz, 1 ms are 6000 timer ticks.
// TODO: value picked for human reada/debuggability; adjust
const TIMER_INT_PERIOD_MS : u32 = 900 * 6000; // fires every 900 milliseconds

// TODO find a place to share them with t-s and t-t?
/// some commonly used pin numbers
const RTS_PIN_NUMBER : u8 = 18;
const CTS_PIN_NUMBER : u8 = 19;
const RED_LED_PIN_NUMBER : u8 = 29;
const GREEN_LED_PIN_NUMBER : u8 = 31;

/// NOTE TO USERS: adjust the pins in this list to change which pin *could* be used as input pin.
/// Currently only two Input-able pins are supported.
#[allow(non_camel_case_types)]
type PININT0_PIN = lpc8xx_hal::pins::PIO1_0;                                 // make sure that these
const PININT0_DYN_PIN: DynamicPin = DynamicPin::GPIO(GREEN_LED_PIN_NUMBER);  // two match!

#[allow(non_camel_case_types)]
type PININT3_PIN = lpc8xx_hal::pins::PIO1_2;                                  // make sure that these
const PININT3_DYN_PIN : DynamicPin = DynamicPin::GPIO(RED_LED_PIN_NUMBER);    // two match!

#[rtic::app(device = lpc8xx_hal::pac)]
const APP: () = {
    struct Resources {
        host_rx_int:  RxInt<'static, USART0, AsyncMode>,
        host_rx_idle: RxIdle<'static>,
        host_tx:      Tx<USART0, AsyncMode>,

        target_rx_int:   RxInt<'static, USART1, AsyncMode>,
        target_rx_idle:  RxIdle<'static>,
        target_tx:       Tx<USART1, AsyncMode>,
        target_tx_dma:   usart::Tx<
            USART2,
            usart::state::Enabled<u8, AsyncMode>,
            usart::state::NoThrottle,
        >,
        target_rts_int:  pin_interrupt::Int<'static, PININT2, PIO0_9, MRT2>,
        target_rts_idle: pin_interrupt::Idle<'static>,

        target_sync_rx_int:  RxInt<'static, USART3, SyncMode>,
        target_sync_rx_idle: RxIdle<'static>,
        target_sync_tx:      Tx<USART3, SyncMode>,

        target_timer_int:  pin_interrupt::Int<'static, PININT1, PIO1_1, MRT1>,
        target_timer_idle: pin_interrupt::Idle<'static>,

        pinint0_int:  pin_interrupt::Int<'static, PININT0, PININT0_PIN, MRT0>,
        pinint0_idle: pin_interrupt::Idle<'static>,

        pinint3_int:  pin_interrupt::Int<'static, PININT3, PININT3_PIN, MRT3>,
        pinint3_idle: pin_interrupt::Idle<'static>,

        dynamic_pins: FnvIndexMap::<u8, (DynamicGpioPin<Dynamic>, Option<pin::Level>), U4>,

        rts: GpioPin<PIO0_9, Dynamic>, // TODO make unidirectional again
        cts: GpioPin<PIO0_8, Dynamic>, // TODO make unidirectional again
        pinint0_pin: GpioPin<PININT0_PIN, Dynamic>, // pin that triggers PININT0 interrupt
        pinint3_pin: GpioPin<PININT3_PIN, Dynamic>, // pin that triggers PININT3 interrupt

        i2c: i2c::Slave<I2C0, Enabled<PhantomData<IOSC>>, Enabled>,
        spi: SPI<SPI0, Enabled<spi::Slave>>,
    }

    #[init]
    fn init(context: init::Context) -> init::LateResources {
        // Normally, access to a `static mut` would be unsafe, but we know that
        // this method is only called once, which means we have exclusive access
        // here. RTFM knows this too, and by putting these statics right here,
        // at the beginning of the method, we're opting into some RTFM magic
        // that gives us safe access to them.
        static mut HOST:        Usart = Usart::new();
        static mut TARGET:      Usart = Usart::new();
        static mut TARGET_SYNC: Usart = Usart::new();

        static mut INT0:         PinInterrupt = PinInterrupt::new();
        static mut INT3:         PinInterrupt = PinInterrupt::new();
        static mut TARGET_TIMER: PinInterrupt = PinInterrupt::new();
        static mut RTS:          PinInterrupt = PinInterrupt::new();

        rtt_target::rtt_init_print!();
        rprintln!("Starting assistant.");

        // Get access to the device's peripherals. This can't panic, since this
        // is the only place in this program where we call this method.
        let p = Peripherals::take().unwrap_or_else(|| unreachable!());

        let mut systick = context.core.SYST;

        let mut syscon = p.SYSCON.split();
        let     swm    = p.SWM.split();
        let     gpio   = p.GPIO.enable(&mut syscon.handle);
        let     pinint = p.PININT.enable(&mut syscon.handle);
        let     timers = p.MRT0.split(&mut syscon.handle);

        let mut swm_handle = swm.handle.enable(&mut syscon.handle);

        // Initialize and enable timer interrupts
        systick.set_reload(TIMER_INT_PERIOD_MS);
        systick.clear_current();
        systick.enable_interrupt();
        systick.enable_counter();

        // Configure interrupts for pins that could be connected to target's GPIO pins
        // TODO more elegantly: make all pins dynamic, interruptable
        let pinint0_pin = p.pins.pio1_0.into_dynamic_pin(
            gpio.tokens.pio1_0,
            gpio::Level::High, // off by default
        );
        let mut pinint0_int = pinint
            .interrupts
            .pinint0
            .select::<PININT0_PIN>(&mut syscon.handle);
        pinint0_int.enable_rising_edge();
        pinint0_int.enable_falling_edge();

        let mut pinint3_pin = p.pins.pio1_2.into_dynamic_pin(
            gpio.tokens.pio1_2,
            gpio::Level::High,
        );

        let mut pinint3_int = pinint
            .interrupts
            .pinint3
            .select::<PININT3_PIN>(&mut syscon.handle);
        pinint3_int.enable_rising_edge();
        pinint3_int.enable_falling_edge();

        // all dynamic pins that are *not* interrupt-controlled
        let mut dynamic_pins = FnvIndexMap::<u8,
            (DynamicGpioPin<Dynamic>, Option<pin::Level>), U4>::new();
        // TODO add ALL the pins \o,
        let mut test_dyn_pin = p.pins.pio0_16.into_dynamic_pin_2(
            gpio.tokens.pio0_16,
            gpio::Level::Low
        );
        test_dyn_pin.switch_to_input(); // TODO note that this is glue and duct tape
        let _ = dynamic_pins.insert(1, (test_dyn_pin, None));

        // Configure interrupt for pin connected to target's timer interrupt pin
        let _target_timer = p.pins.pio1_1.into_input_pin(gpio.tokens.pio1_1);
        let mut target_timer_int = pinint
            .interrupts
            .pinint1
            .select::<PIO1_1>(&mut syscon.handle);
        target_timer_int.enable_rising_edge();
        target_timer_int.enable_falling_edge();

        let cts = p.pins.pio0_8.into_dynamic_pin(
            gpio.tokens.pio0_8,
            gpio::Level::Low,
        );

        // Configure the clock for USART0, using the Fractional Rate Generator
        // (FRG) and the USART's own baud rate divider value (BRG). See user
        // manual, section 17.7.1.
        //
        // This assumes a system clock of 12 MHz (which is the default and, as
        // of this writing, has not been changed in this program). The resulting
        // rate is roughly 115200 baud.
        let clock_config = {
            syscon.frg0.select_clock(frg::Clock::FRO);
            syscon.frg0.set_mult(22);
            syscon.frg0.set_div(0xFF);
            usart::Clock::new(&syscon.frg0, 5, 16)
        };

        // Assign pins to USART0 for RX/TX functions. On the LPC845-BRK, those
        // are the pins connected to the programmer, and bridged to the host via
        // USB.
        //
        // Careful, the LCP845-BRK documentation uses the opposite designations
        // (i.e. from the perspective of the on-board programmer, not the
        // microcontroller).
        let (u0_rxd, _) = swm.movable_functions.u0_rxd.assign(
            p.pins.pio0_24.into_swm_pin(),
            &mut swm_handle,
        );
        let (u0_txd, _) = swm.movable_functions.u0_txd.assign(
            p.pins.pio0_25.into_swm_pin(),
            &mut swm_handle,
        );

        // Use USART0 to communicate with the test suite
        let mut host = p.USART0.enable_async(
            &clock_config,
            &mut syscon.handle,
            u0_rxd,
            u0_txd,
            usart::Settings::default(),
        );
        host.enable_interrupts(usart::Interrupts {
            RXRDY: true,
            .. usart::Interrupts::default()
        });

        // Assign pins to USART1.
        let (u1_rxd, _) = swm.movable_functions.u1_rxd.assign(
            p.pins.pio0_26.into_swm_pin(),
            &mut swm_handle,
        );
        let (u1_txd, _) = swm.movable_functions.u1_txd.assign(
            p.pins.pio0_27.into_swm_pin(),
            &mut swm_handle,
        );

        // Use USART1 to communicate with the test target
        let mut target = p.USART1.enable_async(
            &clock_config,
            &mut syscon.handle,
            u1_rxd,
            u1_txd,
            usart::Settings::default(),
        );
        target.enable_interrupts(usart::Interrupts {
            RXRDY: true,
            .. usart::Interrupts::default()
        });

        // Configure interrupt for RTS pin
        let rts = p.pins.pio0_9.into_dynamic_pin(
            gpio.tokens.pio0_9,
            gpio::Level::High, // off by default (shouldn't matter because rts is input)
        );
        let mut rts_int = pinint
            .interrupts
            .pinint2
            .select::<PIO0_9>(&mut syscon.handle);
        rts_int.enable_rising_edge();
        rts_int.enable_falling_edge();
        let (rts_int, rts_idle) = RTS.init(rts_int, timers.mrt2);

        // Assign pins to USART2.
        let (u2_rxd, _) = swm.movable_functions.u2_rxd.assign(
            p.pins.pio0_28.into_swm_pin(),
            &mut swm_handle,
        );
        let (u2_txd, _) = swm.movable_functions.u2_txd.assign(
            p.pins.pio0_29.into_swm_pin(),
            &mut swm_handle,
        );

        // Use USART2 as secondary means to communicate with test target.
        let target2 = p.USART2.enable_async(
            &clock_config,
            &mut syscon.handle,
            u2_rxd,
            u2_txd,
            usart::Settings::default(),
        );

        // Assign pins to USART3.
        let (u3_rxd, _) = swm.movable_functions.u3_rxd.assign(
            p.pins.pio0_13.into_swm_pin(),
            &mut swm_handle,
        );
        let (u3_txd, _) = swm.movable_functions.u3_txd.assign(
            p.pins.pio0_14.into_swm_pin(),
            &mut swm_handle,
        );
        let (u3_sclk, _) = swm.movable_functions.u3_sclk.assign(
            p.pins.pio0_15.into_swm_pin(),
            &mut swm_handle,
        );

        // Use USART3 as tertiary means to communicate with the test target.
        let mut target_sync = p.USART3.enable_sync_as_slave(
            &syscon.iosc,
            &mut syscon.handle,
            u3_rxd,
            u3_txd,
            u3_sclk,
            usart::Settings::default(),
        );
        target_sync.enable_interrupts(usart::Interrupts {
            RXRDY: true,
            .. usart::Interrupts::default()
        });

        let (host_rx_int,   host_rx_idle,   host_tx)   = HOST.init(host);
        let (target_rx_int, target_rx_idle, target_tx) = TARGET.init(target);
        let (target_sync_rx_int, target_sync_rx_idle, target_sync_tx) =
            TARGET_SYNC.init(target_sync);

        let (pinint0_int, pinint0_idle) = INT0.init(pinint0_int, timers.mrt0);
        let (pinint3_int, pinint3_idle) = INT3.init(pinint3_int, timers.mrt3);
        let (target_timer_int, target_timer_idle) = TARGET_TIMER.init(target_timer_int, timers.mrt1);

        // Assign I2C0 pin functions
        let (i2c0_sda, _) = swm.fixed_functions.i2c0_sda
            .assign(p.pins.pio0_11.into_swm_pin(), &mut swm_handle);
        let (i2c0_scl, _) = swm.fixed_functions.i2c0_scl
            .assign(p.pins.pio0_10.into_swm_pin(), &mut swm_handle);

        // Initialize I2C0
        let mut i2c = p.I2C0
            .enable(
                &syscon.iosc,
                i2c0_scl,
                i2c0_sda,
                &mut syscon.handle,
            )
            .enable_slave_mode(
                0x48,
            )
            .expect("Not using a valid address");
        i2c.enable_interrupts(i2c::Interrupts {
            slave_pending: true,
            .. i2c::Interrupts::default()
        });

        // TODO undo pin change!
        let (spi0_sck, _) = swm
            .movable_functions
            .spi0_sck
            .assign(p.pins.pio0_20.into_swm_pin(), &mut swm_handle);
        let (spi0_mosi, _) = swm
            .movable_functions
            .spi0_mosi
            .assign(p.pins.pio0_17.into_swm_pin(), &mut swm_handle);
        let (spi0_miso, _) = swm
            .movable_functions
            .spi0_miso
            .assign(p.pins.pio0_18.into_swm_pin(), &mut swm_handle);
        let (spi0_ssel0, _) = swm
            .movable_functions
            .spi0_ssel0
            .assign(p.pins.pio0_19.into_swm_pin(), &mut swm_handle);

        let mut spi = p.SPI0.enable_as_slave(
            &syscon.iosc,
            &mut syscon.handle,
            spi::MODE_0,
            spi0_sck,
            spi0_mosi,
            spi0_miso,
            spi0_ssel0,
        );
        spi.enable_interrupts(spi::Interrupts {
            rx_ready: true,
            .. Default::default()
        });
        spi.enable_interrupts(spi::Interrupts {
            rx_ready: true,
            slave_select_asserted: true,
            slave_select_deasserted: true,
            .. Default::default()
        });

        init::LateResources {
            host_rx_int,
            host_rx_idle,
            host_tx,

            target_rx_int,
            target_rx_idle,
            target_tx,
            target_tx_dma:   target2.tx,
            target_rts_int:  rts_int,
            target_rts_idle: rts_idle,

            target_sync_rx_int,
            target_sync_rx_idle,
            target_sync_tx,

            target_timer_int,
            target_timer_idle,

            pinint0_int,
            pinint0_idle,

            pinint3_int,
            pinint3_idle,

            pinint0_pin,
            pinint3_pin,

            dynamic_pins,

            cts,
            rts,

            i2c: i2c.slave,
            spi,
        }
    }

    #[idle(
        resources = [
            host_rx_idle,
            host_tx,
            target_rx_idle,
            target_tx,
            target_tx_dma,
            target_sync_rx_idle,
            target_sync_tx,
            pinint0_idle,
            pinint3_idle,
            target_timer_idle,
            target_rts_idle,
            pinint3_pin,
            pinint0_pin,
            dynamic_pins,
            cts,
            rts,
        ]
    )]
    fn idle(cx: idle::Context) -> ! {
        let host_rx        = cx.resources.host_rx_idle;
        let host_tx        = cx.resources.host_tx;
        let target_rx      = cx.resources.target_rx_idle;
        let target_tx      = cx.resources.target_tx;
        let target_tx_dma  = cx.resources.target_tx_dma;
        let target_sync_rx = cx.resources.target_sync_rx_idle;
        let target_sync_tx = cx.resources.target_sync_tx;
        let target_timer_idle       = cx.resources.target_timer_idle;
        let pinint0_idle   = cx.resources.pinint0_idle;
        let pinint3_idle   = cx.resources.pinint3_idle;
        let target_rts_idle= cx.resources.target_rts_idle;
        let pinint0_pin             = cx.resources.pinint0_pin;
        let pinint3_pin             = cx.resources.pinint3_pin;
        let dynamic_pins   = cx.resources.dynamic_pins;
        let cts            = cx.resources.cts;
        let rts            = cx.resources.rts;

        let mut pins = FnvIndexMap::<_, _, U4>::new();

        // TODO pick name that is less easy to confuse with `dynamic_pins`
        let mut dynamic_int_pins = FnvIndexMap::<_, _, U4>::new();

        let mut buf = [0; 256];

        loop {
            target_rx
                .process_raw(|data| {
                    host_tx.send_message(
                        &AssistantToHost::UsartReceive {
                            mode: UsartMode::Regular,
                            data,
                        },
                        &mut buf,
                    )
                })
                .expect("Error processing USART data");
            target_sync_rx
                .process_raw(|data| {
                    host_tx.send_message(
                        &AssistantToHost::UsartReceive {
                            mode: UsartMode::Sync,
                            data,
                        },
                        &mut buf,
                    )
                })
                .expect("Error processing USART data");

            host_rx
                .process_message(|message| {
                    match message {
                        HostToAssistant::SendUsart {
                            mode: UsartMode::Regular,
                            data,
                        } => {
                            target_tx.send_raw(data)
                        }
                        HostToAssistant::SendUsart {
                            mode: UsartMode::Dma,
                            data,
                        } => {
                            target_tx_dma.bwrite_all(data)
                        }
                        HostToAssistant::SendUsart {
                            mode: UsartMode::FlowControl,
                            data: _,
                        } => {
                            Ok(())
                        }
                        HostToAssistant::SendUsart {
                            mode: UsartMode::Sync,
                            data,
                        } => {
                            target_sync_tx.send_raw(data)
                        }
                        HostToAssistant::SetPin(
                            pin::SetLevel {
                                pin: _,
                                level: _,
                            }
                        ) => {
                            // currently we don't have defined any non-dynamic Output Pins that could be set
                            // TODO refactor back: this should be usable
                            unreachable!()
                        }
                        HostToAssistant::SetDynamicPin(
                            pin::SetLevel {
                                pin,
                                level,
                            }
                        ) => {
                            // todo nicer and more generic once we resolve the Pin Type Conundrum
                            let pin_is_output: bool = match pin {
                                PININT3_DYN_PIN => pinint3_pin.direction_is_output(),
                                PININT0_DYN_PIN => pinint0_pin.direction_is_output(),
                                DynamicPin::GPIO(CTS_PIN_NUMBER) => cts.direction_is_output(),
                                _ => false
                            };

                            if pin_is_output {
                                // todo nicer and more generic once we resolve the Pin Type Conundrum
                                match level {
                                    pin::Level::High => {
                                        rprintln!("dynamic HIGH for {:?}", pin);
                                        match pin {
                                            PININT3_DYN_PIN => pinint3_pin.set_high(),
                                            PININT0_DYN_PIN => pinint0_pin.set_high(),
                                            DynamicPin::GPIO(RTS_PIN_NUMBER) => cts.set_high(),
                                            _ => todo!(),
                                        };
                                    }
                                    pin::Level::Low => {
                                        rprintln!("dynamic LOW for {:?}", pin);
                                        match pin {
                                            PININT3_DYN_PIN => pinint3_pin.set_low(),
                                            PININT0_DYN_PIN => pinint0_pin.set_low(),
                                            DynamicPin::GPIO(RTS_PIN_NUMBER) => cts.set_low(),
                                            _ => todo!(),
                                        };
                                    }
                                }
                            }
                            else {
                                rprintln!("Warning: Can't set pin #{} since it is configured as input.",
                                          get_pin_number(pin) );
                            }
                            Ok(())
                        }
                        HostToAssistant::ReadPin(
                            pin::ReadLevel { pin }
                        ) => {
                            let result = pins.get(&(pin as usize))
                                .map(|&(level, period_ms)| {
                                    pin::ReadLevelResult {
                                        pin,
                                        level,
                                        period_ms,
                                    }
                                });

                            host_tx
                                .send_message(
                                    &AssistantToHost::ReadPinResult(result),
                                    &mut buf,
                                )
                                .unwrap();

                            Ok(())
                        }
                        HostToAssistant::SetDirection(
                            pin::SetDirection {
                                pin,
                                direction: pin::Direction::Input,
                                level: None,
                            }
                        ) => {
                            rprintln!("SET DIRECTION -> INPUT for {:?}.", pin);
                            // todo nicer and more generic once we start enabling ALL the pins
                            // note: the problem with moving this to a helper is again that pin IDs are
                            // part of the GPIO type so we can't just move this into a
                            // fn get_gpio_from_pin_number() -> Option<GpioPin<??, Dynamic>>
                            // (I'll call this the in Type Conundrum for now)
                            match pin {
                                PININT3_DYN_PIN => pinint3_pin.switch_to_input(),
                                PININT0_DYN_PIN => pinint0_pin.switch_to_input(),
                                // TODO don't hardcode, check dynamic_pins
                                DynamicPin::GPIO(1) => {
                                    // Ignore for now, we've hardcoded this pin as input
                                    // TODO fix this (I think the problem is that we can't just pass
                                    // an IndexMap as a resource? that'd also explain the int/idle split)
                                },
                                DynamicPin::GPIO(CTS_PIN_NUMBER) => {
                                    // TODO proper error handling
                                    rprintln!("CTS pin is never Input");
                                    unreachable!()
                                },
                                DynamicPin::GPIO(RTS_PIN_NUMBER) => {
                                    rts.switch_to_input()
                                }
                                _ => todo!(),
                            };
                            Ok(())
                        },
                        HostToAssistant::SetDirection(
                            pin::SetDirection {
                                pin,
                                direction: pin::Direction::Output,
                                level: Some(level),
                            }
                        ) => {
                            rprintln!("SET DIRECTION -> OUTPUT for {:?}. Level {:?}", pin, level);
                            // convert from lpc8xx_hal::gpio::Level to protocol::pin::Level
                            // TODO impl From instead?
                            let gpio_level = match level {
                                pin::Level::High => {gpio::Level::High}
                                pin::Level::Low => {gpio::Level::Low}
                            };

                            // todo nicer and more generic once we start enabling ALL the pins
                            match pin {
                                PININT3_DYN_PIN => pinint3_pin.switch_to_output(gpio_level),
                                PININT0_DYN_PIN => pinint0_pin.switch_to_output(gpio_level),
                                DynamicPin::GPIO(CTS_PIN_NUMBER) => cts.switch_to_output(gpio_level),
                                DynamicPin::GPIO(RTS_PIN_NUMBER) => {
                                    // TODO proper error handling
                                    rprintln!("RTS pin is never Output");
                                    unreachable!()
                                }
                                _ => todo!(),
                            };
                            Ok(())
                        },
                        HostToAssistant::SetDirection(
                            pin::SetDirection {
                                pin: _,
                                direction: _,
                                level: _,
                            }
                        ) => {
                            // illegal level/direction combination
                            // TODO handle error more neatly
                            unreachable!()
                        },
                        HostToAssistant::ReadDynamicPin(
                            pin::ReadLevel { pin }
                        ) => {
                            rprintln!("READ DYNAMIC PIN command for {:?}", pin);
                            rprintln!("dynamic_int_pins: {:?}", dynamic_int_pins);

                            // todo nicer and more generic once we resolve the Pin Type Conundrum
                            let pin_is_input: bool = match pin {
                                PININT3_DYN_PIN => pinint3_pin.direction_is_input(),
                                PININT0_DYN_PIN => pinint0_pin.direction_is_input(),
                                DynamicPin::GPIO(1) => {
                                    // TODO don't hardcode this! (see discussion in SetDirection)
                                    true
                                },
                                DynamicPin::GPIO(RTS_PIN_NUMBER) => rts.direction_is_input(),
                                _ => false
                            };

                            let result = match pin_is_input {
                                true => {
                                    // TODO: really applicable to all?
                                    let pin_number = get_pin_number(pin);

                                    dynamic_int_pins
                                    .get(&(pin_number as usize))
                                    .map(|&(level, period_ms)| {
                                        pin::ReadLevelResult {
                                            pin,
                                            level,
                                            period_ms,
                                        }
                                    })
                                }
                                false => {
                                    rprintln!("Warning: Can't read pin #{} since it is configured as output.",
                                              get_pin_number(pin));
                                    None
                                }
                            };

                            rprintln!("sending read result: {:?}", result);

                            host_tx
                                .send_message(
                                    &AssistantToHost::ReadPinResultDynamic(result),
                                    &mut buf,
                                )
                                .unwrap();

                            Ok(())

                        }
                    }
                })
                .expect("Error processing host request");
            host_rx.clear_buf();

            // TODO only do this for pins that are currently in input direction?
            handle_pin_interrupt_dynamic(pinint0_idle, PININT0_DYN_PIN, &mut dynamic_int_pins);
            handle_pin_interrupt_dynamic(pinint3_idle, PININT3_DYN_PIN, &mut dynamic_int_pins);
            handle_pin_interrupt_dynamic(target_rts_idle, DynamicPin::GPIO(RTS_PIN_NUMBER), &mut dynamic_int_pins);
            handle_pin_interrupt(target_timer_idle, InputPin::TargetTimer, &mut pins);

            // We need this critical section to protect against a race
            // conditions with the interrupt handlers. Otherwise, the following
            // sequence of events could occur:
            // 1. We check the queues here, they're empty.
            // 2. New data is received, an interrupt handler adds it to a queue.
            // 3. The interrupt handler is done, we're back here and going to
            //    sleep.
            //
            // This might not be observable, if something else happens to wake
            // us up before the test suite times out. But it could also lead to
            // spurious test failures.
            interrupt::free(|_| {
                let should_sleep =
                    !host_rx.can_process()
                    && !target_rx.can_process()
                    && pinint0_idle.is_ready(); // TODO double check for soundness

                if should_sleep {
                    // On LPC84x MCUs, debug mode is not supported when
                    // sleeping. This interferes with RTT communication. Only
                    // sleep, if the user enables this through a compile-time
                    // flag.
                    #[cfg(feature = "sleep")]
                    asm::wfi();
                }
            });
        }
    }

    #[task(binds = USART0, resources = [host_rx_int])]
    fn usart0(cx: usart0::Context) {
        cx.resources.host_rx_int.receive()
            .expect("Error receiving from USART0");
    }

    #[task(binds = USART1, resources = [target_rx_int])]
    fn usart1(cx: usart1::Context) {
        cx.resources.target_rx_int.receive()
            .expect("Error receiving from USART1");
    }

    #[task(binds = PIN_INT6_USART3, resources = [target_sync_rx_int])]
    fn usart3(cx: usart3::Context) {
        cx.resources.target_sync_rx_int.receive()
            .expect("Error receiving from USART3");
    }

    #[task(binds = PIN_INT3, resources = [pinint3_int])]
    fn pinint3(context: pinint3::Context) {
        context.resources.pinint3_int.handle_interrupt();
    }

    #[task(binds = PIN_INT0, resources = [pinint0_int])]
    fn pinint0(context: pinint0::Context) {
        context.resources.pinint0_int.handle_interrupt();
    }

    #[task(binds = PIN_INT1, resources = [target_timer_int])]
    fn pinint1(context: pinint1::Context) {
        context.resources.target_timer_int.handle_interrupt();
    }

    #[task(binds = PIN_INT2, resources = [target_rts_int])]
    fn pinint2(context: pinint2::Context) {
        context.resources.target_rts_int.handle_interrupt();
    }

    #[task(binds = SysTick, resources = [dynamic_pins])]
    fn syst(context: syst::Context) {
        // TODO there's probably a race condition in here, re-think this

        for tuple in context.resources.dynamic_pins.values_mut() {
            // TODO de-uglify
            match tuple {
                (pin, level) => {
                    if pin.direction_is_input() {
                        let l = match pin.is_high() {
                            // TODO rm debug oputput
                            true => {
                                rprintln!("h");
                                pin::Level::High
                            },
                            false => {
                                rprintln!("l");
                                pin::Level::Low
                            },
                        };
                        // TODO un-uncomment
                        //*level = Some(l);
                    }
                }
            }
        }
    }

    #[task(binds = I2C0, resources = [i2c])]
    fn i2c0(context: i2c0::Context) {
        static mut DATA: Option<u8> = None;

        rprintln!("I2C: Handling I2C0 interrupt...");

        match context.resources.i2c.wait() {
            Ok(i2c::slave::State::AddressMatched(i2c)) => {
                rprintln!("I2C: Address matched.");

                i2c.ack().unwrap();

                rprintln!("I2C: Ack'ed address.");
            }
            Ok(i2c::slave::State::RxReady(i2c)) => {
                rprintln!("I2C: Ready to receive.");

                *DATA = Some(i2c.read().unwrap());
                i2c.ack().unwrap();

                rprintln!("I2C: Received and ack'ed.");
            }
            Ok(i2c::slave::State::TxReady(i2c)) => {
                rprintln!("I2C: Ready to transmit.");

                if let Some(data) = *DATA {
                    i2c.transmit(data << 1).unwrap();
                    rprintln!("I2C: Transmitted.");
                }
            }
            Err(nb::Error::WouldBlock) => {
                // I2C not ready; nothing to do
            }
            Err(err) => {
                panic!("I2C error: {:?}", err);
            }
        }
    }

    #[task(binds = SPI0, resources = [spi])]
    fn spi0(context: spi0::Context) {
        static mut ACTIVE: bool = false;

        let spi = context.resources.spi;

        if spi.is_slave_select_asserted() {
            *ACTIVE = true;
        }
        if *ACTIVE {
            if spi.is_ready_to_receive() {
                let data = spi.receive().unwrap();
                block!(spi.transmit(data << 1))
                    .unwrap();
            }
        }
        if spi.is_slave_select_deasserted() {
            *ACTIVE = false;
        }
    }
};

/// Collect data from all Interrupts that were fired for `pin`
fn handle_pin_interrupt_dynamic(
    int:  &mut pin_interrupt::Idle,
    pin:  DynamicPin,
    // TODO: why are we even using usize for index if values never exceed u0?
    pins: &mut FnvIndexMap<usize, (pin::Level, Option<u32>), U4>,
) {
    while let Some(event) = int.next() {
        match event {
            pin_interrupt::Event { level, period } => {
                let pin_number = get_pin_number(pin);

                let level = match level {
                    gpio::Level::High => pin::Level::High,
                    gpio::Level::Low  => pin::Level::Low,
                };

                let period_ms = period.map(|value| value / 12_000);
                pins.insert(pin_number as usize, (level, period_ms)).unwrap();
            }
        }
    }
}

// TODO merge w handle_pin_interrupt_dynamic / make more generic
fn handle_pin_interrupt(
    int:  &mut pin_interrupt::Idle,
    pin:  InputPin,
    pins: &mut FnvIndexMap<usize, (pin::Level, Option<u32>), U4>,
) {
    while let Some(event) = int.next() {
        match event {
            pin_interrupt::Event { level, period } => {
                let level = match level {
                    gpio::Level::High => pin::Level::High,
                    gpio::Level::Low  => pin::Level::Low,
                };

                let period_ms = period.map(|value| value / 12_000);
                pins.insert(pin as usize, (level, period_ms)).unwrap();
            }
        }
    }
}

/// Get the index of `pin` as counted on the breakout board (Arduino Style)
// TODO return Option instead of panicking here
fn get_pin_number(pin: DynamicPin) -> u8 {
    match pin {
        DynamicPin::GPIO(number) => number,
        _ => todo!()
    }
}
