//! Inter-Integrated Circuit (I2C) bus

// This document describes a correct i2c implementation and is what
// parts of this code is based on
// https://www.st.com/content/ccc/resource/technical/document/application_note/5d/ae/a3/6f/08/69/4e/9b/CD00209826.pdf/files/CD00209826.pdf/jcr:content/translations/en.CD00209826.pdf

use crate::gpio::{self, Alternate, Cr, OpenDrain};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::pac::{AFIO, DWT, I2C1, I2C2, RCC};
use crate::rcc::{BusClock, Clocks, Enable, Reset};
use crate::time::{kHz, Hertz};
use core::ops::Deref;

pub mod blocking;
pub use blocking::BlockingI2c;

/// I2C error
#[derive(Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// No ack received
    Acknowledge,
    /// Overrun/underrun
    Overrun,
    // Pec, // SMBUS mode only
    Timeout,
    // Alert, // SMBUS mode only
}

#[derive(Debug, Eq, PartialEq)]
pub enum DutyCycle {
    Ratio2to1,
    Ratio16to9,
}

#[derive(Debug, PartialEq, Eq)]
pub enum Mode {
    Standard {
        frequency: Hertz,
    },
    Fast {
        frequency: Hertz,
        duty_cycle: DutyCycle,
    },
}

impl Mode {
    pub fn standard(frequency: Hertz) -> Self {
        Mode::Standard { frequency }
    }

    pub fn fast(frequency: Hertz, duty_cycle: DutyCycle) -> Self {
        Mode::Fast {
            frequency,
            duty_cycle,
        }
    }

    pub fn get_frequency(&self) -> Hertz {
        match *self {
            Mode::Standard { frequency } => frequency,
            Mode::Fast { frequency, .. } => frequency,
        }
    }
}

impl From<Hertz> for Mode {
    fn from(frequency: Hertz) -> Self {
        if frequency <= kHz(100) {
            Self::Standard { frequency }
        } else {
            Self::Fast {
                frequency,
                duty_cycle: DutyCycle::Ratio2to1,
            }
        }
    }
}

pub mod i2c1 {
    use super::*;

    pub enum Pins {
        No(
            gpio::PB6<Alternate<OpenDrain>>,
            gpio::PB7<Alternate<OpenDrain>>,
        ),
        Remap1(
            gpio::PB8<Alternate<OpenDrain>>,
            gpio::PB9<Alternate<OpenDrain>>,
        ),
    }
    impl
        From<(
            gpio::PB6<Alternate<OpenDrain>>,
            gpio::PB7<Alternate<OpenDrain>>,
        )> for Pins
    {
        fn from(
            p: (
                gpio::PB6<Alternate<OpenDrain>>,
                gpio::PB7<Alternate<OpenDrain>>,
            ),
        ) -> Self {
            let mapr = unsafe { &(*AFIO::ptr()).mapr };
            mapr.modify(|_, w| w.i2c1_remap().bit(false));
            Self::No(p.0, p.1)
        }
    }
    impl From<(gpio::PB6, gpio::PB7)> for Pins {
        fn from(p: (gpio::PB6, gpio::PB7)) -> Self {
            let mapr = unsafe { &(*AFIO::ptr()).mapr };
            mapr.modify(|_, w| w.i2c1_remap().bit(false));
            let mut crl = Cr::new();
            Self::No(
                p.0.into_alternate_open_drain(&mut crl),
                p.1.into_alternate_open_drain(&mut crl),
            )
        }
    }
    impl
        From<(
            gpio::PB8<Alternate<OpenDrain>>,
            gpio::PB9<Alternate<OpenDrain>>,
        )> for Pins
    {
        fn from(
            p: (
                gpio::PB8<Alternate<OpenDrain>>,
                gpio::PB9<Alternate<OpenDrain>>,
            ),
        ) -> Self {
            let mapr = unsafe { &(*AFIO::ptr()).mapr };
            mapr.modify(|_, w| w.i2c1_remap().bit(true));
            Self::Remap1(p.0, p.1)
        }
    }
    impl From<(gpio::PB8, gpio::PB9)> for Pins {
        fn from(p: (gpio::PB8, gpio::PB9)) -> Self {
            let mapr = unsafe { &(*AFIO::ptr()).mapr };
            mapr.modify(|_, w| w.i2c1_remap().bit(true));
            let mut crh = Cr::new();
            Self::Remap1(
                p.0.into_alternate_open_drain(&mut crh),
                p.1.into_alternate_open_drain(&mut crh),
            )
        }
    }
}
pub mod i2c2 {
    use super::*;

    pub enum Pins {
        No(
            gpio::PB10<Alternate<OpenDrain>>,
            gpio::PB11<Alternate<OpenDrain>>,
        ),
    }
    impl
        From<(
            gpio::PB10<Alternate<OpenDrain>>,
            gpio::PB11<Alternate<OpenDrain>>,
        )> for Pins
    {
        fn from(
            p: (
                gpio::PB10<Alternate<OpenDrain>>,
                gpio::PB11<Alternate<OpenDrain>>,
            ),
        ) -> Self {
            Self::No(p.0, p.1)
        }
    }
    impl From<(gpio::PB10, gpio::PB11)> for Pins {
        fn from(p: (gpio::PB10, gpio::PB11)) -> Self {
            let mut crh = Cr::new();
            Self::No(
                p.0.into_alternate_open_drain(&mut crh),
                p.1.into_alternate_open_drain(&mut crh),
            )
        }
    }
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C: Instance> {
    i2c: I2C,
    pins: I2C::Pins,
    mode: Mode,
    pclk1: Hertz,
}

pub trait Instance:
    crate::Sealed + Deref<Target = crate::pac::i2c1::RegisterBlock> + Enable + Reset + BusClock
{
    type Pins;
}

impl Instance for I2C1 {
    type Pins = i2c1::Pins;
}
impl Instance for I2C2 {
    type Pins = i2c2::Pins;
}

impl<I2C: Instance> I2c<I2C> {
    /// Creates a generic I2C object
    pub fn new<M: Into<Mode>>(
        i2c: I2C,
        pins: impl Into<I2C::Pins>,
        mode: M,
        clocks: Clocks,
    ) -> Self {
        let mode = mode.into();
        let rcc = unsafe { &(*RCC::ptr()) };
        I2C::enable(rcc);
        I2C::reset(rcc);

        let pclk1 = I2C::clock(&clocks);

        assert!(mode.get_frequency() <= kHz(400));

        let mut i2c = I2c {
            i2c,
            pins: pins.into(),
            mode,
            pclk1,
        };
        i2c.init();
        i2c
    }
}

impl<I2C: Instance> I2c<I2C> {
    /// Initializes I2C. Configures the `I2C_TRISE`, `I2C_CRX`, and `I2C_CCR` registers
    /// according to the system frequency and I2C mode.
    fn init(&mut self) {
        let freq = self.mode.get_frequency();
        let pclk1_mhz = self.pclk1.to_MHz() as u16;

        self.i2c
            .cr2
            .write(|w| unsafe { w.freq().bits(pclk1_mhz as u8) });
        self.i2c.cr1.write(|w| w.pe().clear_bit());

        match self.mode {
            Mode::Standard { .. } => {
                self.i2c
                    .trise
                    .write(|w| w.trise().bits((pclk1_mhz + 1) as u8));
                self.i2c
                    .ccr
                    .write(|w| unsafe { w.ccr().bits(((self.pclk1 / (freq * 2)) as u16).max(4)) });
            }
            Mode::Fast { ref duty_cycle, .. } => {
                self.i2c
                    .trise
                    .write(|w| w.trise().bits((pclk1_mhz * 300 / 1000 + 1) as u8));

                self.i2c.ccr.write(|w| {
                    let (freq, duty) = match duty_cycle {
                        DutyCycle::Ratio2to1 => (((self.pclk1 / (freq * 3)) as u16).max(1), false),
                        DutyCycle::Ratio16to9 => (((self.pclk1 / (freq * 25)) as u16).max(1), true),
                    };

                    unsafe { w.ccr().bits(freq).duty().bit(duty).f_s().set_bit() }
                });
            }
        };

        self.i2c.cr1.modify(|_, w| w.pe().set_bit());
    }

    /// Perform an I2C software reset
    fn reset(&mut self) {
        self.i2c.cr1.write(|w| w.pe().set_bit().swrst().set_bit());
        self.i2c.cr1.reset();
        self.init();
    }

    /// Generate START condition
    fn send_start(&mut self) {
        self.i2c.cr1.modify(|_, w| w.start().set_bit());
    }

    /// Sends the (7-Bit) address on the I2C bus. The 8th bit on the bus is set
    /// depending on wether it is a read or write transfer.
    fn send_addr(&self, addr: u8, read: bool) {
        self.i2c
            .dr
            .write(|w| w.dr().bits(addr << 1 | (u8::from(read))));
    }

    /// Generate STOP condition
    fn send_stop(&self) {
        self.i2c.cr1.modify(|_, w| w.stop().set_bit());
    }

    /// Releases the I2C peripheral and associated pins
    pub fn release(self) -> (I2C, I2C::Pins) {
        (self.i2c, self.pins)
    }
}
