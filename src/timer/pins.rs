use crate::pac;
use crate::gpio::{self, Cr, Alternate};
use crate::afio::MAPR;

pub trait CPin<REMAP, const C: u8> {}
pub struct Ch<const C: u8>;
pub const C1: u8 = 0;
pub const C2: u8 = 1;
pub const C3: u8 = 2;
pub const C4: u8 = 3;

pub(crate) mod sealed {
    pub trait Remap {
        type Periph;
        const REMAP: u8;

        fn remap(mapr: &mut crate::afio::MAPR);
    }
}

macro_rules! remap {
    ($($name:ident: ($TIMX:ty, $state:literal, $P1:ident, $P2:ident, $P3:ident, $P4:ident, { $remapex:expr }),)+) => {
        $(
            pub struct $name;
            impl sealed::Remap for $name {
                type Periph = $TIMX;
                const REMAP: u8 = $state;

                fn remap(mapr: &mut crate::afio::MAPR) {
                    mapr.modify_mapr($remapex);
                }
            }
            impl<MODE> CPin<$name, 0> for crate::gpio::$P1<MODE> {}
            impl<MODE> CPin<$name, 1> for crate::gpio::$P2<MODE> {}
            impl<MODE> CPin<$name, 2> for crate::gpio::$P3<MODE> {}
            impl<MODE> CPin<$name, 3> for crate::gpio::$P4<MODE> {}
        )+
    }
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity",))]
remap!(
    Tim1NoRemap: (pac::TIM1, 0b00, PA8, PA9, PA10, PA11, {|_, w| unsafe { w.tim1_remap().bits(Self::REMAP)}}),
    //Tim1PartialRemap: (pac::TIM1, 0b01, PA8, PA9, PA10, PA11),
    Tim1FullRemap: (pac::TIM1, 0b11, PE9, PE11, PE13, PE14, {|_, w| unsafe { w.tim1_remap().bits(Self::REMAP)}}),
);

remap!(
    Tim2NoRemap: (pac::TIM2, 0b00, PA0, PA1, PA2, PA3, {|_, w| unsafe { w.tim2_remap().bits(Self::REMAP)}}),
    Tim2PartialRemap1: (pac::TIM2, 0b01, PA15, PB3, PA2, PA3, {|_, w| unsafe { w.tim2_remap().bits(Self::REMAP)}}),
    Tim2PartialRemap2: (pac::TIM2, 0b10, PA0, PA1, PB10, PB11, {|_, w| unsafe { w.tim2_remap().bits(Self::REMAP)}}),
    Tim2FullRemap: (pac::TIM2, 0b11, PA15, PB3, PB10, PB11, {|_, w| unsafe { w.tim2_remap().bits(Self::REMAP)}}),

    Tim3NoRemap: (pac::TIM3, 0b00, PA6, PA7, PB0, PB1, {|_, w| unsafe { w.tim3_remap().bits(Self::REMAP)}}),
    Tim3PartialRemap: (pac::TIM3, 0b10, PB4, PB5, PB0, PB1, {|_, w| unsafe { w.tim3_remap().bits(Self::REMAP)}}),
    Tim3FullRemap: (pac::TIM3, 0b11, PC6, PC7, PC8, PC9, {|_, w| unsafe { w.tim3_remap().bits(Self::REMAP)}}),
);

#[cfg(feature = "medium")]
remap!(
    Tim4NoRemap: (pac::TIM4, 0b00, PB6, PB7, PB8, PB9, {|_, w| w.tim4_remap().bit(Self::REMAP == 1)}),
    Tim4Remap: (pac::TIM4, 0b01, PD12, PD13, PD14, PD15, {|_, w| w.tim4_remap().bit(Self::REMAP == 1)}),
);

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
pub mod tim1 {
    use super::*;

    remap4! {
        Pins: [
            No, PA8, PA9, PA10, PA11 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0)} };
            Full, PE9, PE11, PE13, PE14 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0b11)} };
        ]
    }
}

pub mod tim2 {
    use super::*;

    remap4! {
        Pins: [
            No, PA0, PA1, PA2, PA3 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0)} };
            Partial1, PA15, PB3, PA2, PA3 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b01)} };
            Partial2, PA0, PA1, PB10, PB11 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b10)} };
            Full, PA15, PB3, PB10, PB11 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b11)} };
        ]
    }
}

macro_rules! remap4 {
    ($name:ident: [
        $($rname:ident, $P1:ident, $P2:ident, $P3:ident, $P4:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        remap_combo! {
            X4 => Channels1234: [
                $($rname, $P1, $P2, $P3, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X3 => Channels123: [
                $($rname, $P1, $P2, $P3 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X3 => Channels124: [
                $($rname, $P1, $P2, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X3 => Channels134: [
                $($rname, $P1, $P3, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X3 => Channels234: [
                $($rname, $P2, $P3, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X2 => Channels12: [
                $($rname, $P1, $P2 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X2 => Channels13: [
                $($rname, $P1, $P3 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X2 => Channels14: [
                $($rname, $P1, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X2 => Channels23: [
                $($rname, $P2, $P3 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X2 => Channels24: [
                $($rname, $P2, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X2 => Channels34: [
                $($rname, $P3, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X1 => Channel1: [
                $($rname, $P1 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X1 => Channel2: [
                $($rname, $P2 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X1 => Channel3: [
                $($rname, $P3 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            X1 => Channel4: [
                $($rname, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
    }
}
use remap4;

macro_rules! remap_combo {
    (X4 => $name:ident: [
        $($rname:ident, $P0:ident, $P1:ident, $P2:ident, $P3:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        pub enum $name {
            $(
                $rname(gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate>, gpio::$P3<Alternate>),
            )+
        }

        $(
            impl From<(gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate>, gpio::$P3<Alternate> $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate>, gpio::$P3<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(p.4.modify_mapr($remapex);)?
                    Self::$rname(p.0, p.1, p.2, p.3)
                }
            }

            impl From<(gpio::$P0, gpio::$P1, gpio::$P2, gpio::$P3 $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0, gpio::$P1, gpio::$P2, gpio::$P3 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr::new());
                    let p1 = p.1.into_mode(&mut Cr::new());
                    let p2 = p.2.into_mode(&mut Cr::new());
                    let p3 = p.3.into_mode(&mut Cr::new());
                    $(p.4.modify_mapr($remapex);)?
                    Self::$rname(p0, p1, p2, p3)
                }
            }
        )+
    };

    (X3 => $name:ident: [
        $($rname:ident, $P0:ident, $P1:ident, $P2:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        pub enum $name {
            $(
                $rname(gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate>),
            )+
        }

        $(
            impl From<(gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate> $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(p.3.modify_mapr($remapex);)?
                    Self::$rname(p.0, p.1, p.2)
                }
            }

            impl From<(gpio::$P0, gpio::$P1, gpio::$P2 $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0, gpio::$P1, gpio::$P2 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr::new());
                    let p1 = p.1.into_mode(&mut Cr::new());
                    let p2 = p.2.into_mode(&mut Cr::new());
                    $(p.3.modify_mapr($remapex);)?
                    Self::$rname(p0, p1, p2)
                }
            }
        )+
    };

    (X2 => $name:ident: [
        $($rname:ident, $P0:ident, $P1:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        pub enum $name {
            $(
                $rname(gpio::$P0<Alternate>, gpio::$P1<Alternate>),
            )+
        }

        $(
            impl From<(gpio::$P0<Alternate>, gpio::$P1<Alternate> $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0<Alternate>, gpio::$P1<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(p.2.modify_mapr($remapex);)?
                    Self::$rname(p.0, p.1)
                }
            }

            impl From<(gpio::$P0, gpio::$P1 $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0, gpio::$P1 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr::new());
                    let p1 = p.1.into_mode(&mut Cr::new());
                    $(p.2.modify_mapr($remapex);)?
                    Self::$rname(p0, p1)
                }
            }
        )+
    };
    
    (X1 => $name:ident: [
        $($rname:ident, $P0:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        pub enum $name {
            $(
                $rname(gpio::$P0<Alternate>),
            )+
        }

        $(
            impl From<(gpio::$P0<Alternate> $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(p.1.modify_mapr($remapex);)?
                    Self::$rname(p.0)
                }
            }

            impl From<(gpio::$P0 $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr::new());
                    $(p.1.modify_mapr($remapex);)?
                    Self::$rname(p0)
                }
            }
        )+
    };
}
use remap_combo;