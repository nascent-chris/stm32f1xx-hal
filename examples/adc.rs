#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{adc, pac, prelude::*};

use cortex_m_semihosting::hprintln;

#[entry]
fn main() -> ! {
    // Acquire peripherals
    let p = pac::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    // Configure ADC clocks
    // Default value is the slowest possible ADC clock: PCLK2 / 8. Meanwhile ADC
    // clock is configurable. So its frequency may be tweaked to meet certain
    // practical needs. User specified value is be approximated using supported
    // prescaler values 2/4/6/8.
    let clocks = rcc.cfgr.adcclk(2.MHz()).freeze(&mut flash.acr);
    hprintln!("adc freq: {}", clocks.adcclk());

    // Setup ADC
    let mut adc1 = adc::Adc::adc1(p.ADC1, clocks);

    #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
    let mut adc2 = adc::Adc::adc2(p.ADC2, clocks);

    // Setup GPIOB
    let mut gpiob = p.GPIOB.split();

    // Configure pb0, pb1 as an analog input
    let mut ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);

    #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
    let mut ch1 = gpiob.pb1.into_analog(&mut gpiob.crl);

    loop {
        let data: u16 = adc1.read(&mut ch0).unwrap();
        hprintln!("adc1: {}", data);

        #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
        {
            let data1: u16 = adc2.read(&mut ch1).unwrap();
            hprintln!("adc2: {}", data1);
        }
    }
}
