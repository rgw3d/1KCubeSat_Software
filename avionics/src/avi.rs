extern crate stm32l4xx_hal as hal;
use packed_struct::prelude::*;

use hal::{
    adc, delay, gpio, prelude::*, serial, serial::Serial, stm32::UART4, stm32::USART2,
    stm32::USART3,
};

#[derive(core::fmt::Debug, core::marker::Copy, core::clone::Clone)]
pub enum RadioState {
    RadioPowerFailure,
    SendRadioOffCmd,
    WaitRadioOffCmd,
    SendHpwrEnOffCmd,
    WaitHpwrEnOffCmd,
    Send3V3RailOffCmd,
    Wait3V3RailOffCmd,
    RadioOff,
    Send3V3RailOnCmd,
    Wait3V3RailOnCmd1,
    Verify3V3RailOnCmd,
    WaitVerify3V3RailOnCmd1,
    SendHpwrEnCmd,
    WaitHpwrEnCmd1,
    SendRadioOnCmd,
    WaitRadioOnCmd1,
    VerifyHpwrOnCmd,
    WaitVerifyHpwrOnCmd1,
    VerifyRadioOnCmd,
    WaitVerifyRadioOnCmd1,
    RadioOnNop,
    RadioOnAction,
}

#[derive(core::fmt::Debug, core::marker::Copy, core::clone::Clone)]
pub enum RadioAction {
    PopulateTelem,
}

#[derive(PackedStruct, core::fmt::Debug, core::marker::Copy, core::clone::Clone)]
#[packed_struct(endian = "lsb")]
pub struct RadioMessageHeader {
    pub hwid: u16,
    pub sequence_number: u16,
    pub destination: u8,
    pub command_id: u8,
}

pub struct AVI {
    //device: hal::stm32::Peripherals,
    //parts: GpioBankParts,
    pub digital_pins: DigitalPins,
    pub analog_pins: AnalogPins,
    pub debug_rx: serial::Rx<USART2>,
    pub debug_tx: serial::Tx<USART2>,
    pub conn_rx: serial::Rx<UART4>,
    pub conn_tx: serial::Tx<UART4>,
    pub radio_rx: serial::Rx<USART3>,
    pub radio_tx: serial::Tx<USART3>,
    pub led1: gpio::PF2<gpio::Output<gpio::OpenDrain>>,
    pub led2: gpio::PF3<gpio::Output<gpio::OpenDrain>>,
    pub led3: gpio::PF4<gpio::Output<gpio::OpenDrain>>,
    pub led4: gpio::PF5<gpio::Output<gpio::OpenDrain>>,
    pub led5: gpio::PF6<gpio::Output<gpio::OpenDrain>>,
    pub watchdog_done: gpio::PE0<gpio::Output<gpio::PushPull>>,
    pub adc: adc::ADC,
    pub vref: adc::Vref,
    pub delay: delay::DelayCM,
}

struct GpioBankParts {
    gpioa: gpio::gpioa::Parts,
    gpiob: gpio::gpiob::Parts,
    gpioc: gpio::gpioc::Parts,
    gpiod: gpio::gpiod::Parts,
    gpioe: gpio::gpioe::Parts,
    gpiof: gpio::gpiof::Parts,
}

pub struct DigitalPins {
    pub temp_sensor_power: gpio::PE7<gpio::Output<gpio::PushPull>>,
}

pub struct AnalogPins {
    pub th1: gpio::PC2<gpio::Analog>,
    pub th2: gpio::PC3<gpio::Analog>,
    pub th3: gpio::PA0<gpio::Analog>,
    pub th4: gpio::PA1<gpio::Analog>,
    pub th5: gpio::PA2<gpio::Analog>,
    pub th6: gpio::PA3<gpio::Analog>,
    pub th7: gpio::PA4<gpio::Analog>,
    pub th8: gpio::PA5<gpio::Analog>,
}

#[derive(core::default::Default)]
pub struct RailState {
    pub rail1: bool,
    pub rail2: bool,
    pub rail3: bool,
    pub rail4: bool,
    pub rail5: bool,
    pub rail6: bool,
    pub rail7: bool,
    pub rail8: bool,
    pub rail9: bool,
    pub rail10: bool,
    pub rail11: bool,
    pub rail12: bool,
    pub rail13: bool,
    pub rail14: bool,
    pub rail15: bool,
    pub rail16: bool,
    pub hpwr1: bool,
    pub hpwr2: bool,
}

impl AVI {
    pub fn init(device: hal::stm32::Peripherals) -> AVI {
        // Constrain some device peripherials so we can setup the clock config below
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut pwr = device.PWR.constrain(&mut rcc.apb1r1);

        // Setup clock config
        let rcc_cfgr = rcc
            .cfgr
            .lsi(true)
            //.lsi(false)
            //.lse(
            //    hal::rcc::CrystalBypass::Enable,
            //    hal::rcc::ClockSecuritySystem::Disable,
            //)
            .hsi48(false)
            .msi(hal::rcc::MsiFreq::RANGE4M)
            //.sysclk(4.mhz());
            .hclk(4.mhz())
            .pclk1(4.mhz())
            .pclk2(4.mhz());

        // Set the USART3 clock to be sourced from the LSE clock
        //let rcc_reg = unsafe { &*hal::stm32::RCC::ptr() };
        //unsafe {
        //    rcc_reg.ccipr.write(|w| w.usart3sel().bits(0b11));
        //}

        // Freeze the clock configuration
        let clocks = rcc_cfgr.freeze(&mut flash.acr, &mut pwr);

        // Must use DelayCM here
        // Can't use SYSTIC because RTIC already consumes SYSTIC
        let mut delay = delay::DelayCM::new(clocks);
        let mut adc = adc::ADC::new(
            device.ADC1,
            device.ADC_COMMON,
            &mut rcc.ahb2,
            &mut rcc.ccipr,
            &mut delay,
        );

        let vref = adc.enable_vref(&mut delay);

        // Grab handles to GPIO banks
        let mut bank = GpioBankParts {
            gpioa: device.GPIOA.split(&mut rcc.ahb2),
            gpiob: device.GPIOB.split(&mut rcc.ahb2),
            gpioc: device.GPIOC.split(&mut rcc.ahb2),
            gpiod: device.GPIOD.split(&mut rcc.ahb2),
            gpioe: device.GPIOE.split(&mut rcc.ahb2),
            gpiof: device.GPIOF.split(&mut rcc.ahb2),
        };

        let led1 = bank
            .gpiof
            .pf2
            .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper);
        let led2 = bank
            .gpiof
            .pf3
            .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper);
        let led3 = bank
            .gpiof
            .pf4
            .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper);
        let led4 = bank
            .gpiof
            .pf5
            .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper);
        let led5 = bank
            .gpiof
            .pf6
            .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper);
        let watchdog_done = bank
            .gpioe
            .pe0
            .into_push_pull_output(&mut bank.gpioe.moder, &mut bank.gpioe.otyper);

        let digital_pins = DigitalPins {
            temp_sensor_power: bank.gpioe.pe7.into_push_pull_output_with_state(
                &mut bank.gpioe.moder,
                &mut bank.gpioe.otyper,
                gpio::State::Low,
            ),
        };

        let analog_pins = AnalogPins {
            th1: bank
                .gpioc
                .pc2
                .into_analog(&mut bank.gpioc.moder, &mut bank.gpioc.pupdr),
            th2: bank
                .gpioc
                .pc3
                .into_analog(&mut bank.gpioc.moder, &mut bank.gpioc.pupdr),
            th3: bank
                .gpioa
                .pa0
                .into_analog(&mut bank.gpioa.moder, &mut bank.gpioa.pupdr),
            th4: bank
                .gpioa
                .pa1
                .into_analog(&mut bank.gpioa.moder, &mut bank.gpioa.pupdr),
            th5: bank
                .gpioa
                .pa2
                .into_analog(&mut bank.gpioa.moder, &mut bank.gpioa.pupdr),
            th6: bank
                .gpioa
                .pa3
                .into_analog(&mut bank.gpioa.moder, &mut bank.gpioa.pupdr),
            th7: bank
                .gpioa
                .pa4
                .into_analog(&mut bank.gpioa.moder, &mut bank.gpioa.pupdr),
            th8: bank
                .gpioa
                .pa5
                .into_analog(&mut bank.gpioa.moder, &mut bank.gpioa.pupdr),
        };

        let debug_tx_pin = bank
            .gpiod
            .pd5
            .into_af7(&mut bank.gpiod.moder, &mut bank.gpiod.afrl);
        let debug_rx_pin = bank
            .gpiod
            .pd6
            .into_af7(&mut bank.gpiod.moder, &mut bank.gpiod.afrl);

        // Setup the Serial abstraction for the debug interface
        let mut debug_serial = Serial::usart2(
            device.USART2,
            (debug_tx_pin, debug_rx_pin),
            serial::Config::default().baudrate(9_600.bps()),
            clocks,
            &mut rcc.apb1r1,
        );
        debug_serial.listen(serial::Event::Rxne);
        // Create the tx & rx handles
        let (debug_tx, debug_rx) = debug_serial.split();

        // Setup the Serial abstraction for the EPS interface
        let conn_tx_pin = bank
            .gpioc
            .pc10
            .into_af8(&mut bank.gpioc.moder, &mut bank.gpioc.afrh);
        let conn_rx_pin = bank
            .gpioc
            .pc11
            .into_af8(&mut bank.gpioc.moder, &mut bank.gpioc.afrh);
        let mut conn_serial = Serial::uart4(
            device.UART4,
            (conn_tx_pin, conn_rx_pin),
            serial::Config::default().baudrate(9_600.bps()),
            clocks,
            &mut rcc.apb1r1,
        );
        conn_serial.listen(serial::Event::Rxne);
        // Create the tx & rx handles
        let (conn_tx, conn_rx) = conn_serial.split();

        // Setup the serial abstraction for the radio interface
        let radio_tx_pin = bank
            .gpioc
            .pc4
            .into_af7(&mut bank.gpioc.moder, &mut bank.gpioc.afrl);
        let radio_rx_pin = bank
            .gpioc
            .pc5
            .into_af7(&mut bank.gpioc.moder, &mut bank.gpioc.afrl);
        let mut radio_serial = Serial::usart3(
            device.USART3,
            (radio_tx_pin, radio_rx_pin),
            serial::Config::default().baudrate(115_200.bps()),
            clocks,
            &mut rcc.apb1r1,
        );
        radio_serial.listen(serial::Event::Rxne);
        // Create the tx & rx handles
        let (radio_tx, radio_rx) = radio_serial.split();

        AVI {
            //device,
            //parts: bank,
            digital_pins,
            analog_pins,
            debug_rx,
            debug_tx,
            conn_rx,
            conn_tx,
            radio_rx,
            radio_tx,
            led1,
            led2,
            led3,
            led4,
            led5,
            watchdog_done,
            adc,
            vref,
            delay,
        }
    }
}
