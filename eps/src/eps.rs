extern crate stm32l4xx_hal as hal;
use hal::{adc, delay, gpio, prelude::*, serial, serial::Serial, stm32::UART4, stm32::USART3};

pub enum BMState {
    Suspended,
    LowPower,
    HighPower,
}

pub struct EPS {
    //device: hal::stm32::Peripherals,
    //parts: GpioBankParts,
    pub digital_pins: DigitalPins,
    pub analog_pins: AnalogPins,
    pub debug_rx: serial::Rx<USART3>,
    pub debug_tx: serial::Tx<USART3>,
    pub conn_rx: serial::Rx<UART4>,
    pub conn_tx: serial::Tx<UART4>,
    pub led1: gpio::PF2<gpio::Output<gpio::OpenDrain>>,
    pub led2: gpio::PF3<gpio::Output<gpio::OpenDrain>>,
    pub led3: gpio::PF4<gpio::Output<gpio::OpenDrain>>,
    pub led4: gpio::PF5<gpio::Output<gpio::OpenDrain>>,
    pub led5: gpio::PF6<gpio::Output<gpio::OpenDrain>>,
    pub watchdog_done: gpio::PE0<gpio::Output<gpio::PushPull>>,
    pub adc: adc::ADC,
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
    pub hpwr_en: gpio::PD12<gpio::Output<gpio::PushPull>>,
    pub ideal_en1: gpio::PD14<gpio::Output<gpio::OpenDrain>>,
    pub ideal_en2: gpio::PD15<gpio::Output<gpio::PushPull>>,

    pub btm1_chrg: gpio::PD1<gpio::Input<gpio::Floating>>,
    pub btm1_shdn: gpio::PD3<gpio::Output<gpio::PushPull>>,
    pub btm1_susp: gpio::PD7<gpio::Output<gpio::PushPull>>,
    pub btm1_pol: gpio::PD9<gpio::Input<gpio::Floating>>,
    pub btm1_hpwr: gpio::PC8<gpio::Output<gpio::PushPull>>,

    pub btm2_chrg: gpio::PD2<gpio::Input<gpio::Floating>>,
    pub btm2_shdn: gpio::PD4<gpio::Output<gpio::PushPull>>,
    pub btm2_susp: gpio::PD8<gpio::Output<gpio::PushPull>>,
    pub btm2_pol: gpio::PD10<gpio::Input<gpio::Floating>>,
    pub btm2_hpwr: gpio::PC9<gpio::Output<gpio::PushPull>>,

    pub pg_solar: gpio::PD0<gpio::Input<gpio::Floating>>,
    pub pg_3v3: gpio::PD13<gpio::Input<gpio::Floating>>,

    pub pwr1: gpio::PF0<gpio::Output<gpio::OpenDrain>>,
    pub pwr2: gpio::PF1<gpio::Output<gpio::OpenDrain>>,
    pub pwr3: gpio::PF7<gpio::Output<gpio::OpenDrain>>,
    pub pwr4: gpio::PF9<gpio::Output<gpio::OpenDrain>>,
    pub pwr5: gpio::PF11<gpio::Output<gpio::OpenDrain>>,
    pub pwr6: gpio::PF12<gpio::Output<gpio::OpenDrain>>,
    pub pwr7: gpio::PF13<gpio::Output<gpio::OpenDrain>>,
    pub pwr8: gpio::PF14<gpio::Output<gpio::OpenDrain>>,
    pub pwr9: gpio::PF15<gpio::Output<gpio::OpenDrain>>,
    pub pwr10: gpio::PE1<gpio::Output<gpio::OpenDrain>>,
    pub pwr11: gpio::PE2<gpio::Output<gpio::OpenDrain>>,
    pub pwr12: gpio::PE3<gpio::Output<gpio::OpenDrain>>,
    pub pwr13: gpio::PE4<gpio::Output<gpio::PushPull>>,
    pub pwr14: gpio::PE5<gpio::Output<gpio::OpenDrain>>,
    pub pwr15: gpio::PE6<gpio::Output<gpio::OpenDrain>>,
    pub pwr16: gpio::PE7<gpio::Output<gpio::OpenDrain>>,

    pub hpwr1: gpio::PE8<gpio::Output<gpio::OpenDrain>>,
    pub hpwr2: gpio::PE9<gpio::Output<gpio::OpenDrain>>,
}

pub struct AnalogPins {
    pub battery1: gpio::PA4<gpio::Analog>,
    pub battery2: gpio::PA5<gpio::Analog>,
    pub solar1: gpio::PC0<gpio::Analog>,
    pub solar2: gpio::PC1<gpio::Analog>,
    pub solar3: gpio::PC2<gpio::Analog>,
    pub solar4: gpio::PC3<gpio::Analog>,
    pub solar5: gpio::PA2<gpio::Analog>,
    pub solar6: gpio::PA3<gpio::Analog>,
}

impl EPS {
    pub fn init(device: hal::stm32::Peripherals) -> EPS {
        // Constrain some device peripherials so we can setup the clock config below
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut pwr = device.PWR.constrain(&mut rcc.apb1r1);

        // Setup clock config
        let rcc_cfgr = rcc
            .cfgr
            .lsi(false)
            .lse(
                hal::rcc::CrystalBypass::Enable,
                hal::rcc::ClockSecuritySystem::Disable,
            )
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
        let adc = adc::ADC::new(device.ADC1, &mut rcc.ahb2, &mut rcc.ccipr, &mut delay);

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
            hpwr_en: bank
                .gpiod
                .pd12
                .into_push_pull_output(&mut bank.gpiod.moder, &mut bank.gpiod.otyper),

            ideal_en1: bank
                .gpiod
                .pd14
                .into_open_drain_output(&mut bank.gpiod.moder, &mut bank.gpiod.otyper),
            ideal_en2: bank
                .gpiod
                .pd15
                .into_push_pull_output(&mut bank.gpiod.moder, &mut bank.gpiod.otyper),

            btm1_chrg: bank
                .gpiod
                .pd1
                .into_floating_input(&mut bank.gpiod.moder, &mut bank.gpiod.pupdr),

            btm1_shdn: bank
                .gpiod
                .pd3
                .into_push_pull_output(&mut bank.gpiod.moder, &mut bank.gpiod.otyper),

            btm1_susp: bank.gpiod.pd7.into_push_pull_output_with_state(
                &mut bank.gpiod.moder,
                &mut bank.gpiod.otyper,
                gpio::State::Low,
            ),

            btm1_pol: bank
                .gpiod
                .pd9
                .into_floating_input(&mut bank.gpiod.moder, &mut bank.gpiod.pupdr),
            btm1_hpwr: bank.gpioc.pc8.into_push_pull_output_with_state(
                &mut bank.gpioc.moder,
                &mut bank.gpioc.otyper,
                gpio::State::Low,
            ),
            btm2_chrg: bank
                .gpiod
                .pd2
                .into_floating_input(&mut bank.gpiod.moder, &mut bank.gpiod.pupdr),
            btm2_shdn: bank
                .gpiod
                .pd4
                .into_push_pull_output(&mut bank.gpiod.moder, &mut bank.gpiod.otyper),
            btm2_susp: bank.gpiod.pd8.into_push_pull_output_with_state(
                &mut bank.gpiod.moder,
                &mut bank.gpiod.otyper,
                gpio::State::Low,
            ),
            btm2_pol: bank
                .gpiod
                .pd10
                .into_floating_input(&mut bank.gpiod.moder, &mut bank.gpiod.pupdr),
            btm2_hpwr: bank.gpioc.pc9.into_push_pull_output_with_state(
                &mut bank.gpioc.moder,
                &mut bank.gpioc.otyper,
                gpio::State::Low,
            ),
            pg_solar: bank
                .gpiod
                .pd0
                .into_floating_input(&mut bank.gpiod.moder, &mut bank.gpiod.pupdr),
            pg_3v3: bank
                .gpiod
                .pd13
                .into_floating_input(&mut bank.gpiod.moder, &mut bank.gpiod.pupdr),

            pwr1: bank
                .gpiof
                .pf0
                .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper),
            pwr2: bank
                .gpiof
                .pf1
                .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper),
            pwr3: bank
                .gpiof
                .pf7
                .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper),
            pwr4: bank
                .gpiof
                .pf9
                .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper),
            pwr5: bank
                .gpiof
                .pf11
                .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper),
            pwr6: bank
                .gpiof
                .pf12
                .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper),
            pwr7: bank
                .gpiof
                .pf13
                .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper),
            pwr8: bank
                .gpiof
                .pf14
                .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper),
            pwr9: bank
                .gpiof
                .pf15
                .into_open_drain_output(&mut bank.gpiof.moder, &mut bank.gpiof.otyper),
            pwr10: bank
                .gpioe
                .pe1
                .into_open_drain_output(&mut bank.gpioe.moder, &mut bank.gpioe.otyper),
            pwr11: bank
                .gpioe
                .pe2
                .into_open_drain_output(&mut bank.gpioe.moder, &mut bank.gpioe.otyper),
            pwr12: bank
                .gpioe
                .pe3
                .into_open_drain_output(&mut bank.gpioe.moder, &mut bank.gpioe.otyper),

            // Must be set to high by default, since this controls the power to this chip.
            // High==enable power
            pwr13: bank.gpioe.pe4.into_push_pull_output_with_state(
                &mut bank.gpioe.moder,
                &mut bank.gpioe.otyper,
                gpio::State::High,
            ),
            pwr14: bank
                .gpioe
                .pe5
                .into_open_drain_output(&mut bank.gpioe.moder, &mut bank.gpioe.otyper),
            pwr15: bank
                .gpioe
                .pe6
                .into_open_drain_output(&mut bank.gpioe.moder, &mut bank.gpioe.otyper),
            pwr16: bank
                .gpioe
                .pe7
                .into_open_drain_output(&mut bank.gpioe.moder, &mut bank.gpioe.otyper),

            hpwr1: bank
                .gpioe
                .pe8
                .into_open_drain_output(&mut bank.gpioe.moder, &mut bank.gpioe.otyper),
            hpwr2: bank
                .gpioe
                .pe9
                .into_open_drain_output(&mut bank.gpioe.moder, &mut bank.gpioe.otyper),
        };
        //let mut analog_pins = init_analog_pins(&mut bank);

        let analog_pins = AnalogPins {
            battery1: bank
                .gpioa
                .pa4
                .into_analog(&mut bank.gpioa.moder, &mut bank.gpioa.pupdr),
            battery2: bank
                .gpioa
                .pa5
                .into_analog(&mut bank.gpioa.moder, &mut bank.gpioa.pupdr),
            solar1: bank
                .gpioc
                .pc0
                .into_analog(&mut bank.gpioc.moder, &mut bank.gpioc.pupdr),
            solar2: bank
                .gpioc
                .pc1
                .into_analog(&mut bank.gpioc.moder, &mut bank.gpioc.pupdr),
            solar3: bank
                .gpioc
                .pc2
                .into_analog(&mut bank.gpioc.moder, &mut bank.gpioc.pupdr),
            solar4: bank
                .gpioc
                .pc3
                .into_analog(&mut bank.gpioc.moder, &mut bank.gpioc.pupdr),
            solar5: bank
                .gpioa
                .pa2
                .into_analog(&mut bank.gpioa.moder, &mut bank.gpioa.pupdr),
            solar6: bank
                .gpioa
                .pa3
                .into_analog(&mut bank.gpioa.moder, &mut bank.gpioa.pupdr),
        };
        //let (debug_tx_pin, debug_rx_pin) = init_debug_serial_pins(&mut bank);
        let debug_tx_pin = bank
            .gpiob
            .pb10
            .into_af7(&mut bank.gpiob.moder, &mut bank.gpiob.afrh);
        let debug_rx_pin = bank
            .gpiob
            .pb11
            .into_af7(&mut bank.gpiob.moder, &mut bank.gpiob.afrh);

        // Setup the Serial abstraction for the debug interface
        let mut debug_serial = Serial::usart3(
            device.USART3,
            (debug_tx_pin, debug_rx_pin),
            serial::Config::default().baudrate(9_600.bps()),
            clocks,
            &mut rcc.apb1r1,
        );
        debug_serial.listen(serial::Event::Rxne);
        // Create the tx & rx handles
        let (debug_tx, debug_rx) = debug_serial.split();

        //let (conn_tx_pin, conn_rx_pin) = init_conn_serial_pins(&mut bank);
        let conn_tx_pin = bank
            .gpioa
            .pa0
            .into_af8(&mut bank.gpioa.moder, &mut bank.gpioa.afrl);
        let conn_rx_pin = bank
            .gpioa
            .pa1
            .into_af8(&mut bank.gpioa.moder, &mut bank.gpioa.afrl);
        let mut conn_serial = Serial::uart4(
            device.UART4,
            (conn_tx_pin, conn_rx_pin),
            serial::Config::default()
                .baudrate(2_400.bps())
                .oversampling(serial::Oversampling::Over8),
            clocks,
            &mut rcc.apb1r1,
        );
        conn_serial.listen(serial::Event::Rxne);
        // Create the tx & rx handles
        let (conn_tx, conn_rx) = conn_serial.split();

        EPS {
            //device,
            //parts: bank,
            digital_pins,
            analog_pins,
            debug_rx,
            debug_tx,
            conn_rx,
            conn_tx,
            led1,
            led2,
            led3,
            led4,
            led5,
            watchdog_done,
            adc,
        }
    }
}
