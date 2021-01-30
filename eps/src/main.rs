//#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate nb;
extern crate rtic;
extern crate stm32l4xx_hal as hal;

use cortex_m_semihosting as _;
use hal::{gpio, prelude::*, serial, serial::Serial, stm32::USART3};
use panic_semihosting as _;

use heapless::{consts::U8, spsc};
use nb::block;
use rtic::cyccnt::U32Ext as _;

const BLINK_PERIOD: u32 = 4_000_000;
type WatchdogPinType = gpio::PE0<gpio::Output<gpio::PushPull>>;

#[rtic::app(device = hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        rx: serial::Rx<USART3>,
        tx: serial::Tx<USART3>,

        rx_prod: spsc::Producer<'static, u8, U8>,
        rx_cons: spsc::Consumer<'static, u8, U8>,

        watchdog_done: WatchdogPinType,
        led1: gpio::PF2<gpio::Output<gpio::OpenDrain>>,
        led2: gpio::PF3<gpio::Output<gpio::OpenDrain>>,
        led3: gpio::PF4<gpio::Output<gpio::OpenDrain>>,
        led4: gpio::PF5<gpio::Output<gpio::OpenDrain>>,
        led5: gpio::PF6<gpio::Output<gpio::OpenDrain>>,
    }

    #[init (schedule = [blinker], spawn = [blinker])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut RX_QUEUE: spsc::Queue<u8, U8> = spsc::Queue(heapless::i::Queue::new());

        // Cortex-M peripherals
        let mut core = cx.core;
        // Device specific peripherals
        let device: hal::stm32::Peripherals = cx.device;

        // Setup (initialize) the monotonic timer CYCCNT
        core.DWT.enable_cycle_counter();

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

        let rcc_reg = unsafe { &*hal::stm32::RCC::ptr() };
        //let rcc_reg = unsafe { &*rcc::ptr() };
        unsafe {
            rcc_reg.ccipr.write(|w| w.usart3sel().bits(0b11));
        }

        // Freeze the clock configuration
        let clocks = rcc_cfgr.freeze(&mut flash.acr, &mut pwr);

        // Grab handles to GPIO banks
        let mut gpiob = device.GPIOB.split(&mut rcc.ahb2);
        let mut gpioe = device.GPIOE.split(&mut rcc.ahb2);
        let mut gpiof = device.GPIOF.split(&mut rcc.ahb2);

        // Configure the watchdog pin
        let mut watchdog_done = gpioe
            .pe0
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        pet_watchdog(&mut watchdog_done);

        // Configure the LED1 pin
        let mut led1 = gpiof
            .pf2
            .into_open_drain_output(&mut gpiof.moder, &mut gpiof.otyper);
        led1.set_high().ok();

        // Configure the LED2 pin
        let mut led2 = gpiof
            .pf3
            .into_open_drain_output(&mut gpiof.moder, &mut gpiof.otyper);
        led2.set_high().ok();

        // Configure the LED3 pin
        let led3 = gpiof
            .pf4
            .into_open_drain_output(&mut gpiof.moder, &mut gpiof.otyper);

        // Configure the LED4 pin
        let led4 = gpiof
            .pf5
            .into_open_drain_output(&mut gpiof.moder, &mut gpiof.otyper);

        // Configure the LED5 pin
        let mut led5 = gpiof
            .pf6
            .into_open_drain_output(&mut gpiof.moder, &mut gpiof.otyper);
        led5.set_low().ok();

        // Configure the USART3 pins
        let tx_pin = gpiob.pb10.into_af7(&mut gpiob.moder, &mut gpiob.afrh);
        let rx_pin = gpiob.pb11.into_af7(&mut gpiob.moder, &mut gpiob.afrh);

        // Setup the Serial abstraction
        let mut serial = Serial::usart3(
            device.USART3,
            (tx_pin, rx_pin),
            serial::Config::default()
                .baudrate(2_400.bps())
                .oversampling(serial::Oversampling::Over8),
            clocks,
            &mut rcc.apb1r1,
        );
        serial.listen(serial::Event::Rxne);

        // Create the tx & rx handles
        let (mut tx, rx) = serial.split();

        // Send a string over the UART
        let sent = b"START\n\r";
        for elem in sent {
            block!(tx.write(*elem)).ok();
        }

        // Create the producer and consumer sides of the Queue
        let (rx_prod, rx_cons) = RX_QUEUE.split();

        // Schedule the blinking LED function
        cx.schedule
            .blinker(cx.start + BLINK_PERIOD.cycles())
            .unwrap();

        init::LateResources {
            rx,
            tx,
            rx_prod,
            rx_cons,
            watchdog_done,
            led1,
            led2,
            led3,
            led4,
            led5,
        }
    }

    #[idle(resources = [tx, rx_cons, watchdog_done, led2, led3, led4])]
    fn idle(mut cx: idle::Context) -> ! {
        let rx_queue = cx.resources.rx_cons;
        let tx = cx.resources.tx;
        loop {
            pet_watchdog(&mut cx.resources.watchdog_done);
            //cx.resources.led1.set_low().ok();
            //cx.resources.led2.set_low().ok();
            //cx.resources.led3.set_low().ok();
            //cx.resources.led4.set_high().ok();

            //cortex_m::asm::delay(1000000);

            //cx.resources.led1.set_high().ok();
            //cx.resources.led2.set_high().ok();
            //cx.resources.led3.set_high().ok();
            //cx.resources.led4.set_low().ok();

            //cortex_m::asm::delay(1000000);

            if let Some(b) = rx_queue.dequeue() {
                block!(tx.write(b)).unwrap();
            }
        }
    }

    #[task(binds = USART3, resources = [rx, rx_prod, led5], priority = 3)]
    fn usart3(cx: usart3::Context) {
        cx.resources.led5.set_high().ok();
        let rx = cx.resources.rx;
        let queue = cx.resources.rx_prod;

        let b = match rx.read() {
            Ok(b) => b,
            Err(_err) => b'x',
        };
        match queue.enqueue(b) {
            Ok(()) => (),
            Err(_err) => {}
        }
    }

    #[task(resources = [led1], schedule = [blinker], priority = 1)]
    fn blinker(cx: blinker::Context) {
        static mut LED_STATE: bool = false;

        //cortex_m::asm::bkpt();

        if *LED_STATE == true {
            cx.resources.led1.set_low().ok();
            *LED_STATE = false;
        } else {
            cx.resources.led1.set_high().ok();
            *LED_STATE = true;
        }

        // Schedule
        cx.schedule
            .blinker(cx.scheduled + BLINK_PERIOD.cycles())
            .unwrap();
    }

    extern "C" {

        // I think any of the interupts work??
        fn LCD();
    }
};

fn pet_watchdog(watchdog_done: &mut WatchdogPinType) {
    // Pet the watchdog with a low to high transition
    watchdog_done.set_high().ok();
    watchdog_done.set_low().ok();
}

//
// //Working peripherals without any runtime. See below
//
// #![deny(unsafe_code)]
// // #![deny(warnings)]
// #![no_std]
// #![no_main]
//
// extern crate cortex_m;
// extern crate cortex_m_rt as rt;
// extern crate cortex_m_semihosting as sh;
// extern crate nb;
// extern crate panic_semihosting;
// extern crate stm32l4xx_hal as hal;
//
// use crate::hal::delay::Delay;
// use crate::hal::prelude::*;
// use crate::hal::serial::{Config, Serial};
// use crate::nb::block;
// use crate::rt::entry;
// use crate::rt::exception;
// use crate::rt::interrupt;
// use crate::rt::ExceptionFrame;
// use core::cell::RefCell;
// use core::ops::DerefMut;
// use cortex_m::{
//     interrupt::{free, Mutex},
//     peripheral::NVIC,
// };
//
// static BUTTON: Mutex<RefCell<Option<hal::gpio::gpioc::PC13<hal::gpio::Input<hal::gpio::PullUp>>>>> =
//     Mutex::new(RefCell::new(None));
//
// #[entry]
// fn main() -> ! {
//     let cp = hal::stm32::CorePeripherals::take().unwrap();
//     let dp = hal::stm32::Peripherals::take().unwrap();
//
//     let mut flash = dp.FLASH.constrain();
//     let mut rcc = dp.RCC.constrain();
//     let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
//
//     // Try a different clock configuration
//     //let clocks = rcc.cfgr.hclk(8.mhz()).freeze(&mut flash.acr, &mut pwr);
//
//     // let clocks = rcc.cfgr
//     //     .sysclk(64.mhz())
//     //     .pclk1(32.mhz())
//     //     .freeze(&mut flash.acr);
//
//     // Setup clock config
//     let rcc_cfgr = rcc
//         .cfgr
//         .lsi(false)
//         .lse(
//             hal::rcc::CrystalBypass::Enable,
//             hal::rcc::ClockSecuritySystem::Disable,
//         )
//         .hsi48(false)
//         .msi(hal::rcc::MsiFreq::RANGE4M)
//         //.sysclk(4.mhz());
//         .hclk(4.mhz())
//         .pclk1(4.mhz())
//         .pclk2(4.mhz());
//
//     let clocks = rcc_cfgr.freeze(&mut flash.acr, &mut pwr);
//     let mut timer = Delay::new(cp.SYST, clocks);
//     let test_test = clocks.sysclk();
//
//     // let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);
//     // let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.afrh);
//
//     //RTC
//     let mut rtc = hal::rtc::Rtc::rtc(
//         dp.RTC,
//         &mut rcc.apb1r1,
//         &mut rcc.bdcr,
//         &mut pwr.cr1,
//         hal::rtc::RtcConfig::default().clock_config(hal::rtc::RtcClockSource::LSE),
//     );
//
//     let time = hal::datetime::Time::new(21.hours(), 57.minutes(), 32.seconds(), 0.micros(), false);
//     let date = hal::datetime::Date::new(1.day(), 24.date(), 4.month(), 2018.year());
//
//     //cortex_m::asm::bkpt();
//     rtc.set_date_time(date, time);
//
//     //timer.delay_ms(1000_u32);
//     //timer.delay_ms(1000_u32);
//     //timer.delay_ms(1000_u32);
//
//     let (rtc_date, rtc_time) = rtc.get_date_time();
//
//     //cortex_m::asm::bkpt();
//
//     let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
//     // The Serial API is highly generic
//     // TRY the commented out, different pin configurations
//     // let tx = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
//     let tx = gpiob.pb10.into_af7(&mut gpiob.moder, &mut gpiob.afrh);
//     // let tx = gpiob.pb6.into_af7(&mut gpiob.moder, &mut gpiob.afrl);
//
//     // let rx = gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
//     let rx = gpiob.pb11.into_af7(&mut gpiob.moder, &mut gpiob.afrh);
//     // let rx = gpiob.pb7.into_af7(&mut gpiob.moder, &mut gpiob.afrl);
//
//     // TRY using a different USART peripheral here
//     let serial = Serial::usart3(
//         dp.USART3,
//         (tx, rx),
//         Config::default().baudrate(9_600.bps()),
//         clocks,
//         &mut rcc.apb1r1,
//     );
//     let (mut tx, mut rx) = serial.split();
//
//     let sent = b'X';
//
//     // The `block!` macro makes an operation block until it finishes
//     // NOTE the error type is `!`
//     block!(tx.write(sent)).ok();
//
//     let mut gpioe = dp.GPIOE.split(&mut rcc.ahb2);
//     let external_watchdog_done = gpioe
//         .pe0
//         .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
//
//     let mut gpiof = dp.GPIOF.split(&mut rcc.ahb2);
//     let led = gpiof
//         .pf2
//         .into_open_drain_output(&mut gpiof.moder, &mut gpiof.otyper);
//
//     testtest(timer, led, external_watchdog_done);
//     //loop {
//     //    // block!(timer.wait()).unwrap();
//     //    timer.delay_ms(100 as u32);
//     //    led.set_high().ok();
//     //    // block!(timer.wait()).unwrap();
//     //    timer.delay_ms(100 as u32);
//     //    led.set_low().ok();
//     //}
// }
//
// fn testtest(
//     mut timer: hal::delay::Delay,
//     mut led: hal::gpio::PF2<hal::gpio::Output<hal::gpio::OpenDrain>>,
//     mut watchdog: hal::gpio::PE0<hal::gpio::Output<hal::gpio::PushPull>>,
// ) -> ! {
//     loop {
//         // block!(timer.wait()).unwrap();
//         timer.delay_ms(100 as u32);
//         led.set_high().ok();
//
//         // Pet the watchdog with a low to high transition
//         watchdog.set_high().ok();
//         watchdog.set_low().ok();
//
//         // block!(timer.wait()).unwrap();
//         timer.delay_ms(100 as u32);
//         led.set_low().ok();
//     }
// }
//
// #[exception]
// fn HardFault(ef: &ExceptionFrame) -> ! {
//     panic!("{:#?}", ef);
// }
//
// #[interrupt]
// fn EXTI15_10() {
//     free(|cs| {
//         let mut btn_ref = BUTTON.borrow(cs).borrow_mut();
//         if let Some(ref mut btn) = btn_ref.deref_mut() {
//             if btn.check_interrupt() {
//                 // if we don't clear this bit, the ISR would trigger indefinitely
//                 btn.clear_interrupt_pending_bit();
//             }
//         }
//     });
// }

//#![no_std]
//#![no_main]
//
//// pick a panicking behavior
//use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
//// use panic_abort as _; // requires nightly
//// use panic_itm as _; // logs messages over ITM; requires ITM support
//// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
//
//use cortex_m_rt::entry;
//use stm32l4::stm32l4x6;
//use stm32l4xx_hal::{prelude::*, delay::Delay, rcc};
//
//#[entry]
//fn main() -> ! {
//    let x = 1;
//    let core_peripherals = stm32l4x6::CorePeripherals::take().unwrap();
//    let peripherals = stm32l4x6::Peripherals::take().unwrap();
//
//    let mut rcc_peripheral   = peripherals.RCC.constrain();
//    let mut flash_peripheral = peripherals.FLASH.constrain();
//    let mut pwr_peripheral   = peripherals.PWR.constrain(&mut rcc_peripheral.apb1r1);
//
//    // Setup clock config
//    let rcc_cfgr = rcc_peripheral.cfgr
//        .lsi(false)
//        .hsi48(false)
//        .msi(rcc::MsiFreq::RANGE4M)
//        .sysclk(4.mhz())
//        .hclk(4.mhz())
//        .pclk1(4.mhz())
//        .pclk2(4.mhz());
//
//    let clock_config = rcc_cfgr.freeze(&mut flash_peripheral.acr, &mut pwr_peripheral);
//
//    let mut delay = Delay::new(core_peripherals.SYST, clock_config);
//    //testpls.cfgr.lsi(true).hsi48(false);
//    // Set HSI48 to be off
//    //testpls.cfgr.hsi48(false);
//
//    //testpls.cfgr.msi(rcc::MsiFreq::RANGE4M);
//
//    //testpls.cfgr.sysclock()
//    //let mut testrcc = rcc::RCC.constrain();
//    //rcc::CFGR.sysclock(8);
//    //testrcc.lsi(true);
//    // let clock_config = rcc::PLLConfig::pll(
//    //     rcc::PLLSource::HSE(12.mhz()),
//    //     rcc::PLLMul::Mul8,
//    //     rcc::PLLDiv::Div4,
//    // );
//    // let mut rcc = peripherals.RCC.freeze(clock_config);
//
//    //let gpiof = peripherals.GPIOF.split(&mut rcc);
//    //let mut status_led = gpiof.pf2.into_
//
//    let mut gpiof = peripherals.GPIOF.split(&mut rcc_peripheral.ahb2);
//    let mut led = gpiof.pf2.into_open_drain_output(&mut gpiof.moder, &mut gpiof.otyper);
//
//    loop {
//        // your code goes here
//        led.set_low().ok();
//        delay.delay_ms(1_000_u16);
//        led.set_high().ok();
//    }
//}
//
