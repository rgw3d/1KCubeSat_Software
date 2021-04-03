//#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate nb;
extern crate rtic;
extern crate stm32l4xx_hal as hal;

use cortex_m_semihosting as _;
use hal::{gpio, prelude::*, serial, stm32::UART4, stm32::USART3};
use panic_semihosting as _;

use heapless::{consts::U8, spsc};
use nb::block;
use rtic::cyccnt::U32Ext as _;

mod eps;

const BLINK_PERIOD: u32 = 4_000_000;
//type WatchdogPinType<'a> = &'a mut dyn hal::prelude::OutputPin<Error = Infallible>;
//type WatchdogPinType = impl hal::prelude::outputPin<Error = Infallible>;

#[rtic::app(device = hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        rx_prod: spsc::Producer<'static, u8, U8>,
        rx_cons: spsc::Consumer<'static, u8, U8>,
        debug_rx: serial::Rx<USART3>,
        debug_tx: serial::Tx<USART3>,
        conn_rx: serial::Rx<UART4>,
        conn_tx: serial::Tx<UART4>,
        led1: gpio::PF2<gpio::Output<gpio::OpenDrain>>,
        led2: gpio::PF3<gpio::Output<gpio::OpenDrain>>,
        led3: gpio::PF4<gpio::Output<gpio::OpenDrain>>,
        led4: gpio::PF5<gpio::Output<gpio::OpenDrain>>,
        led5: gpio::PF6<gpio::Output<gpio::OpenDrain>>,
        watchdog_done: gpio::PE0<gpio::Output<gpio::PushPull>>,
    }

    #[init (schedule = [blinker], spawn = [blinker])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut RX_QUEUE: spsc::Queue<u8, U8> = spsc::Queue(heapless::i::Queue::new());

        // Cortex-M peripherals
        let mut core = cx.core;
        // Setup (initialize) the monotonic timer CYCCNT
        core.DWT.enable_cycle_counter();

        //cortex_m::asm::bkpt();

        // Device specific peripherals
        let device: hal::stm32::Peripherals = cx.device;

        // Do the bulk of our initilization
        let mut eps = eps::EPS::init(device);

        // turn on led2
        eps.led2.set_low().ok();
        // turn off led3
        eps.led3.set_high().ok();
        // turn off led4
        eps.led4.set_high().ok();

        // Send a string over the debug UART
        let sent = b"START\n\r";
        for elem in sent {
            block!(eps.debug_tx.write(*elem)).ok();
        }

        // Create the producer and consumer sides of the Queue
        let (rx_prod, rx_cons) = RX_QUEUE.split();

        // Schedule the blinking LED function
        cx.schedule
            .blinker(cx.start + BLINK_PERIOD.cycles())
            .unwrap();

        init::LateResources {
            rx_prod,
            rx_cons,
            debug_rx: eps.debug_rx,
            debug_tx: eps.debug_tx,
            conn_rx: eps.conn_rx,
            conn_tx: eps.conn_tx,
            led1: eps.led1,
            led2: eps.led2,
            led3: eps.led3,
            led4: eps.led4,
            led5: eps.led5,
            watchdog_done: eps.watchdog_done,
        }
    }

    #[idle(resources = [debug_tx, rx_cons, watchdog_done])]
    fn idle(cx: idle::Context) -> ! {
        let rx_queue = cx.resources.rx_cons;
        let tx = cx.resources.debug_tx;
        loop {
            // 1)
            // Measure state & save it somewhere

            // 2)
            // Run Battery manager state machine

            // 3)
            // battery watchdog (turn off things if battery voltage too low)

            // 4)
            // Process & respond to commands

            if let Some(b) = rx_queue.dequeue() {
                block!(tx.write(b)).unwrap();
            }

            // 5)
            // Pet watchdog
            pet_watchdog(cx.resources.watchdog_done);

            // 6)
            // Sleep
            //cortex_m::asm::delay(1000000);
        }
    }

    #[task(binds = USART3, resources = [led5,debug_rx, rx_prod], priority = 3)]
    fn usart3(cx: usart3::Context) {
        cx.resources.led5.set_high().ok();
        let rx = cx.resources.debug_rx;
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

//fn pet_watchdog(watchdog_done: &mut WatchdogPinType) {
//fn pet_watchdog<'a>(watchdog_done: &'a mut hal::prelude::OutputPin<Error = Infallible>) {
fn pet_watchdog(pin: &mut impl hal::prelude::OutputPin) {
    // Pet the watchdog with a low to high transition
    pin.set_high().ok();
    pin.set_low().ok();
}
