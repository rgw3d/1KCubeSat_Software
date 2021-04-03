//#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate arrayvec;
extern crate cortex_m;
extern crate nb;
extern crate rtic;
extern crate stm32l4xx_hal as hal;

use cortex_m_semihosting as _;
use hal::{adc, gpio, prelude::*, serial, stm32::UART4, stm32::USART3};
use panic_semihosting as _;

use arrayvec::ArrayString;
use heapless::{consts::U8, spsc};
use nb::block;
use rtic::cyccnt::U32Ext as _;

mod eps;
use eps::BMState;

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
        adc: adc::ADC,
        analog_pins: eps::AnalogPins,
        digital_pins: eps::DigitalPins,
    }

    #[init (schedule = [blinker], spawn = [blinker])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut RX_QUEUE: spsc::Queue<u8, U8> = spsc::Queue(heapless::i::Queue::new());

        // Cortex-M peripherals
        let mut core: rtic::Peripherals = cx.core;
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
            adc: eps.adc,
            analog_pins: eps.analog_pins,
            digital_pins: eps.digital_pins,
        }
    }

    #[idle(resources = [adc, analog_pins, debug_tx, rx_cons, watchdog_done, digital_pins, led3])]
    fn idle(cx: idle::Context) -> ! {
        // Pull variables from the Context struct for conveinece.
        let rx_queue = cx.resources.rx_cons;
        let tx = cx.resources.debug_tx;
        let adc = cx.resources.adc;
        let analog_pins = cx.resources.analog_pins;
        let digital_pins = cx.resources.digital_pins;
        let led3 = cx.resources.led3;

        const BATTERY_VOLTAGE_LOWER_LIMIT_VOLTS: f32 = 3.0;
        const BATTERY_DIVIDER_RATIO: f32 = 0.6;
        const ADC_NUM_STEPS: f32 = 4096.0;
        const BATTERY_VOTLAGE_LOWER_LIMIT: u16 =
            (BATTERY_VOLTAGE_LOWER_LIMIT_VOLTS * BATTERY_DIVIDER_RATIO * ADC_NUM_STEPS) as u16;

        let mut battery_manager_state: (BMState, BMState) =
            (BMState::Suspended, BMState::Suspended);

        loop {
            // 1)
            // Measure state & save it somewhere
            let battery1 = adc.read(&mut analog_pins.battery1).unwrap();
            let battery2 = adc.read(&mut analog_pins.battery2).unwrap();
            let _solar1 = adc.read(&mut analog_pins.solar1).unwrap();
            let _solar2 = adc.read(&mut analog_pins.solar2).unwrap();
            let _solar3 = adc.read(&mut analog_pins.solar3).unwrap();
            let _solar4 = adc.read(&mut analog_pins.solar4).unwrap();
            let _solar5 = adc.read(&mut analog_pins.solar5).unwrap();
            let _solar6 = adc.read(&mut analog_pins.solar6).unwrap();
            let pg_solar = digital_pins.pg_solar.is_high().unwrap();
            let _pg_3v3 = digital_pins.pg_3v3.is_high().unwrap();

            // 2)
            // Run Battery manager state machine
            run_battery_manager_state_machine(
                &mut battery_manager_state,
                pg_solar,
                battery1,
                battery2,
            );
            apply_battery_manager_state_machine(
                &mut battery_manager_state.0,
                &mut digital_pins.btm1_susp,
                &mut digital_pins.btm1_hpwr,
            );
            apply_battery_manager_state_machine(
                &mut battery_manager_state.1,
                &mut digital_pins.btm2_susp,
                &mut digital_pins.btm2_hpwr,
            );

            // 3)
            // battery watchdog (turn off things if battery voltage too low)
            if battery1 < BATTERY_VOTLAGE_LOWER_LIMIT {
                // turn on led3 as a warning
                led3.set_low().ok();
            }

            // 4)
            // Process & respond to commands
            // let mut test_str_buffer = ArrayString::<128>::new();
            // core::fmt::write(
            //     &mut test_str_buffer,
            //     format_args!("battery1: {}\n\r", battery1),
            // )
            // .unwrap();

            // // Write string buffer out to UART
            // for c in test_str_buffer.as_str().bytes() {
            //     block!(tx.write(c)).unwrap();
            // }

            // This is echo code
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
fn pet_watchdog(pin: &mut impl OutputPin) {
    // Pet the watchdog with a low to high transition
    pin.set_high().ok();
    pin.set_low().ok();
}

fn run_battery_manager_state_machine(
    state: &mut (BMState, BMState),
    pg_solar: bool,
    battery1: u16,
    battery2: u16,
) {
    match state {
        (BMState::Suspended, BMState::Suspended) => {
            if pg_solar {
                // Give lower-voltage battery preference
                if battery1 > battery2 {
                    state.1 = BMState::LowPower;
                } else {
                    state.0 = BMState::LowPower;
                }
            }
        }
        (BMState::LowPower, BMState::Suspended) => {
            if pg_solar {
                state.1 = BMState::LowPower;
            } else {
                state.0 = BMState::Suspended;
            }
        }
        (BMState::Suspended, BMState::LowPower) => {
            if pg_solar {
                state.0 = BMState::LowPower;
            } else {
                state.1 = BMState::Suspended;
            }
        }
        (BMState::LowPower, BMState::LowPower) => {
            if pg_solar {
                // Give lower-voltage battery preference
                if battery1 > battery2 {
                    state.1 = BMState::HighPower;
                } else {
                    state.0 = BMState::HighPower;
                }
            } else {
                // Give lower-voltage battery preference
                if battery1 > battery2 {
                    state.0 = BMState::Suspended;
                } else {
                    state.1 = BMState::Suspended;
                }
            }
        }
        (BMState::HighPower, BMState::LowPower) => {
            if pg_solar {
                state.1 = BMState::HighPower;
            } else {
                state.0 = BMState::LowPower;
            }
        }
        (BMState::LowPower, BMState::HighPower) => {
            if pg_solar {
                state.0 = BMState::HighPower;
            } else {
                state.1 = BMState::LowPower;
            }
        }
        (BMState::HighPower, BMState::HighPower) => {
            if !pg_solar {
                if battery1 > battery2 {
                    state.0 = BMState::LowPower;
                } else {
                    state.1 = BMState::LowPower;
                }
            }
        }
        _ => {
            // This should be impossible to reach
            cortex_m::asm::bkpt();
        }
    }
}

fn apply_battery_manager_state_machine(
    state: &mut eps::BMState,
    btm_susp: &mut impl OutputPin,
    btm_hpwr: &mut impl OutputPin,
) {
    match state {
        eps::BMState::Suspended => {
            // Suspend the chip
            btm_susp.set_high().ok();
            // Disable high power (is redundant since input is suspended)
            btm_hpwr.set_low().ok();
        }
        eps::BMState::LowPower => {
            // Suspend the chip
            btm_susp.set_low().ok();
            // Disable high power
            btm_hpwr.set_low().ok();
        }
        eps::BMState::HighPower => {
            // Suspend the chip
            btm_susp.set_low().ok();
            // Disable high power
            btm_hpwr.set_low().ok();
        }
    }
}
