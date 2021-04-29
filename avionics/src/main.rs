//#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

extern crate alloc;
extern crate arrayvec;
extern crate cortex_m;
extern crate nb;
extern crate quick_protobuf;
extern crate rtic;
extern crate stm32l4xx_hal as hal;

use alloc_cortex_m::CortexMHeap;
use arrayvec::ArrayString;
use core::alloc::Layout;
use core::convert::Infallible;
use cortex_m_semihosting as _;
use hal::{adc, delay, gpio, prelude::*, serial, stm32::UART4, stm32::USART3};
use heapless::{consts::*, spsc, Vec};
use nb::block;
use panic_semihosting as _;
use rtic::cyccnt::{Instant, U32Ext as _};
pub mod protos;
use protos::no_std::{
    mod_EpsResponse, BatteryManagerState, BatteryManagerStates, BatteryVoltage,
    BatteryVoltageState, CommandID, EpsCommand, EpsResponse, RailState, SolarVoltage,
};
use quick_protobuf::{deserialize_from_slice, serialize_into_slice, MessageWrite};
mod avi;
use avi::{BMState, BVState, PowerRails};

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

const BLINK_PERIOD: u32 = 8_000_000;
const UART_PARSE_PERIOD: u32 = 1_000_000;
// ADC constants
const ADC_NUM_STEPS: f32 = 4096.0;
const ADC_MAX_VOLTAGE: f32 = 3.3;
// Calculate Low battery voltage limit
const BATTERY_VOLTAGE_LOWER_LIMIT_VOLTS: f32 = 3.0;
const BATTERY_DIVIDER_RATIO: f32 = 0.6;
const BATTERY_ADC_VOLTS_TO_COUNTS: f32 = BATTERY_DIVIDER_RATIO / ADC_MAX_VOLTAGE * ADC_NUM_STEPS;
const BATTERY_ADC_COUNTS_TO_VOLTS: f32 = 1.0 / BATTERY_ADC_VOLTS_TO_COUNTS;
const BATTERY_VOTLAGE_LOWER_LIMIT: u16 =
    (BATTERY_VOLTAGE_LOWER_LIMIT_VOLTS * BATTERY_ADC_VOLTS_TO_COUNTS) as u16;
// Low battery voltage hysteresis
const LOW_BATTERY_HYSTERESIS_VOLTS: f32 = 0.1;
const LOW_BATTERY_HYSTERESIS: u16 =
    (LOW_BATTERY_HYSTERESIS_VOLTS / ADC_MAX_VOLTAGE * ADC_NUM_STEPS) as u16;
const NUM_POWER_RAILS: usize = 19;

#[rtic::app(device = hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        rx_prod: spsc::Producer<'static, EpsCommand, U8>,
        rx_cons: spsc::Consumer<'static, EpsCommand, U8>,
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
        vref: adc::Vref,
        analog_pins: avi::AnalogPins,
        digital_pins: avi::DigitalPins,
        delay: delay::DelayCM,
        uart_parse_active: bool,
        uart_parse_vec: &'static mut Vec<u8, U1024>,
    }

    #[init (schedule = [blinker, uart_buffer_clear], spawn = [blinker])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut RX_QUEUE: spsc::Queue<EpsCommand, U8> = spsc::Queue(heapless::i::Queue::new());
        static mut UART_PARSE_VEC: Vec<u8, U1024> = Vec(heapless::i::Vec::new());

        // Setup the Allocator
        // On the STM32L496 platform, there is 320K of RAM (shared by the stack)
        let start = cortex_m_rt::heap_start() as usize;
        let size = 10 * 1024; // in bytes
        unsafe { ALLOCATOR.init(start, size) }

        // Cortex-M peripherals
        let mut core: rtic::Peripherals = cx.core;
        // Setup (initialize) the monotonic timer CYCCNT
        core.DWT.enable_cycle_counter();

        //cortex_m::asm::bkpt();

        // Device specific peripherals
        let device: hal::stm32::Peripherals = cx.device;

        // Do the bulk of our initilization
        let mut avi = avi::AVI::init(device);

        // turn off led2
        avi.led2.set_high().ok();
        // turn off led3
        avi.led3.set_high().ok();
        // turn off led4
        avi.led4.set_high().ok();

        // Send a string over the debug UART
        let sent = b"\n\rSTART\n\r";
        for elem in sent {
            block!(avi.debug_tx.write(*elem)).ok();
        }

        // Create the producer and consumer sides of the Queue
        let (rx_prod, rx_cons) = RX_QUEUE.split();

        // Schedule the blinking LED function
        cx.schedule
            .blinker(cx.start + BLINK_PERIOD.cycles())
            .unwrap();

        // Schedule the Uart clear buffer function (only runs once at start)
        cx.schedule
            .uart_buffer_clear(cx.start + UART_PARSE_PERIOD.cycles())
            .unwrap();

        init::LateResources {
            rx_prod,
            rx_cons,
            debug_rx: avi.debug_rx,
            debug_tx: avi.debug_tx,
            conn_rx: avi.conn_rx,
            conn_tx: avi.conn_tx,
            led1: avi.led1,
            led2: avi.led2,
            led3: avi.led3,
            led4: avi.led4,
            led5: avi.led5,
            watchdog_done: avi.watchdog_done,
            vref: avi.vref,
            adc: avi.adc,
            analog_pins: avi.analog_pins,
            digital_pins: avi.digital_pins,
            delay: avi.delay,
            uart_parse_active: true, // true because we've scheduled the buffer clear function above
            uart_parse_vec: UART_PARSE_VEC,
        }
    }

    #[idle(resources = [adc, vref, analog_pins, debug_tx, rx_cons, watchdog_done, digital_pins, led3, delay])]
    fn idle(cx: idle::Context) -> ! {
        // Pull variables from the Context struct for conveinece.
        let rx_queue = cx.resources.rx_cons;
        let tx = cx.resources.debug_tx;
        let adc = cx.resources.adc;
        let analog_pins = cx.resources.analog_pins;
        let digital_pins = cx.resources.digital_pins;
        let led3 = cx.resources.led3;
        let vref = cx.resources.vref;
        let delay = cx.resources.delay;

        loop {
            //
            // 1)
            // Measure state & save it somewhere
            //adc.calibrate(vref);
            adc.set_sample_time(adc::SampleTime::Cycles47_5);
            let battery1 = adc.read(&mut analog_pins.battery1).unwrap();
            let battery2 = adc.read(&mut analog_pins.battery2).unwrap();
            let solar1 = adc.read(&mut analog_pins.solar1).unwrap();
            let solar2 = adc.read(&mut analog_pins.solar2).unwrap();
            let solar3 = adc.read(&mut analog_pins.solar3).unwrap();
            let solar4 = adc.read(&mut analog_pins.solar4).unwrap();
            let solar5 = adc.read(&mut analog_pins.solar5).unwrap();
            let solar6 = adc.read(&mut analog_pins.solar6).unwrap();

            let pg_solar = digital_pins.pg_solar.is_high().unwrap();
            let _pg_3v3 = digital_pins.pg_3v3.is_high().unwrap();

            //
            // 2)
            // Run Battery manager state machine

            //
            // 3)

            //
            // 4)
            // Loop over any responses we may have recieved and update state
            while let Some(eps_response) = rx_queue.dequeue() {
                match eps_response.cid {
                    CommandID::SetPowerRailState => {}
                    CommandID::GetPowerRailState => {}
                    CommandID::GetBatteryVoltage => {}
                    CommandID::GetSolarVoltage => {}
                    CommandID::GetBatteryVoltageState => {}
                    CommandID::GetBatteryManagerState => {}
                };
            }

            //
            // 5)
            // Pet watchdog
            pet_watchdog(cx.resources.watchdog_done);

            //
            // 6)
            // Sleep
            delay.delay_ms(20u32);
        }
    }

    // This task and uart_parse_active share the same priority, so they can't pre-empt each other
    #[task(binds = USART3, resources = [led5, debug_rx, rx_prod, uart_parse_active, uart_parse_vec], schedule = [uart_buffer_clear], priority = 3)]
    fn usart3(cx: usart3::Context) {
        let rx = cx.resources.debug_rx;
        let queue = cx.resources.rx_prod;
        let uart_parse_active = cx.resources.uart_parse_active;
        let uart_parse_vec = cx.resources.uart_parse_vec;
        if let Ok(b) = rx.read() {
            // push byte onto vector queue
            uart_parse_vec.push(b).ok();
        };

        if let Ok(parsed_cmd) = deserialize_from_slice(&uart_parse_vec) {
            queue.enqueue(parsed_cmd).ok();
        }

        // Schedule the buffer clear activity (if it isn't already running)
        if !(*uart_parse_active) {
            *uart_parse_active = true;
            cx.resources.led5.set_low().ok();
            cx.schedule
                .uart_buffer_clear(Instant::now() + UART_PARSE_PERIOD.cycles())
                .unwrap();
        }
    }

    // This task and usart3 share the same priority, so they can't pre-empt each other
    #[task(resources = [led5, uart_parse_active, uart_parse_vec], priority = 3)]
    fn uart_buffer_clear(cx: uart_buffer_clear::Context) {
        let uart_parse_active = cx.resources.uart_parse_active;
        let uart_parse_vec = cx.resources.uart_parse_vec;
        let led5 = cx.resources.led5;

        // turn off LED
        led5.set_high().ok();
        // Reset bool guard
        *uart_parse_active = false;

        // Clear buffer
        uart_parse_vec.clear();
        //while let Some(elem) = uart_parse_vec.pop() {
        //    //cortex_m::asm::bkpt();
        //    let _x = elem;
        //}
    }

    #[task(resources = [led1], schedule = [blinker], priority = 1)]
    fn blinker(cx: blinker::Context) {
        static mut LED_IS_ON: bool = false;

        //cortex_m::asm::bkpt();

        if *LED_IS_ON {
            cx.resources.led1.set_low().ok();
            *LED_IS_ON = false;
        } else {
            cx.resources.led1.set_high().ok();
            *LED_IS_ON = true;
        }

        // Schedule
        cx.schedule
            .blinker(cx.scheduled + BLINK_PERIOD.cycles())
            .unwrap();
    }

    extern "C" {

        // I think any of the interupts work??
        fn LCD();
        fn SAI1();
    }
};

//fn pet_watchdog(watchdog_done: &mut WatchdogPinType) {
//fn pet_watchdog<'a>(watchdog_done: &'a mut hal::prelude::OutputPin<Error = Infallible>) {
fn pet_watchdog(pin: &mut impl OutputPin) {
    // Pet the watchdog with a low to high transition
    pin.set_high().ok();
    pin.set_low().ok();
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    // uh oh
    loop {
        cortex_m::asm::bkpt();
    }
}
