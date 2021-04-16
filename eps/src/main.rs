//#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

extern crate arrayvec;
extern crate cortex_m;
extern crate nb;
extern crate rtic;
extern crate stm32l4xx_hal as hal;
extern crate alloc;

use cortex_m_semihosting as _;
use hal::{adc, delay, gpio, prelude::*, serial, stm32::UART4, stm32::USART3};
use panic_semihosting as _;

use arrayvec::ArrayString;
use heapless::{Vec, consts::*, spsc};
use nb::block;
use rtic::cyccnt::{Instant, U32Ext as _};

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

pub mod protos;
use protos::no_std::EpsCommand;
extern crate quick_protobuf;
use quick_protobuf::{deserialize_from_slice, serialize_into_slice, MessageWrite};

mod eps;
use eps::BMState;
use eps::BVState;

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
const BATTERY_VOTLAGE_LOWER_LIMIT: u16 = (BATTERY_VOLTAGE_LOWER_LIMIT_VOLTS * BATTERY_ADC_VOLTS_TO_COUNTS) as u16;
// Low battery voltage hysteresis
const LOW_BATTERY_HYSTERESIS_VOLTS: f32 = 0.1;
const LOW_BATTERY_HYSTERESIS: u16 =
    (LOW_BATTERY_HYSTERESIS_VOLTS / ADC_MAX_VOLTAGE * ADC_NUM_STEPS) as u16;

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
        analog_pins: eps::AnalogPins,
        digital_pins: eps::DigitalPins,
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
        let size = 10*1024; // in bytes
        unsafe { ALLOCATOR.init(start, size) }


        // Cortex-M peripherals
        let mut core: rtic::Peripherals = cx.core;
        // Setup (initialize) the monotonic timer CYCCNT
        core.DWT.enable_cycle_counter();

        //cortex_m::asm::bkpt();

        // Device specific peripherals
        let device: hal::stm32::Peripherals = cx.device;

        // Do the bulk of our initilization
        let mut eps = eps::EPS::init(device);

        // turn off led2
        eps.led2.set_high().ok();
        // turn off led3
        eps.led3.set_high().ok();
        // turn off led4
        eps.led4.set_high().ok();

        // Send a string over the debug UART
        let sent = b"\n\rSTART\n\r";
        for elem in sent {
            block!(eps.debug_tx.write(*elem)).ok();
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
            vref: eps.vref,
            adc: eps.adc,
            analog_pins: eps.analog_pins,
            digital_pins: eps.digital_pins,
            delay: eps.delay,
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

        const BATTERY_HISTORY_LEN: usize = 10;
        let mut battery1_history: [u16; BATTERY_HISTORY_LEN] = [0; BATTERY_HISTORY_LEN];
        let mut battery2_history: [u16; BATTERY_HISTORY_LEN] = [0; BATTERY_HISTORY_LEN];
        let mut battery1_idx: usize = 0;
        let mut battery2_idx: usize = 0;
        let mut battery_voltage_state: BVState = BVState::BothLow;
        let mut battery_manager_state: (BMState, BMState) =
            (BMState::Suspended, BMState::Suspended);

        loop {
            //
            // 1)
            // Measure state & save it somewhere
            adc.calibrate(vref);
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

            //
            // 3)
            // battery watchdog (turn off things if battery voltage too low)
            // Calculate battery 1 average voltage
            battery1_history[battery1_idx] = battery1;
            battery1_idx = (battery1_idx + 1) % BATTERY_HISTORY_LEN;
            let battery1_avg: u16 =
                (battery1_history.iter().sum::<u16>() as f32 / BATTERY_HISTORY_LEN as f32) as u16;
            // Calculate battery 2 average voltage
            battery2_history[battery2_idx] = battery2;
            battery2_idx = (battery2_idx + 1) % BATTERY_HISTORY_LEN;
            let battery2_avg: u16 =
                (battery2_history.iter().sum::<u16>() as f32 / BATTERY_HISTORY_LEN as f32) as u16;
            // Calculate next state
            let next_battery_voltage_state = run_battery_voltage_state_machine(
                &battery_voltage_state,
                battery1_avg,
                battery2_avg,
            );
            // Apply change (if any)
            apply_battery_voltage_state_machine(
                &next_battery_voltage_state,
                &battery_voltage_state,
                digital_pins,
                delay,
            );
            // Increment state
            battery_voltage_state = next_battery_voltage_state;

            //
            // 4)
            // Process & respond to commands
            //let mut test_str_buffer = ArrayString::<256>::new();
            //core::fmt::write(
            //    &mut test_str_buffer,
            //    format_args!(
            //        "b1: {:.2} b2: {:.2} pg_s: {} state: {:?} bvstate: {:?}\n\r  s1:{} s2:{} s3:{} s4:{} s5:{} s6:{}\n\r",
            //        battery1 as f32 *BATTERY_ADC_COUNTS_TO_VOLTS, battery2 as f32 *BATTERY_ADC_COUNTS_TO_VOLTS, 
            //        pg_solar, battery_manager_state, battery_voltage_state, solar1, solar2, solar3, solar4, solar5, solar6 
            //    ),
            //)
            //.unwrap();

            //// Write string buffer out to UART
            //for c in test_str_buffer.as_str().bytes() {
            //    block!(tx.write(c)).unwrap();
            //}

            // This is echo code
            //while let Some(b) = rx_queue.dequeue() {
            //    block!(tx.write(b)).unwrap();
            //}

            while let Some(b) = rx_queue.dequeue() {
                let mut tmp_buf = [0u8; 1024];
                serialize_into_slice(&b, &mut tmp_buf).ok();
                for x in (0..(b.get_size() + 1)){
                    block!(tx.write( tmp_buf[x])).unwrap();

                }
                //for elem in &tmp_buf {
                //    block!(tx.write(tmp_buf.len() as u8)).unwrap();
                //    block!(tx.write(*elem)).unwrap();
                //}
            }

            //
            // 5)
            // Pet watchdog
            pet_watchdog(cx.resources.watchdog_done);

            //
            // 6)
            // Sleep
            delay.delay_ms(250u32);
        }
    }


    // This task and uart_parse_active share the same priority, so they can't pre-empt each other
    #[task(binds = USART3, resources = [led5, debug_rx, rx_prod, uart_parse_active, uart_parse_vec], schedule = [uart_buffer_clear], priority = 3)]
    fn usart3(cx: usart3::Context) {

        let rx = cx.resources.debug_rx;
        let queue = cx.resources.rx_prod;
        let uart_parse_active = cx.resources.uart_parse_active;
        let uart_parse_vec = cx.resources.uart_parse_vec;
        match rx.read() {
            Ok(b) => {
                // push byte onto vector queue
                // Nothing to do here if there if the buffer is full
                uart_parse_vec.push(b).ok();
            },
            _ => {}
        };

        match deserialize_from_slice(&uart_parse_vec) {
            Ok(parsed_cmd) => {
                // nothing to do here on error
                queue.enqueue(parsed_cmd).ok();
            },
            Err(_) => {/* Do nothing on error*/}
        }


        // Schedule the buffer clear activity (if it isn't already running)
        if *uart_parse_active == false {
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

fn run_battery_voltage_state_machine(
    battery_voltage_state: &BVState,
    battery1_avg: u16,
    battery2_avg: u16,
) -> BVState {
    // Determine if battery is low or not
    let b1_low = battery1_avg < BATTERY_VOTLAGE_LOWER_LIMIT;
    let b2_low = battery2_avg < BATTERY_VOTLAGE_LOWER_LIMIT;

    let b1_above_hyst = battery1_avg >= BATTERY_VOTLAGE_LOWER_LIMIT + LOW_BATTERY_HYSTERESIS;
    let b2_above_hyst = battery2_avg >= BATTERY_VOTLAGE_LOWER_LIMIT + LOW_BATTERY_HYSTERESIS;

    match battery_voltage_state {
        BVState::BothHigh => match (b1_low, b2_low) {
            (true, true) => {
                if battery1_avg > battery2_avg {
                    BVState::B1HighB2Low
                } else {
                    BVState::B1LowB2High
                }
            }
            (true, false) => BVState::B1LowB2High,
            (false, true) => BVState::B1HighB2Low,
            _ => BVState::BothHigh,
        },
        BVState::B1HighB2Low => match (b1_low, b1_above_hyst, b2_low, b2_above_hyst) {
            // B2 is high again
            (false, _, false, true) => BVState::BothHigh,
            // B1 is low, B2 is low
            (true, false, _, false) => BVState::BothLow,
            // B1 is low and B2 is High. This is probably unlikely, but jump to B1LowB2High
            (true, false, false, true) => BVState::B1LowB2High,
            // All other cases are either not possible, or are self-loops
            _ => BVState::B1HighB2Low,
        },
        BVState::B1LowB2High => match (b1_low, b1_above_hyst, b2_low, b2_above_hyst) {
            // B1 is high again
            (false, true, false, _) => BVState::BothHigh,
            // B1 is low, B2 is low
            (_, false, true, false) => BVState::BothLow,
            // B2 is low and B1 is High. This is probably unlikely, but jump to B1HighB2Low
            (false, true, true, false) => BVState::B1HighB2Low,
            // All other cases are either not possible, or are self-loops
            _ => BVState::B1LowB2High,
        },
        BVState::BothLow => match (b1_low, b1_above_hyst, b2_low, b2_above_hyst) {
            // B1 is high again, B2 still low
            (false, true, true, false) => BVState::B1HighB2Low,
            // B2 is high again, B1 still low
            (true, false, false, true) => BVState::B1LowB2High,
            // B1 is high again, B2 high again
            (false, true, false, true) => BVState::BothHigh,
            // All other cases are either not possible, or are self-loops
            _ => BVState::BothLow,
        },
    }
}

fn apply_battery_voltage_state_machine(
    next_battery_voltage_state: &BVState,
    battery_voltage_state: &BVState,
    digital_pins: &mut eps::DigitalPins,
    delay: &mut delay::DelayCM,
) {
    if next_battery_voltage_state != battery_voltage_state {
        // Enable both ideal diodes
        // Now
        match next_battery_voltage_state {
            BVState::BothHigh => {
                // Enable both ideal diodes
                digital_pins.ideal_en1.set_high().ok();
                digital_pins.ideal_en2.set_low().ok();

                // Make sure that the Avionics board STM32 is on
                // With the Avionics board on, it can turn on other rails
                digital_pins.pwr1.set_high().ok();
            }
            BVState::B1LowB2High => {
                // To avoid accidentally powering off our system,
                // Enable both ideal diodes before we change anything
                digital_pins.ideal_en1.set_high().ok();
                digital_pins.ideal_en2.set_low().ok();
                // Delay to make sure things are enabled
                delay.delay_ms(2u32);
                // Disable ideal1 en
                digital_pins.ideal_en1.set_low().ok();
                // Enable ideal2 en
                digital_pins.ideal_en2.set_low().ok();

                // Make sure that the Avionics board STM32 is on
                // With the Avionics board on, it can turn on other rails
                digital_pins.pwr1.set_high().ok();
            }
            BVState::B1HighB2Low => {
                // To avoid accidentally powering off our system,
                // Enable both ideal diodes before we change anything
                digital_pins.ideal_en1.set_high().ok();
                digital_pins.ideal_en2.set_low().ok();
                // Delay to make sure things are enabled
                delay.delay_ms(2u32);
                // Enable ideal1 en
                digital_pins.ideal_en1.set_high().ok();
                // Disable ideal2 en
                digital_pins.ideal_en2.set_high().ok();

                // Make sure that the Avionics board STM32 is on
                // With the Avionics board on, it can turn on other rails
                digital_pins.pwr1.set_high().ok();
            }
            BVState::BothLow => {
                // Do nothing to ideal diodes, our last state should have been B1HighB2Low
                // or B1LowB2High, so we just keep that state
                // We can't turn off both ideal diodes because that would reset us.

                // Set all power rails (except EPS) low
                digital_pins.pwr1.set_low().ok();
                digital_pins.pwr2.set_low().ok();
                digital_pins.pwr3.set_low().ok();
                digital_pins.pwr4.set_low().ok();
                digital_pins.pwr5.set_low().ok();
                digital_pins.pwr6.set_low().ok();
                digital_pins.pwr7.set_low().ok();
                digital_pins.pwr8.set_low().ok();
                digital_pins.pwr9.set_low().ok();
                digital_pins.pwr10.set_low().ok();
                digital_pins.pwr11.set_low().ok();
                digital_pins.pwr12.set_low().ok();
                digital_pins.pwr13.set_high().ok(); // EPS rail
                digital_pins.pwr14.set_low().ok();
                digital_pins.pwr16.set_low().ok();
                digital_pins.hpwr1.set_low().ok();
                digital_pins.hpwr2.set_low().ok();
                digital_pins.hpwr_en.set_low().ok();
            }
        }
    }
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    // uh oh
    loop {
        cortex_m::asm::bkpt();
    }
}
