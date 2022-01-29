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
use hal::{adc, delay, gpio, prelude::*, serial, stm32::UART4, stm32::USART2};
use heapless::{consts::*, spsc, Vec};
use nb::block;
use panic_semihosting as _;
use rtic::cyccnt::{Instant, U32Ext as _};
pub mod protos;
use protos::no_std::{
    mod_EpsResponse::OneOfresp, mod_RadioTelemetry, mod_RadioTelemetry::OneOfmessage,
    BatteryManagerState, BatteryManagerStates, BatteryVoltage, BatteryVoltageState, CommandID,
    EpsCommand, EpsResponse, PowerRails, RadioSOH, RadioTelemetry, RailSOH, RailState,
    SolarVoltage, TelemetryID,
};
use quick_protobuf::{deserialize_from_slice, serialize_into_slice, MessageWrite};
mod avi;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

const BLINK_PERIOD: u32 = 8_000_000;
const EPS_QUERY_PERIOD: u32 = 3_000_000;
const UART_PARSE_PERIOD: u32 = 1_000_000;

#[rtic::app(device = hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        rx_prod: spsc::Producer<'static, EpsResponse, U8>,
        rx_cons: spsc::Consumer<'static, EpsResponse, U8>,
        tx_prod: spsc::Producer<'static, EpsCommand, U8>,
        tx_cons: spsc::Consumer<'static, EpsCommand, U8>,
        debug_rx: serial::Rx<USART2>,
        debug_tx: serial::Tx<USART2>,
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

    #[init (schedule = [blinker, uart_buffer_clear, eps_query], spawn = [blinker])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut RX_QUEUE: spsc::Queue<EpsResponse, U8> = spsc::Queue(heapless::i::Queue::new());
        static mut TX_QUEUE: spsc::Queue<EpsCommand, U8> = spsc::Queue(heapless::i::Queue::new());
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

        // turn on led2
        avi.led2.set_low().ok();
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

        // Create the producer and consumer sides of the Queue
        let (tx_prod, tx_cons) = TX_QUEUE.split();

        // Schedule the blinking LED function
        cx.schedule
            .blinker(cx.start + BLINK_PERIOD.cycles())
            .unwrap();

        // Schedule the EPS status request thread
        cx.schedule
            .eps_query(cx.start + EPS_QUERY_PERIOD.cycles())
            .unwrap();

        // Schedule the Uart clear buffer function (only runs once at start)
        cx.schedule
            .uart_buffer_clear(cx.start + UART_PARSE_PERIOD.cycles())
            .unwrap();

        init::LateResources {
            rx_prod,
            rx_cons,
            tx_prod,
            tx_cons,
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

    #[idle(resources = [adc, vref, analog_pins, conn_tx, debug_tx, rx_cons, tx_cons, watchdog_done, digital_pins, led3, delay])]
    fn idle(cx: idle::Context) -> ! {
        // Pull variables from the Context struct for conveinece.
        let rx_queue = cx.resources.rx_cons;
        let tx_queue = cx.resources.tx_cons;
        let debug_tx = cx.resources.debug_tx;
        let conn_tx = cx.resources.conn_tx;
        let adc = cx.resources.adc;
        let analog_pins = cx.resources.analog_pins;
        let _digital_pins = cx.resources.digital_pins;
        let led3 = cx.resources.led3;
        let vref = cx.resources.vref;
        let delay = cx.resources.delay;

        // Radio telemetry stuct sent to the radio
        let mut radioTelemetry = RadioTelemetry {
            tid: TelemetryID::SOH,
            message: mod_RadioTelemetry::OneOfmessage::soh(RadioSOH {
                batteryVoltage: Some(BatteryVoltage {
                    battery1: 0,
                    battery2: 0,
                }),
                solarVoltage: Some(SolarVoltage {
                    side1: 0,
                    side2: 0,
                    side3: 0,
                    side4: 0,
                    side5: 0,
                    side6: 0,
                }),
                batteryVoltageState: BatteryVoltageState::BothLow,
                batteryManagerStates: Some(BatteryManagerStates {
                    battery1State: BatteryManagerState::Suspended,
                    battery2State: BatteryManagerState::Suspended,
                }),
                railSoh: Some(RailSOH {
                    rail1: false,
                    rail2: false,
                    rail3: false,
                    rail4: false,
                    rail5: false,
                    rail6: false,
                    rail7: false,
                    rail8: false,
                    rail9: false,
                    rail10: false,
                    rail11: false,
                    rail12: false,
                    rail13: false,
                    rail14: false,
                    rail15: false,
                    rail16: false,
                    hpwr1: false,
                    hpwr2: false,
                    hpwrEn: false,
                }),
            }),
        };
        if let OneOfmessage::soh(ref mut radioSoh) = radioTelemetry.message {
            loop {
                //
                // 1)
                // Measure state & save it somewhere
                //adc.calibrate(vref);
                adc.set_sample_time(adc::SampleTime::Cycles47_5);
                let th1 = adc.read(&mut analog_pins.th1).unwrap();
                let th2 = adc.read(&mut analog_pins.th2).unwrap();
                let th3 = adc.read(&mut analog_pins.th3).unwrap();
                let th4 = adc.read(&mut analog_pins.th4).unwrap();
                let th5 = adc.read(&mut analog_pins.th5).unwrap();
                let th6 = adc.read(&mut analog_pins.th6).unwrap();
                let th7 = adc.read(&mut analog_pins.th6).unwrap();
                let th8 = adc.read(&mut analog_pins.th6).unwrap();

                //
                // 2)

                //
                // 3)
                // Send any commands to the EPS
                // Only send one at a time
                if let Some(eps_command) = tx_queue.dequeue() {
                    let mut tmp_buf = [0u8; 1024];
                    serialize_into_slice(&eps_command, &mut tmp_buf).ok();
                    for elem in tmp_buf.iter().take(eps_command.get_size() + 1) {
                        block!(conn_tx.write(*elem)).unwrap();
                    }
                }

                //
                // 4)
                // Loop over any responses we may have recieved and update state
                while let Some(eps_response) = rx_queue.dequeue() {
                    match eps_response.cid {
                        CommandID::SetPowerRailState => { /* Do nothing, this response is just an ACK*/
                        }
                        // Update our internal storage with the rail state
                        CommandID::GetPowerRailState => match eps_response.resp {
                            OneOfresp::railState(ref rs) => {
                                if let Some(ref mut railSoh) = radioSoh.railSoh {
                                    match rs.railIdx {
                                        // This ought to be an index into an array, but it works fine as is.
                                        PowerRails::Rail1 => railSoh.rail1 = rs.railState,
                                        PowerRails::Rail2 => railSoh.rail2 = rs.railState,
                                        PowerRails::Rail3 => railSoh.rail3 = rs.railState,
                                        PowerRails::Rail4 => railSoh.rail4 = rs.railState,
                                        PowerRails::Rail5 => railSoh.rail5 = rs.railState,
                                        PowerRails::Rail6 => railSoh.rail6 = rs.railState,
                                        PowerRails::Rail7 => railSoh.rail7 = rs.railState,
                                        PowerRails::Rail8 => railSoh.rail8 = rs.railState,
                                        PowerRails::Rail9 => railSoh.rail9 = rs.railState,
                                        PowerRails::Rail10 => railSoh.rail10 = rs.railState,
                                        PowerRails::Rail11 => railSoh.rail11 = rs.railState,
                                        PowerRails::Rail12 => railSoh.rail12 = rs.railState,
                                        PowerRails::Rail13 => railSoh.rail13 = rs.railState,
                                        PowerRails::Rail14 => railSoh.rail14 = rs.railState,
                                        PowerRails::Rail15 => railSoh.rail15 = rs.railState,
                                        PowerRails::Rail16 => railSoh.rail16 = rs.railState,
                                        PowerRails::Hpwr1 => railSoh.hpwr1 = rs.railState,
                                        PowerRails::Hpwr2 => railSoh.hpwr2 = rs.railState,
                                        PowerRails::HpwrEn => railSoh.hpwrEn = rs.railState,
                                    }
                                }
                            }
                            OneOfresp::None => {}
                            _ => {}
                        },
                        CommandID::GetBatteryVoltage => {
                            if let OneOfresp::batteryVoltage(ref bv) = eps_response.resp {
                                if let Some(ref mut batteryVoltage) = radioSoh.batteryVoltage {
                                    batteryVoltage.battery1 = bv.battery1;
                                    batteryVoltage.battery2 = bv.battery2;
                                }
                            }
                        }
                        CommandID::GetSolarVoltage => {
                            if let OneOfresp::solarVoltage(ref sv) = eps_response.resp {
                                if let Some(ref mut solarVoltage) = radioSoh.solarVoltage {
                                    solarVoltage.side1 = sv.side1;
                                    solarVoltage.side2 = sv.side2;
                                    solarVoltage.side3 = sv.side3;
                                    solarVoltage.side4 = sv.side4;
                                    solarVoltage.side5 = sv.side5;
                                    solarVoltage.side6 = sv.side6;
                                }
                            }
                        }
                        CommandID::GetBatteryVoltageState => {
                            if let OneOfresp::batteryVoltageState(ref bvs) = eps_response.resp {
                                radioSoh.batteryVoltageState = *bvs;
                            }
                        }
                        CommandID::GetBatteryManagerState => {
                            if let OneOfresp::batteryManagerStates(ref bms) = eps_response.resp {
                                if let Some(ref mut batteryManagerStates) =
                                    radioSoh.batteryManagerStates
                                {
                                    batteryManagerStates.battery1State = bms.battery1State;
                                    batteryManagerStates.battery2State = bms.battery2State;
                                }
                            }
                        }
                    };

                    let mut test_str_buffer = ArrayString::<512>::new();
                    core::fmt::write(
                        &mut test_str_buffer,
                        format_args!("parsed message from EPS: {:?}\n\r", eps_response),
                    )
                    .unwrap();

                    // Write string buffer out to UART
                    for c in test_str_buffer.as_str().bytes() {
                        block!(debug_tx.write(c)).unwrap();
                    }
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

        // Should not return
        loop {
            cortex_m::asm::bkpt();
        }
    }

    // This task and uart_parse_active share the same priority, so they can't pre-empt each other
    #[task(binds = UART4, resources = [led5, conn_rx, rx_prod, uart_parse_active, uart_parse_vec], schedule = [uart_buffer_clear], priority = 3)]
    fn uart4(cx: uart4::Context) {
        let rx = cx.resources.conn_rx;
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

    #[task(resources = [tx_prod], schedule = [eps_query], priority = 1)]
    fn eps_query(cx: eps_query::Context) {
        static mut CMD_TO_SEND: u8 = 0;
        let tx_prod = cx.resources.tx_prod;

        //cortex_m::asm::bkpt();
        let cmd = match *CMD_TO_SEND {
            1 => EpsCommand {
                cid: CommandID::GetSolarVoltage,
                railState: None,
            },
            2 => EpsCommand {
                cid: CommandID::GetBatteryVoltageState,
                railState: None,
            },
            3 => EpsCommand {
                cid: CommandID::GetBatteryManagerState,
                railState: None,
            },
            _ => EpsCommand {
                cid: CommandID::GetBatteryVoltage,
                railState: None,
            },
        };
        *CMD_TO_SEND = (*CMD_TO_SEND + 1) % 4;

        tx_prod.enqueue(cmd).ok();

        // Schedule
        cx.schedule
            .eps_query(cx.scheduled + EPS_QUERY_PERIOD.cycles())
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
