use quick_protobuf::{deserialize_from_slice, serialize_into_slice, serialize_into_vec};
use std::{thread, time::Duration};
pub mod messages;
use messages::{
    mod_EpsResponse::OneOfresp, CommandID, EpsCommand, EpsResponse, PowerRails, RailState,
};
extern crate byteorder;
use byteorder::{ByteOrder, LittleEndian};

const SLEEP_TIME_BETWEEN_COMMANDS: u64 = 50;

fn main() {
    println!("Start");
    let mut port = serialport::new("/dev/ttyUSB0", 9_600)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");
    //
    //
    // Get Power rail 1 state, should be on (STM rail)
    let cmd = EpsCommand {
        cid: CommandID::GetPowerRailState,
        railState: Some(RailState {
            railIdx: PowerRails::Rail1,
            railState: false, // this variable doesn't matter
        }),
    };
    let expected_resp = EpsResponse {
        cid: CommandID::GetPowerRailState,
        resp: OneOfresp::railState(RailState {
            railIdx: PowerRails::Rail1,
            railState: true,
        }),
    };
    if let Ok(resp) = send_eps_command(&mut port, cmd) {
        assert_eq!(resp, expected_resp);
    }
    thread::sleep(Duration::from_millis(SLEEP_TIME_BETWEEN_COMMANDS));

    //
    //
    // Get Power rail 13, should be on (EPS power rail)
    let cmd = EpsCommand {
        cid: CommandID::GetPowerRailState,
        railState: Some(RailState {
            railIdx: PowerRails::Rail13,
            railState: false,
        }),
    };
    let expected_resp = EpsResponse {
        cid: CommandID::GetPowerRailState,
        resp: OneOfresp::railState(RailState {
            railIdx: PowerRails::Rail13,
            // TODO Fix
            //
            // should be true
            railState: true,
        }),
    };
    if let Ok(resp) = send_eps_command(&mut port, cmd) {
        assert_eq!(resp, expected_resp);
    }
    thread::sleep(Duration::from_millis(SLEEP_TIME_BETWEEN_COMMANDS));

    //
    //
    // Set power rail 2 to on
    let cmd = EpsCommand {
        cid: CommandID::SetPowerRailState,
        railState: Some(RailState {
            railIdx: PowerRails::Rail2,
            railState: true,
        }),
    };
    let expected_resp = EpsResponse {
        cid: CommandID::SetPowerRailState,
        resp: OneOfresp::None,
    };
    if let Ok(resp) = send_eps_command(&mut port, cmd) {
        assert_eq!(resp, expected_resp);
    }
    thread::sleep(Duration::from_millis(SLEEP_TIME_BETWEEN_COMMANDS));

    set_power_rail(PowerRails::Rail3, &mut port);
    set_power_rail(PowerRails::Rail4, &mut port);
    set_power_rail(PowerRails::Rail5, &mut port);
    set_power_rail(PowerRails::Rail6, &mut port);
    set_power_rail(PowerRails::Rail7, &mut port);
    set_power_rail(PowerRails::Rail8, &mut port);
    set_power_rail(PowerRails::Rail9, &mut port);
    set_power_rail(PowerRails::Rail10, &mut port);
    set_power_rail(PowerRails::Rail11, &mut port);
    set_power_rail(PowerRails::Rail12, &mut port);
    set_power_rail(PowerRails::Rail14, &mut port);
    set_power_rail(PowerRails::Rail15, &mut port);
    set_power_rail(PowerRails::Rail16, &mut port);

    //
    //
    // Get power rail 2, make sure it is on
    let cmd = EpsCommand {
        cid: CommandID::GetPowerRailState,
        railState: Some(RailState {
            railIdx: PowerRails::Rail2,
            railState: false,
        }),
    };
    let expected_resp = EpsResponse {
        cid: CommandID::GetPowerRailState,
        resp: OneOfresp::railState(RailState {
            railIdx: PowerRails::Rail2,
            railState: true,
        }),
    };
    if let Ok(resp) = send_eps_command(&mut port, cmd) {
        assert_eq!(resp, expected_resp);
    }
    thread::sleep(Duration::from_millis(SLEEP_TIME_BETWEEN_COMMANDS));

    //
    //
    // Set power rail 2 to off
    // let cmd = EpsCommand {
    //     cid: CommandID::SetPowerRailState,
    //     railState: Some(RailState {
    //         railIdx: PowerRails::Rail2,
    //         railState: false,
    //     }),
    // };
    // let expected_resp = EpsResponse {
    //     cid: CommandID::SetPowerRailState,
    //     resp: OneOfresp::None,
    // };
    // if let Ok(resp) = send_eps_command(&mut port, cmd) {
    //     assert_eq!(resp, expected_resp);
    // }
    // thread::sleep(Duration::from_millis(SLEEP_TIME_BETWEEN_COMMANDS));

    // //
    // //
    // // Get power rail 2, make sure it is off
    // let cmd = EpsCommand {
    //     cid: CommandID::GetPowerRailState,
    //     railState: Some(RailState {
    //         railIdx: PowerRails::Rail2,
    //         railState: false,
    //     }),
    // };
    // let expected_resp = EpsResponse {
    //     cid: CommandID::GetPowerRailState,
    //     resp: OneOfresp::railState(RailState {
    //         railIdx: PowerRails::Rail2,
    //         railState: false,
    //     }),
    // };
    // if let Ok(resp) = send_eps_command(&mut port, cmd) {
    //     assert_eq!(resp, expected_resp);
    // }
    // thread::sleep(Duration::from_millis(SLEEP_TIME_BETWEEN_COMMANDS));

    //
    //
    // Get Battery Voltages
    let cmd = EpsCommand {
        cid: CommandID::GetBatteryVoltage,
        railState: None,
    };
    if let Ok(resp) = send_eps_command(&mut port, cmd) {
        if let OneOfresp::batteryVoltage(_) = resp.resp {
            println!("Pass!");
        }
    }
    thread::sleep(Duration::from_millis(SLEEP_TIME_BETWEEN_COMMANDS));

    //
    //
    // Get Solar Voltages
    let cmd = EpsCommand {
        cid: CommandID::GetSolarVoltage,
        railState: None,
    };
    if let Ok(resp) = send_eps_command(&mut port, cmd) {
        if let OneOfresp::solarVoltage(_) = resp.resp {
            println!("Pass!");
        }
    }
    thread::sleep(Duration::from_millis(SLEEP_TIME_BETWEEN_COMMANDS));

    //
    //
    // Get Battery Voltage State
    let cmd = EpsCommand {
        cid: CommandID::GetBatteryVoltageState,
        railState: None,
    };
    if let Ok(resp) = send_eps_command(&mut port, cmd) {
        if let OneOfresp::batteryVoltageState(_) = resp.resp {
            println!("Pass!");
        }
    }
    thread::sleep(Duration::from_millis(SLEEP_TIME_BETWEEN_COMMANDS));

    //
    //
    // Get Battery Manager States
    let cmd = EpsCommand {
        cid: CommandID::GetBatteryManagerState,
        railState: None,
    };
    if let Ok(resp) = send_eps_command(&mut port, cmd) {
        if let OneOfresp::batteryManagerStates(_) = resp.resp {
            println!("Pass!");
        }
    }
    thread::sleep(Duration::from_millis(SLEEP_TIME_BETWEEN_COMMANDS));
}

fn set_power_rail(rail: PowerRails, port: &mut std::boxed::Box<dyn serialport::SerialPort>) {
    // //
    // Set power rail 5 to on
    let cmd = EpsCommand {
        cid: CommandID::SetPowerRailState,
        railState: Some(RailState {
            railIdx: rail,
            railState: true,
        }),
    };
    let expected_resp = EpsResponse {
        cid: CommandID::SetPowerRailState,
        resp: OneOfresp::None,
    };
    if let Ok(resp) = send_eps_command(port, cmd) {
        assert_eq!(resp, expected_resp);
    }
    thread::sleep(Duration::from_millis(SLEEP_TIME_BETWEEN_COMMANDS));
}

fn send_eps_command(
    port: &mut std::boxed::Box<dyn serialport::SerialPort>,
    eps_cmd: EpsCommand,
) -> quick_protobuf::Result<EpsResponse> {
    println!("\nSending EpsCommand: {:?}", eps_cmd);
    let out_vec = serialize_into_vec(&eps_cmd).expect("oof ouch why can't I write message");
    println!("out_vec: {:?}", out_vec);
    port.clear(serialport::ClearBuffer::All).ok();
    port.set_timeout(Duration::from_millis(200)).ok();
    port.write_all(&out_vec).ok();

    let mut one_byte_vec: Vec<u8> = vec![0; 1];
    let mut incoming_message: Vec<u8> = Vec::new();
    while let Ok(()) = port.read_exact(one_byte_vec.as_mut_slice()) {
        incoming_message.push(one_byte_vec[0]);
    }
    println!("in_vec : {:?}", incoming_message);

    // Parse the message
    let eps_response: quick_protobuf::Result<EpsResponse> =
        deserialize_from_slice(&incoming_message);
    match eps_response {
        Ok(ref resp) => {
            println!("Eps Response: {:?}", resp);
        }
        Err(ref err) => {
            println!("Eps Response Err: {:?}", err);
        }
    }
    eps_response
}
