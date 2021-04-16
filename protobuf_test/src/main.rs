use quick_protobuf::{deserialize_from_slice, serialize_into_slice, serialize_into_vec};
use std::time::Duration;
pub mod messages;
use messages::{CommandID, EpsCommand};
extern crate byteorder;
use byteorder::{ByteOrder, LittleEndian};

fn main() {
    println!("Start");
    let mut port = serialport::new("/dev/ttyUSB0", 9_600)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");
    // Write
    let epsCmd = EpsCommand {
        cid: CommandID::SetPowerRailState,
        rail: 1,
        setRailOn: true,
    };

    //serialize_into_slice(epsCmd, out: &mut [u8])
    let out_vec = serialize_into_vec(&epsCmd).expect("oof ouch why can't I write message");
    println!("len of out_vec: {:?}", out_vec.len());
    for elem in out_vec.iter() {
        //port.write(elem).ok();
        println!("elem: {:?}", elem);
    }

    //let output = "This is a test. This is only a test.".as_bytes();
    //port.write_all(output).expect("Write failed!");

    port.clear(serialport::ClearBuffer::All).ok();
    port.set_timeout(Duration::new(1, 0)).ok();
    port.write_all(&out_vec).ok();

    let epsCmdOut: EpsCommand = deserialize_from_slice(&out_vec).expect("pls work");
    assert_eq!(epsCmd, epsCmdOut);

    // Read how big the message is (in bytes)
    let mut incoming_message_size: Vec<u8> = vec![0; 1];
    port.read_exact(incoming_message_size.as_mut_slice())
        .expect("Found no data1!");
    println!("got size: {:?}", incoming_message_size[0]);
    // Now read the appropriate number of bytes
    let mut incoming_message: Vec<u8> = Vec::new();
    incoming_message.resize(incoming_message_size[0] as usize, 0);
    port.read_exact(incoming_message.as_mut_slice())
        .expect("Found no data2!");

    // print it out
    for elem in &incoming_message {
        //port.write(elem).ok();
        println!("got: {:?}", elem);
    }
}
