// Automatically generated rust module for 'messages.proto' file

#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(unused_imports)]
#![allow(unknown_lints)]
#![allow(clippy::all)]
#![cfg_attr(rustfmt, rustfmt_skip)]


use quick_protobuf::{MessageRead, MessageWrite, BytesReader, Writer, WriterBackend, Result};
use quick_protobuf::sizeofs::*;
use super::*;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum CommandID {
    SetPowerRailState = 0,
    GetPowerRailState = 1,
    MeasureBatteryVoltage = 2,
    GetBatteryVoltage = 3,
    MeasureSolarVoltage = 4,
    GetSolarVoltage = 5,
    GetBatteryState = 6,
}

impl Default for CommandID {
    fn default() -> Self {
        CommandID::SetPowerRailState
    }
}

impl From<i32> for CommandID {
    fn from(i: i32) -> Self {
        match i {
            0 => CommandID::SetPowerRailState,
            1 => CommandID::GetPowerRailState,
            2 => CommandID::MeasureBatteryVoltage,
            3 => CommandID::GetBatteryVoltage,
            4 => CommandID::MeasureSolarVoltage,
            5 => CommandID::GetSolarVoltage,
            6 => CommandID::GetBatteryState,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for CommandID {
    fn from(s: &'a str) -> Self {
        match s {
            "SetPowerRailState" => CommandID::SetPowerRailState,
            "GetPowerRailState" => CommandID::GetPowerRailState,
            "MeasureBatteryVoltage" => CommandID::MeasureBatteryVoltage,
            "GetBatteryVoltage" => CommandID::GetBatteryVoltage,
            "MeasureSolarVoltage" => CommandID::MeasureSolarVoltage,
            "GetSolarVoltage" => CommandID::GetSolarVoltage,
            "GetBatteryState" => CommandID::GetBatteryState,
            _ => Self::default(),
        }
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct EpsCommand {
    pub cid: CommandID,
    pub rail: i32,
    pub setRailOn: bool,
}

impl<'a> MessageRead<'a> for EpsCommand {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.cid = r.read_enum(bytes)?,
                Ok(16) => msg.rail = r.read_int32(bytes)?,
                Ok(24) => msg.setRailOn = r.read_bool(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for EpsCommand {
    fn get_size(&self) -> usize {
        0
        + if self.cid == messages::CommandID::SetPowerRailState { 0 } else { 1 + sizeof_varint(*(&self.cid) as u64) }
        + if self.rail == 0i32 { 0 } else { 1 + sizeof_varint(*(&self.rail) as u64) }
        + if self.setRailOn == false { 0 } else { 1 + sizeof_varint(*(&self.setRailOn) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.cid != messages::CommandID::SetPowerRailState { w.write_with_tag(8, |w| w.write_enum(*&self.cid as i32))?; }
        if self.rail != 0i32 { w.write_with_tag(16, |w| w.write_int32(*&self.rail))?; }
        if self.setRailOn != false { w.write_with_tag(24, |w| w.write_bool(*&self.setRailOn))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct RailState {
    pub isRailOn: bool,
}

impl<'a> MessageRead<'a> for RailState {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.isRailOn = r.read_bool(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for RailState {
    fn get_size(&self) -> usize {
        0
        + if self.isRailOn == false { 0 } else { 1 + sizeof_varint(*(&self.isRailOn) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.isRailOn != false { w.write_with_tag(8, |w| w.write_bool(*&self.isRailOn))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct BatteryVoltage {
    pub battery1: i32,
    pub battery2: i32,
}

impl<'a> MessageRead<'a> for BatteryVoltage {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.battery1 = r.read_int32(bytes)?,
                Ok(16) => msg.battery2 = r.read_int32(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for BatteryVoltage {
    fn get_size(&self) -> usize {
        0
        + if self.battery1 == 0i32 { 0 } else { 1 + sizeof_varint(*(&self.battery1) as u64) }
        + if self.battery2 == 0i32 { 0 } else { 1 + sizeof_varint(*(&self.battery2) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.battery1 != 0i32 { w.write_with_tag(8, |w| w.write_int32(*&self.battery1))?; }
        if self.battery2 != 0i32 { w.write_with_tag(16, |w| w.write_int32(*&self.battery2))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct SolarVoltage {
    pub side1: i32,
    pub side2: i32,
    pub side3: i32,
    pub side4: i32,
    pub side5: i32,
    pub side6: i32,
}

impl<'a> MessageRead<'a> for SolarVoltage {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.side1 = r.read_int32(bytes)?,
                Ok(16) => msg.side2 = r.read_int32(bytes)?,
                Ok(24) => msg.side3 = r.read_int32(bytes)?,
                Ok(32) => msg.side4 = r.read_int32(bytes)?,
                Ok(40) => msg.side5 = r.read_int32(bytes)?,
                Ok(48) => msg.side6 = r.read_int32(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for SolarVoltage {
    fn get_size(&self) -> usize {
        0
        + if self.side1 == 0i32 { 0 } else { 1 + sizeof_varint(*(&self.side1) as u64) }
        + if self.side2 == 0i32 { 0 } else { 1 + sizeof_varint(*(&self.side2) as u64) }
        + if self.side3 == 0i32 { 0 } else { 1 + sizeof_varint(*(&self.side3) as u64) }
        + if self.side4 == 0i32 { 0 } else { 1 + sizeof_varint(*(&self.side4) as u64) }
        + if self.side5 == 0i32 { 0 } else { 1 + sizeof_varint(*(&self.side5) as u64) }
        + if self.side6 == 0i32 { 0 } else { 1 + sizeof_varint(*(&self.side6) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.side1 != 0i32 { w.write_with_tag(8, |w| w.write_int32(*&self.side1))?; }
        if self.side2 != 0i32 { w.write_with_tag(16, |w| w.write_int32(*&self.side2))?; }
        if self.side3 != 0i32 { w.write_with_tag(24, |w| w.write_int32(*&self.side3))?; }
        if self.side4 != 0i32 { w.write_with_tag(32, |w| w.write_int32(*&self.side4))?; }
        if self.side5 != 0i32 { w.write_with_tag(40, |w| w.write_int32(*&self.side5))?; }
        if self.side6 != 0i32 { w.write_with_tag(48, |w| w.write_int32(*&self.side6))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct BatteryState {
    pub battery1State: i32,
    pub battery2State: i32,
}

impl<'a> MessageRead<'a> for BatteryState {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.battery1State = r.read_int32(bytes)?,
                Ok(16) => msg.battery2State = r.read_int32(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for BatteryState {
    fn get_size(&self) -> usize {
        0
        + if self.battery1State == 0i32 { 0 } else { 1 + sizeof_varint(*(&self.battery1State) as u64) }
        + if self.battery2State == 0i32 { 0 } else { 1 + sizeof_varint(*(&self.battery2State) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.battery1State != 0i32 { w.write_with_tag(8, |w| w.write_int32(*&self.battery1State))?; }
        if self.battery2State != 0i32 { w.write_with_tag(16, |w| w.write_int32(*&self.battery2State))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct EpsResponse {
    pub cid: CommandID,
    pub resp: mod_EpsResponse::OneOfresp,
}

impl<'a> MessageRead<'a> for EpsResponse {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.cid = r.read_enum(bytes)?,
                Ok(18) => msg.resp = mod_EpsResponse::OneOfresp::railState(r.read_message::<RailState>(bytes)?),
                Ok(26) => msg.resp = mod_EpsResponse::OneOfresp::batteryVoltage(r.read_message::<BatteryVoltage>(bytes)?),
                Ok(34) => msg.resp = mod_EpsResponse::OneOfresp::solarVoltage(r.read_message::<SolarVoltage>(bytes)?),
                Ok(42) => msg.resp = mod_EpsResponse::OneOfresp::batteryState(r.read_message::<BatteryState>(bytes)?),
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for EpsResponse {
    fn get_size(&self) -> usize {
        0
        + if self.cid == messages::CommandID::SetPowerRailState { 0 } else { 1 + sizeof_varint(*(&self.cid) as u64) }
        + match self.resp {
            mod_EpsResponse::OneOfresp::railState(ref m) => 1 + sizeof_len((m).get_size()),
            mod_EpsResponse::OneOfresp::batteryVoltage(ref m) => 1 + sizeof_len((m).get_size()),
            mod_EpsResponse::OneOfresp::solarVoltage(ref m) => 1 + sizeof_len((m).get_size()),
            mod_EpsResponse::OneOfresp::batteryState(ref m) => 1 + sizeof_len((m).get_size()),
            mod_EpsResponse::OneOfresp::None => 0,
    }    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.cid != messages::CommandID::SetPowerRailState { w.write_with_tag(8, |w| w.write_enum(*&self.cid as i32))?; }
        match self.resp {            mod_EpsResponse::OneOfresp::railState(ref m) => { w.write_with_tag(18, |w| w.write_message(m))? },
            mod_EpsResponse::OneOfresp::batteryVoltage(ref m) => { w.write_with_tag(26, |w| w.write_message(m))? },
            mod_EpsResponse::OneOfresp::solarVoltage(ref m) => { w.write_with_tag(34, |w| w.write_message(m))? },
            mod_EpsResponse::OneOfresp::batteryState(ref m) => { w.write_with_tag(42, |w| w.write_message(m))? },
            mod_EpsResponse::OneOfresp::None => {},
    }        Ok(())
    }
}

pub mod mod_EpsResponse {

use super::*;

#[derive(Debug, PartialEq, Clone)]
pub enum OneOfresp {
    railState(RailState),
    batteryVoltage(BatteryVoltage),
    solarVoltage(SolarVoltage),
    batteryState(BatteryState),
    None,
}

impl Default for OneOfresp {
    fn default() -> Self {
        OneOfresp::None
    }
}

}

