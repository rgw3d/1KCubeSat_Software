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
use super::super::*;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum CommandID {
    SetPowerRailState = 1,
    GetPowerRailState = 2,
    GetBatteryVoltage = 3,
    GetSolarVoltage = 4,
    GetBatteryVoltageState = 5,
    GetBatteryManagerState = 6,
}

impl Default for CommandID {
    fn default() -> Self {
        CommandID::SetPowerRailState
    }
}

impl From<i32> for CommandID {
    fn from(i: i32) -> Self {
        match i {
            1 => CommandID::SetPowerRailState,
            2 => CommandID::GetPowerRailState,
            3 => CommandID::GetBatteryVoltage,
            4 => CommandID::GetSolarVoltage,
            5 => CommandID::GetBatteryVoltageState,
            6 => CommandID::GetBatteryManagerState,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for CommandID {
    fn from(s: &'a str) -> Self {
        match s {
            "SetPowerRailState" => CommandID::SetPowerRailState,
            "GetPowerRailState" => CommandID::GetPowerRailState,
            "GetBatteryVoltage" => CommandID::GetBatteryVoltage,
            "GetSolarVoltage" => CommandID::GetSolarVoltage,
            "GetBatteryVoltageState" => CommandID::GetBatteryVoltageState,
            "GetBatteryManagerState" => CommandID::GetBatteryManagerState,
            _ => Self::default(),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum PowerRails {
    Rail1 = 0,
    Rail2 = 1,
    Rail3 = 2,
    Rail4 = 3,
    Rail5 = 4,
    Rail6 = 5,
    Rail7 = 6,
    Rail8 = 7,
    Rail9 = 8,
    Rail10 = 9,
    Rail11 = 10,
    Rail12 = 11,
    Rail13 = 12,
    Rail14 = 13,
    Rail15 = 14,
    Rail16 = 15,
    Hpwr1 = 16,
    Hpwr2 = 17,
    HpwrEn = 18,
}

impl Default for PowerRails {
    fn default() -> Self {
        PowerRails::Rail1
    }
}

impl From<i32> for PowerRails {
    fn from(i: i32) -> Self {
        match i {
            0 => PowerRails::Rail1,
            1 => PowerRails::Rail2,
            2 => PowerRails::Rail3,
            3 => PowerRails::Rail4,
            4 => PowerRails::Rail5,
            5 => PowerRails::Rail6,
            6 => PowerRails::Rail7,
            7 => PowerRails::Rail8,
            8 => PowerRails::Rail9,
            9 => PowerRails::Rail10,
            10 => PowerRails::Rail11,
            11 => PowerRails::Rail12,
            12 => PowerRails::Rail13,
            13 => PowerRails::Rail14,
            14 => PowerRails::Rail15,
            15 => PowerRails::Rail16,
            16 => PowerRails::Hpwr1,
            17 => PowerRails::Hpwr2,
            18 => PowerRails::HpwrEn,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for PowerRails {
    fn from(s: &'a str) -> Self {
        match s {
            "Rail1" => PowerRails::Rail1,
            "Rail2" => PowerRails::Rail2,
            "Rail3" => PowerRails::Rail3,
            "Rail4" => PowerRails::Rail4,
            "Rail5" => PowerRails::Rail5,
            "Rail6" => PowerRails::Rail6,
            "Rail7" => PowerRails::Rail7,
            "Rail8" => PowerRails::Rail8,
            "Rail9" => PowerRails::Rail9,
            "Rail10" => PowerRails::Rail10,
            "Rail11" => PowerRails::Rail11,
            "Rail12" => PowerRails::Rail12,
            "Rail13" => PowerRails::Rail13,
            "Rail14" => PowerRails::Rail14,
            "Rail15" => PowerRails::Rail15,
            "Rail16" => PowerRails::Rail16,
            "Hpwr1" => PowerRails::Hpwr1,
            "Hpwr2" => PowerRails::Hpwr2,
            "HpwrEn" => PowerRails::HpwrEn,
            _ => Self::default(),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum BatteryVoltageState {
    BothHigh = 1,
    B1HighB2Low = 2,
    B1LowB2High = 3,
    BothLow = 4,
}

impl Default for BatteryVoltageState {
    fn default() -> Self {
        BatteryVoltageState::BothHigh
    }
}

impl From<i32> for BatteryVoltageState {
    fn from(i: i32) -> Self {
        match i {
            1 => BatteryVoltageState::BothHigh,
            2 => BatteryVoltageState::B1HighB2Low,
            3 => BatteryVoltageState::B1LowB2High,
            4 => BatteryVoltageState::BothLow,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for BatteryVoltageState {
    fn from(s: &'a str) -> Self {
        match s {
            "BothHigh" => BatteryVoltageState::BothHigh,
            "B1HighB2Low" => BatteryVoltageState::B1HighB2Low,
            "B1LowB2High" => BatteryVoltageState::B1LowB2High,
            "BothLow" => BatteryVoltageState::BothLow,
            _ => Self::default(),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum BatteryManagerState {
    Suspended = 1,
    LowPower = 2,
    HighPower = 3,
}

impl Default for BatteryManagerState {
    fn default() -> Self {
        BatteryManagerState::Suspended
    }
}

impl From<i32> for BatteryManagerState {
    fn from(i: i32) -> Self {
        match i {
            1 => BatteryManagerState::Suspended,
            2 => BatteryManagerState::LowPower,
            3 => BatteryManagerState::HighPower,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for BatteryManagerState {
    fn from(s: &'a str) -> Self {
        match s {
            "Suspended" => BatteryManagerState::Suspended,
            "LowPower" => BatteryManagerState::LowPower,
            "HighPower" => BatteryManagerState::HighPower,
            _ => Self::default(),
        }
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct RailState {
    pub railIdx: protos::no_std::PowerRails,
    pub railState: bool,
}

impl<'a> MessageRead<'a> for RailState {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.railIdx = r.read_enum(bytes)?,
                Ok(16) => msg.railState = r.read_bool(bytes)?,
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
        + if self.railIdx == protos::no_std::PowerRails::Rail1 { 0 } else { 1 + sizeof_varint(*(&self.railIdx) as u64) }
        + if self.railState == false { 0 } else { 1 + sizeof_varint(*(&self.railState) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.railIdx != protos::no_std::PowerRails::Rail1 { w.write_with_tag(8, |w| w.write_enum(*&self.railIdx as i32))?; }
        if self.railState != false { w.write_with_tag(16, |w| w.write_bool(*&self.railState))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct EpsCommand {
    pub cid: protos::no_std::CommandID,
    pub railState: Option<protos::no_std::RailState>,
}

impl<'a> MessageRead<'a> for EpsCommand {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.cid = r.read_enum(bytes)?,
                Ok(18) => msg.railState = Some(r.read_message::<protos::no_std::RailState>(bytes)?),
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
        + if self.cid == protos::no_std::CommandID::SetPowerRailState { 0 } else { 1 + sizeof_varint(*(&self.cid) as u64) }
        + self.railState.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.cid != protos::no_std::CommandID::SetPowerRailState { w.write_with_tag(8, |w| w.write_enum(*&self.cid as i32))?; }
        if let Some(ref s) = self.railState { w.write_with_tag(18, |w| w.write_message(s))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct BatteryVoltage {
    pub battery1: u32,
    pub battery2: u32,
}

impl<'a> MessageRead<'a> for BatteryVoltage {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.battery1 = r.read_uint32(bytes)?,
                Ok(16) => msg.battery2 = r.read_uint32(bytes)?,
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
        + if self.battery1 == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.battery1) as u64) }
        + if self.battery2 == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.battery2) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.battery1 != 0u32 { w.write_with_tag(8, |w| w.write_uint32(*&self.battery1))?; }
        if self.battery2 != 0u32 { w.write_with_tag(16, |w| w.write_uint32(*&self.battery2))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct SolarVoltage {
    pub side1: u32,
    pub side2: u32,
    pub side3: u32,
    pub side4: u32,
    pub side5: u32,
    pub side6: u32,
}

impl<'a> MessageRead<'a> for SolarVoltage {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.side1 = r.read_uint32(bytes)?,
                Ok(16) => msg.side2 = r.read_uint32(bytes)?,
                Ok(24) => msg.side3 = r.read_uint32(bytes)?,
                Ok(32) => msg.side4 = r.read_uint32(bytes)?,
                Ok(40) => msg.side5 = r.read_uint32(bytes)?,
                Ok(48) => msg.side6 = r.read_uint32(bytes)?,
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
        + if self.side1 == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.side1) as u64) }
        + if self.side2 == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.side2) as u64) }
        + if self.side3 == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.side3) as u64) }
        + if self.side4 == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.side4) as u64) }
        + if self.side5 == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.side5) as u64) }
        + if self.side6 == 0u32 { 0 } else { 1 + sizeof_varint(*(&self.side6) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.side1 != 0u32 { w.write_with_tag(8, |w| w.write_uint32(*&self.side1))?; }
        if self.side2 != 0u32 { w.write_with_tag(16, |w| w.write_uint32(*&self.side2))?; }
        if self.side3 != 0u32 { w.write_with_tag(24, |w| w.write_uint32(*&self.side3))?; }
        if self.side4 != 0u32 { w.write_with_tag(32, |w| w.write_uint32(*&self.side4))?; }
        if self.side5 != 0u32 { w.write_with_tag(40, |w| w.write_uint32(*&self.side5))?; }
        if self.side6 != 0u32 { w.write_with_tag(48, |w| w.write_uint32(*&self.side6))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct BatteryManagerStates {
    pub battery1State: protos::no_std::BatteryManagerState,
    pub battery2State: protos::no_std::BatteryManagerState,
}

impl<'a> MessageRead<'a> for BatteryManagerStates {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.battery1State = r.read_enum(bytes)?,
                Ok(16) => msg.battery2State = r.read_enum(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for BatteryManagerStates {
    fn get_size(&self) -> usize {
        0
        + if self.battery1State == protos::no_std::BatteryManagerState::Suspended { 0 } else { 1 + sizeof_varint(*(&self.battery1State) as u64) }
        + if self.battery2State == protos::no_std::BatteryManagerState::Suspended { 0 } else { 1 + sizeof_varint(*(&self.battery2State) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.battery1State != protos::no_std::BatteryManagerState::Suspended { w.write_with_tag(8, |w| w.write_enum(*&self.battery1State as i32))?; }
        if self.battery2State != protos::no_std::BatteryManagerState::Suspended { w.write_with_tag(16, |w| w.write_enum(*&self.battery2State as i32))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct EpsResponse {
    pub cid: protos::no_std::CommandID,
    pub resp: protos::no_std::mod_EpsResponse::OneOfresp,
}

impl<'a> MessageRead<'a> for EpsResponse {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.cid = r.read_enum(bytes)?,
                Ok(18) => msg.resp = protos::no_std::mod_EpsResponse::OneOfresp::railState(r.read_message::<protos::no_std::RailState>(bytes)?),
                Ok(26) => msg.resp = protos::no_std::mod_EpsResponse::OneOfresp::batteryVoltage(r.read_message::<protos::no_std::BatteryVoltage>(bytes)?),
                Ok(34) => msg.resp = protos::no_std::mod_EpsResponse::OneOfresp::solarVoltage(r.read_message::<protos::no_std::SolarVoltage>(bytes)?),
                Ok(40) => msg.resp = protos::no_std::mod_EpsResponse::OneOfresp::batteryVoltageState(r.read_enum(bytes)?),
                Ok(50) => msg.resp = protos::no_std::mod_EpsResponse::OneOfresp::batteryManagerStates(r.read_message::<protos::no_std::BatteryManagerStates>(bytes)?),
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
        + if self.cid == protos::no_std::CommandID::SetPowerRailState { 0 } else { 1 + sizeof_varint(*(&self.cid) as u64) }
        + match self.resp {
            protos::no_std::mod_EpsResponse::OneOfresp::railState(ref m) => 1 + sizeof_len((m).get_size()),
            protos::no_std::mod_EpsResponse::OneOfresp::batteryVoltage(ref m) => 1 + sizeof_len((m).get_size()),
            protos::no_std::mod_EpsResponse::OneOfresp::solarVoltage(ref m) => 1 + sizeof_len((m).get_size()),
            protos::no_std::mod_EpsResponse::OneOfresp::batteryVoltageState(ref m) => 1 + sizeof_varint(*(m) as u64),
            protos::no_std::mod_EpsResponse::OneOfresp::batteryManagerStates(ref m) => 1 + sizeof_len((m).get_size()),
            protos::no_std::mod_EpsResponse::OneOfresp::None => 0,
    }    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.cid != protos::no_std::CommandID::SetPowerRailState { w.write_with_tag(8, |w| w.write_enum(*&self.cid as i32))?; }
        match self.resp {            protos::no_std::mod_EpsResponse::OneOfresp::railState(ref m) => { w.write_with_tag(18, |w| w.write_message(m))? },
            protos::no_std::mod_EpsResponse::OneOfresp::batteryVoltage(ref m) => { w.write_with_tag(26, |w| w.write_message(m))? },
            protos::no_std::mod_EpsResponse::OneOfresp::solarVoltage(ref m) => { w.write_with_tag(34, |w| w.write_message(m))? },
            protos::no_std::mod_EpsResponse::OneOfresp::batteryVoltageState(ref m) => { w.write_with_tag(40, |w| w.write_enum(*m as i32))? },
            protos::no_std::mod_EpsResponse::OneOfresp::batteryManagerStates(ref m) => { w.write_with_tag(50, |w| w.write_message(m))? },
            protos::no_std::mod_EpsResponse::OneOfresp::None => {},
    }        Ok(())
    }
}

pub mod mod_EpsResponse {

use super::*;

#[derive(Debug, PartialEq, Clone)]
pub enum OneOfresp {
    railState(protos::no_std::RailState),
    batteryVoltage(protos::no_std::BatteryVoltage),
    solarVoltage(protos::no_std::SolarVoltage),
    batteryVoltageState(protos::no_std::BatteryVoltageState),
    batteryManagerStates(protos::no_std::BatteryManagerStates),
    None,
}

impl Default for OneOfresp {
    fn default() -> Self {
        OneOfresp::None
    }
}

}

