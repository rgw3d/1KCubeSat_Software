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

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TelemetryID {
    SOH = 0,
}

impl Default for TelemetryID {
    fn default() -> Self {
        TelemetryID::SOH
    }
}

impl From<i32> for TelemetryID {
    fn from(i: i32) -> Self {
        match i {
            0 => TelemetryID::SOH,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for TelemetryID {
    fn from(s: &'a str) -> Self {
        match s {
            "SOH" => TelemetryID::SOH,
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

#[derive(Debug, Default, PartialEq, Clone)]
pub struct RailSOH {
    pub rail1: bool,
    pub rail2: bool,
    pub rail3: bool,
    pub rail4: bool,
    pub rail5: bool,
    pub rail6: bool,
    pub rail7: bool,
    pub rail8: bool,
    pub rail9: bool,
    pub rail10: bool,
    pub rail11: bool,
    pub rail12: bool,
    pub rail13: bool,
    pub rail14: bool,
    pub rail15: bool,
    pub rail16: bool,
    pub hpwr1: bool,
    pub hpwr2: bool,
    pub hpwrEn: bool,
}

impl<'a> MessageRead<'a> for RailSOH {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.rail1 = r.read_bool(bytes)?,
                Ok(16) => msg.rail2 = r.read_bool(bytes)?,
                Ok(24) => msg.rail3 = r.read_bool(bytes)?,
                Ok(32) => msg.rail4 = r.read_bool(bytes)?,
                Ok(40) => msg.rail5 = r.read_bool(bytes)?,
                Ok(48) => msg.rail6 = r.read_bool(bytes)?,
                Ok(56) => msg.rail7 = r.read_bool(bytes)?,
                Ok(64) => msg.rail8 = r.read_bool(bytes)?,
                Ok(72) => msg.rail9 = r.read_bool(bytes)?,
                Ok(80) => msg.rail10 = r.read_bool(bytes)?,
                Ok(88) => msg.rail11 = r.read_bool(bytes)?,
                Ok(96) => msg.rail12 = r.read_bool(bytes)?,
                Ok(104) => msg.rail13 = r.read_bool(bytes)?,
                Ok(112) => msg.rail14 = r.read_bool(bytes)?,
                Ok(120) => msg.rail15 = r.read_bool(bytes)?,
                Ok(128) => msg.rail16 = r.read_bool(bytes)?,
                Ok(136) => msg.hpwr1 = r.read_bool(bytes)?,
                Ok(144) => msg.hpwr2 = r.read_bool(bytes)?,
                Ok(152) => msg.hpwrEn = r.read_bool(bytes)?,
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for RailSOH {
    fn get_size(&self) -> usize {
        0
        + if self.rail1 == false { 0 } else { 1 + sizeof_varint(*(&self.rail1) as u64) }
        + if self.rail2 == false { 0 } else { 1 + sizeof_varint(*(&self.rail2) as u64) }
        + if self.rail3 == false { 0 } else { 1 + sizeof_varint(*(&self.rail3) as u64) }
        + if self.rail4 == false { 0 } else { 1 + sizeof_varint(*(&self.rail4) as u64) }
        + if self.rail5 == false { 0 } else { 1 + sizeof_varint(*(&self.rail5) as u64) }
        + if self.rail6 == false { 0 } else { 1 + sizeof_varint(*(&self.rail6) as u64) }
        + if self.rail7 == false { 0 } else { 1 + sizeof_varint(*(&self.rail7) as u64) }
        + if self.rail8 == false { 0 } else { 1 + sizeof_varint(*(&self.rail8) as u64) }
        + if self.rail9 == false { 0 } else { 1 + sizeof_varint(*(&self.rail9) as u64) }
        + if self.rail10 == false { 0 } else { 1 + sizeof_varint(*(&self.rail10) as u64) }
        + if self.rail11 == false { 0 } else { 1 + sizeof_varint(*(&self.rail11) as u64) }
        + if self.rail12 == false { 0 } else { 1 + sizeof_varint(*(&self.rail12) as u64) }
        + if self.rail13 == false { 0 } else { 1 + sizeof_varint(*(&self.rail13) as u64) }
        + if self.rail14 == false { 0 } else { 1 + sizeof_varint(*(&self.rail14) as u64) }
        + if self.rail15 == false { 0 } else { 1 + sizeof_varint(*(&self.rail15) as u64) }
        + if self.rail16 == false { 0 } else { 2 + sizeof_varint(*(&self.rail16) as u64) }
        + if self.hpwr1 == false { 0 } else { 2 + sizeof_varint(*(&self.hpwr1) as u64) }
        + if self.hpwr2 == false { 0 } else { 2 + sizeof_varint(*(&self.hpwr2) as u64) }
        + if self.hpwrEn == false { 0 } else { 2 + sizeof_varint(*(&self.hpwrEn) as u64) }
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.rail1 != false { w.write_with_tag(8, |w| w.write_bool(*&self.rail1))?; }
        if self.rail2 != false { w.write_with_tag(16, |w| w.write_bool(*&self.rail2))?; }
        if self.rail3 != false { w.write_with_tag(24, |w| w.write_bool(*&self.rail3))?; }
        if self.rail4 != false { w.write_with_tag(32, |w| w.write_bool(*&self.rail4))?; }
        if self.rail5 != false { w.write_with_tag(40, |w| w.write_bool(*&self.rail5))?; }
        if self.rail6 != false { w.write_with_tag(48, |w| w.write_bool(*&self.rail6))?; }
        if self.rail7 != false { w.write_with_tag(56, |w| w.write_bool(*&self.rail7))?; }
        if self.rail8 != false { w.write_with_tag(64, |w| w.write_bool(*&self.rail8))?; }
        if self.rail9 != false { w.write_with_tag(72, |w| w.write_bool(*&self.rail9))?; }
        if self.rail10 != false { w.write_with_tag(80, |w| w.write_bool(*&self.rail10))?; }
        if self.rail11 != false { w.write_with_tag(88, |w| w.write_bool(*&self.rail11))?; }
        if self.rail12 != false { w.write_with_tag(96, |w| w.write_bool(*&self.rail12))?; }
        if self.rail13 != false { w.write_with_tag(104, |w| w.write_bool(*&self.rail13))?; }
        if self.rail14 != false { w.write_with_tag(112, |w| w.write_bool(*&self.rail14))?; }
        if self.rail15 != false { w.write_with_tag(120, |w| w.write_bool(*&self.rail15))?; }
        if self.rail16 != false { w.write_with_tag(128, |w| w.write_bool(*&self.rail16))?; }
        if self.hpwr1 != false { w.write_with_tag(136, |w| w.write_bool(*&self.hpwr1))?; }
        if self.hpwr2 != false { w.write_with_tag(144, |w| w.write_bool(*&self.hpwr2))?; }
        if self.hpwrEn != false { w.write_with_tag(152, |w| w.write_bool(*&self.hpwrEn))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct RadioSOH {
    pub batteryVoltage: Option<protos::no_std::BatteryVoltage>,
    pub solarVoltage: Option<protos::no_std::SolarVoltage>,
    pub batteryVoltageState: protos::no_std::BatteryVoltageState,
    pub batteryManagerStates: Option<protos::no_std::BatteryManagerStates>,
    pub railSoh: Option<protos::no_std::RailSOH>,
}

impl<'a> MessageRead<'a> for RadioSOH {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(10) => msg.batteryVoltage = Some(r.read_message::<protos::no_std::BatteryVoltage>(bytes)?),
                Ok(18) => msg.solarVoltage = Some(r.read_message::<protos::no_std::SolarVoltage>(bytes)?),
                Ok(24) => msg.batteryVoltageState = r.read_enum(bytes)?,
                Ok(34) => msg.batteryManagerStates = Some(r.read_message::<protos::no_std::BatteryManagerStates>(bytes)?),
                Ok(42) => msg.railSoh = Some(r.read_message::<protos::no_std::RailSOH>(bytes)?),
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for RadioSOH {
    fn get_size(&self) -> usize {
        0
        + self.batteryVoltage.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + self.solarVoltage.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + if self.batteryVoltageState == protos::no_std::BatteryVoltageState::BothHigh { 0 } else { 1 + sizeof_varint(*(&self.batteryVoltageState) as u64) }
        + self.batteryManagerStates.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
        + self.railSoh.as_ref().map_or(0, |m| 1 + sizeof_len((m).get_size()))
    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if let Some(ref s) = self.batteryVoltage { w.write_with_tag(10, |w| w.write_message(s))?; }
        if let Some(ref s) = self.solarVoltage { w.write_with_tag(18, |w| w.write_message(s))?; }
        if self.batteryVoltageState != protos::no_std::BatteryVoltageState::BothHigh { w.write_with_tag(24, |w| w.write_enum(*&self.batteryVoltageState as i32))?; }
        if let Some(ref s) = self.batteryManagerStates { w.write_with_tag(34, |w| w.write_message(s))?; }
        if let Some(ref s) = self.railSoh { w.write_with_tag(42, |w| w.write_message(s))?; }
        Ok(())
    }
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct RadioTelemetry {
    pub tid: protos::no_std::TelemetryID,
    pub message: protos::no_std::mod_RadioTelemetry::OneOfmessage,
}

impl<'a> MessageRead<'a> for RadioTelemetry {
    fn from_reader(r: &mut BytesReader, bytes: &'a [u8]) -> Result<Self> {
        let mut msg = Self::default();
        while !r.is_eof() {
            match r.next_tag(bytes) {
                Ok(8) => msg.tid = r.read_enum(bytes)?,
                Ok(18) => msg.message = protos::no_std::mod_RadioTelemetry::OneOfmessage::soh(r.read_message::<protos::no_std::RadioSOH>(bytes)?),
                Ok(t) => { r.read_unknown(bytes, t)?; }
                Err(e) => return Err(e),
            }
        }
        Ok(msg)
    }
}

impl MessageWrite for RadioTelemetry {
    fn get_size(&self) -> usize {
        0
        + if self.tid == protos::no_std::TelemetryID::SOH { 0 } else { 1 + sizeof_varint(*(&self.tid) as u64) }
        + match self.message {
            protos::no_std::mod_RadioTelemetry::OneOfmessage::soh(ref m) => 1 + sizeof_len((m).get_size()),
            protos::no_std::mod_RadioTelemetry::OneOfmessage::None => 0,
    }    }

    fn write_message<W: WriterBackend>(&self, w: &mut Writer<W>) -> Result<()> {
        if self.tid != protos::no_std::TelemetryID::SOH { w.write_with_tag(8, |w| w.write_enum(*&self.tid as i32))?; }
        match self.message {            protos::no_std::mod_RadioTelemetry::OneOfmessage::soh(ref m) => { w.write_with_tag(18, |w| w.write_message(m))? },
            protos::no_std::mod_RadioTelemetry::OneOfmessage::None => {},
    }        Ok(())
    }
}

pub mod mod_RadioTelemetry {

use super::*;

#[derive(Debug, PartialEq, Clone)]
pub enum OneOfmessage {
    soh(protos::no_std::RadioSOH),
    None,
}

impl Default for OneOfmessage {
    fn default() -> Self {
        OneOfmessage::None
    }
}

}

