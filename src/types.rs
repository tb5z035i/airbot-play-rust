use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fmt;

pub const CLASSIC_CAN_MAX_DLEN: usize = 8;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub struct RawCanFrame {
    pub can_id: u32,
    pub can_dlc: u8,
    pub data: [u8; CLASSIC_CAN_MAX_DLEN],
}

impl RawCanFrame {
    pub fn new(can_id: u32, payload: &[u8]) -> Result<Self, String> {
        if payload.len() > CLASSIC_CAN_MAX_DLEN {
            return Err(format!(
                "classic CAN payload too long: {} > {}",
                payload.len(),
                CLASSIC_CAN_MAX_DLEN
            ));
        }

        let mut data = [0_u8; CLASSIC_CAN_MAX_DLEN];
        data[..payload.len()].copy_from_slice(payload);
        Ok(Self {
            can_id,
            can_dlc: payload.len() as u8,
            data,
        })
    }

    pub fn payload(&self) -> &[u8] {
        &self.data[..self.can_dlc as usize]
    }
}

impl fmt::Display for RawCanFrame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "0x{:03X} [{}]", self.can_id, self.can_dlc)?;
        for byte in self.payload() {
            write!(f, " {:02X}", byte)?;
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum ProtocolNodeKind {
    OdMotor,
    DmMotor,
    PlayBaseBoard,
    PlayEndBoard,
    PlayLed,
    PlayButton,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub struct ProtocolNode {
    pub kind: ProtocolNodeKind,
    pub id: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum FrameKind {
    MitReq,
    CsvReq,
    PvtReq,
    PvReq,
    GetParamReq,
    SetParamReq,
    PersistParamReq,
    PingReq,
    EnableReq,
    DisableReq,
    SetZeroReq,
    ResetErrReq,
    MotionFeedback,
    GetParamResp,
    SetParamResp,
    PersistParamResp,
    LedEffectReq,
    ButtonStateResp,
    Void,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ParamType {
    UInt8Le,
    Int8Le,
    UInt16Le,
    Int16Le,
    UInt32Le,
    Int32Le,
    Float32Le,
    UInt8Be,
    UInt16Be,
    UInt32Be,
    Float32Be,
    Bytes4,
    String,
    FloatLeMulti,
    Untouched,
    Invalid,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ParamValue {
    U8(u8),
    I8(i8),
    U16(u16),
    I16(i16),
    U32(u32),
    I32(i32),
    F32(f32),
    Bytes4([u8; 4]),
    String(String),
    FloatVec(Vec<f32>),
    Raw4([u8; 4]),
    Empty,
}

impl ParamValue {
    pub fn type_matches(&self, param_type: ParamType) -> bool {
        matches!(
            (self, param_type),
            (Self::U8(_), ParamType::UInt8Le | ParamType::UInt8Be)
                | (Self::I8(_), ParamType::Int8Le)
                | (Self::U16(_), ParamType::UInt16Le | ParamType::UInt16Be)
                | (Self::I16(_), ParamType::Int16Le)
                | (Self::U32(_), ParamType::UInt32Le | ParamType::UInt32Be)
                | (Self::I32(_), ParamType::Int32Le)
                | (Self::F32(_), ParamType::Float32Le | ParamType::Float32Be)
                | (Self::Bytes4(_), ParamType::Bytes4)
                | (Self::String(_), ParamType::String)
                | (Self::FloatVec(_), ParamType::FloatLeMulti)
                | (Self::Raw4(_), ParamType::Untouched)
                | (Self::Empty, ParamType::Invalid)
        )
    }
}

impl fmt::Display for ParamValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::U8(value) => write!(f, "{value}"),
            Self::I8(value) => write!(f, "{value}"),
            Self::U16(value) => write!(f, "{value}"),
            Self::I16(value) => write!(f, "{value}"),
            Self::U32(value) => write!(f, "{value}"),
            Self::I32(value) => write!(f, "{value}"),
            Self::F32(value) => write!(f, "{value:.6}"),
            Self::Bytes4(bytes) | Self::Raw4(bytes) => {
                write!(
                    f,
                    "{:02X} {:02X} {:02X} {:02X}",
                    bytes[0], bytes[1], bytes[2], bytes[3]
                )
            }
            Self::String(value) => write!(f, "{value}"),
            Self::FloatVec(values) => {
                let parts = values
                    .iter()
                    .map(|value| format!("{value:.6}"))
                    .collect::<Vec<_>>();
                write!(f, "{}", parts.join(","))
            }
            Self::Empty => write!(f, "<empty>"),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ParamDefinition {
    pub id: u8,
    pub name: &'static str,
    pub param_type: ParamType,
    pub length: u8,
    pub readable: bool,
    pub writable: bool,
}

impl ParamDefinition {
    pub const fn new(
        id: u8,
        name: &'static str,
        param_type: ParamType,
        length: u8,
        readable: bool,
        writable: bool,
    ) -> Self {
        Self {
            id,
            name,
            param_type,
            length,
            readable,
            writable,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotorCommand {
    pub pos: f64,
    pub vel: f64,
    pub eff: f64,
    pub mit_kp: f64,
    pub mit_kd: f64,
    pub current_threshold: f64,
}

impl Default for MotorCommand {
    fn default() -> Self {
        Self {
            pos: 0.0,
            vel: 0.0,
            eff: 0.0,
            mit_kp: 0.0,
            mit_kd: 0.0,
            current_threshold: 0.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotorState {
    pub is_valid: bool,
    pub joint_id: u16,
    pub pos: f64,
    pub vel: f64,
    pub eff: f64,
    pub motor_temp: u8,
    pub mos_temp: u8,
    pub error_id: u8,
}

impl Default for MotorState {
    fn default() -> Self {
        Self {
            is_valid: true,
            joint_id: 0,
            pos: 0.0,
            vel: 0.0,
            eff: 0.0,
            motor_temp: 0,
            mos_temp: 0,
            error_id: 0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum DecodedFrame {
    MotionCommand {
        node: ProtocolNode,
        kind: FrameKind,
        command: MotorCommand,
    },
    MotionFeedback {
        node: ProtocolNode,
        state: MotorState,
    },
    ParamRequest {
        node: ProtocolNode,
        kind: FrameKind,
        name: String,
        value: Option<ParamValue>,
    },
    ParamResponse {
        node: ProtocolNode,
        kind: FrameKind,
        values: BTreeMap<String, ParamValue>,
    },
    LifecycleCommand {
        node: ProtocolNode,
        kind: FrameKind,
    },
    BoardEvent {
        node: ProtocolNode,
        name: String,
        value: ParamValue,
    },
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DiscoveredInstance {
    pub interface: String,
    pub identified_as: Option<String>,
    pub product_sn: Option<String>,
    pub pcba_sn: Option<String>,
    pub mounted_eef: Option<String>,
    pub metadata: BTreeMap<String, String>,
}
