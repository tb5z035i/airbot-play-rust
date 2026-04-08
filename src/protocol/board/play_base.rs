use super::{BoardAssemblerState, BoardProtocol};
use crate::types::{ParamDefinition, ParamType, ProtocolNode, ProtocolNodeKind};

const BASE_PARAMS: &[ParamDefinition] = &[
    ParamDefinition::new(0x00, "board_id", ParamType::UInt32Le, 1, true, false),
    ParamDefinition::new(0x01, "hardware_version", ParamType::Bytes4, 1, true, false),
    ParamDefinition::new(0x02, "firmware_version", ParamType::Bytes4, 1, true, false),
    ParamDefinition::new(0x03, "pcba_sn", ParamType::String, 4, true, true),
    ParamDefinition::new(0x04, "product_sn", ParamType::String, 4, true, true),
    ParamDefinition::new(0x07, "pcba_name", ParamType::String, 6, true, false),
    ParamDefinition::new(0x0A, "motor_type", ParamType::UInt8Le, 1, true, false),
    ParamDefinition::new(
        0x0B,
        "self_check_status",
        ParamType::UInt32Le,
        1,
        true,
        false,
    ),
    ParamDefinition::new(
        0x0C,
        "zero_position",
        ParamType::FloatLeMulti,
        6,
        true,
        false,
    ),
    ParamDefinition::new(
        0x11,
        "last_position",
        ParamType::FloatLeMulti,
        7,
        true,
        false,
    ),
    ParamDefinition::new(
        0x12,
        "current_voltage",
        ParamType::Float32Le,
        1,
        true,
        false,
    ),
    ParamDefinition::new(0x14, "manufacture_flag", ParamType::UInt32Le, 1, true, true),
    ParamDefinition::new(
        0x17,
        "gravity_comp_param",
        ParamType::FloatLeMulti,
        24,
        true,
        true,
    ),
];

#[derive(Debug, Default)]
pub struct PlayBaseBoardProtocol {
    state: BoardAssemblerState,
}

impl PlayBaseBoardProtocol {
    pub const BOARD_ID: u16 = 0x00;

    pub fn new() -> Self {
        Self::default()
    }
}

impl BoardProtocol for PlayBaseBoardProtocol {
    fn node(&self) -> ProtocolNode {
        ProtocolNode {
            kind: ProtocolNodeKind::PlayBaseBoard,
            id: Self::BOARD_ID,
        }
    }

    fn definitions(&self) -> &[ParamDefinition] {
        BASE_PARAMS
    }

    fn state(&mut self) -> &mut BoardAssemblerState {
        &mut self.state
    }
}
