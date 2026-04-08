use super::{BoardAssemblerState, BoardProtocol};
use crate::types::{ParamDefinition, ParamType, ProtocolNode, ProtocolNodeKind};

const END_PARAMS: &[ParamDefinition] = &[
    ParamDefinition::new(0x00, "board_id", ParamType::UInt32Le, 1, true, false),
    ParamDefinition::new(0x01, "hardware_version", ParamType::Bytes4, 1, true, false),
    ParamDefinition::new(0x02, "firmware_version", ParamType::Bytes4, 1, true, false),
    ParamDefinition::new(0x03, "pcba_sn", ParamType::String, 4, true, true),
    ParamDefinition::new(0x04, "product_sn", ParamType::String, 4, true, true),
    ParamDefinition::new(0x05, "eef_type", ParamType::UInt32Le, 4, true, false),
    ParamDefinition::new(0x07, "pcba_name", ParamType::String, 6, true, false),
    ParamDefinition::new(
        0x14,
        "manufacture_flag",
        ParamType::UInt32Le,
        1,
        true,
        false,
    ),
];

#[derive(Debug, Default)]
pub struct PlayEndBoardProtocol {
    state: BoardAssemblerState,
}

impl PlayEndBoardProtocol {
    pub const BOARD_ID: u16 = 0x08;

    pub fn new() -> Self {
        Self::default()
    }
}

impl BoardProtocol for PlayEndBoardProtocol {
    fn node(&self) -> ProtocolNode {
        ProtocolNode {
            kind: ProtocolNodeKind::PlayEndBoard,
            id: Self::BOARD_ID,
        }
    }

    fn definitions(&self) -> &[ParamDefinition] {
        END_PARAMS
    }

    fn state(&mut self) -> &mut BoardAssemblerState {
        &mut self.state
    }
}
