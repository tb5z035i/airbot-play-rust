use crate::protocol::ProtocolError;
use crate::types::{
    DecodedFrame, FrameKind, ParamDefinition, ParamType, ParamValue, ProtocolNode,
    ProtocolNodeKind, RawCanFrame,
};

pub const CMD_GET_BIAS: u32 = 0x000;
pub const CMD_SET_BIAS: u32 = 0x080;
pub const CMD_RET_BIAS: u32 = 0x100;

const LED_EFFECT: ParamDefinition =
    ParamDefinition::new(0x15, "led_effect", ParamType::UInt8Be, 1, false, true);
const BUTTON_STATE: ParamDefinition =
    ParamDefinition::new(0x13, "button_state", ParamType::UInt32Le, 1, true, false);

#[derive(Debug, Clone, Copy)]
pub struct PlayLedProtocol {
    board_id: u16,
}

impl PlayLedProtocol {
    pub fn new(board_id: u16) -> Self {
        Self { board_id }
    }

    pub fn generate_led_effect(&self, effect: u8) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let payload = [LED_EFFECT.id, 0x01, effect, 0x00, 0x00, 0x00];
        Ok(vec![
            RawCanFrame::new(self.board_id as u32 | CMD_SET_BIAS, &payload)
                .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    pub fn inspect(&self, frame: &RawCanFrame) -> Option<DecodedFrame> {
        if frame.can_id == self.board_id as u32 | CMD_SET_BIAS
            && frame.can_dlc == 6
            && frame.data[0] == LED_EFFECT.id
        {
            return Some(DecodedFrame::ParamRequest {
                node: ProtocolNode {
                    kind: ProtocolNodeKind::PlayLed,
                    id: self.board_id,
                },
                kind: FrameKind::LedEffectReq,
                name: LED_EFFECT.name.to_owned(),
                value: Some(ParamValue::U8(frame.data[2])),
            });
        }

        None
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PlayButtonProtocol {
    board_id: u16,
}

impl PlayButtonProtocol {
    pub fn new(board_id: u16) -> Self {
        Self { board_id }
    }

    pub fn inspect(&self, frame: &RawCanFrame) -> Option<DecodedFrame> {
        if frame.can_id == self.board_id as u32 | CMD_RET_BIAS
            && frame.can_dlc >= 6
            && frame.data[0] == BUTTON_STATE.id
        {
            let value =
                u32::from_le_bytes([frame.data[2], frame.data[3], frame.data[4], frame.data[5]]);
            return Some(DecodedFrame::BoardEvent {
                node: ProtocolNode {
                    kind: ProtocolNodeKind::PlayButton,
                    id: self.board_id,
                },
                name: BUTTON_STATE.name.to_owned(),
                value: ParamValue::U32(value),
            });
        }
        None
    }
}
