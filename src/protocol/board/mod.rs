pub mod gpio;
pub mod play_base;
pub mod play_end;

use crate::protocol::{
    MultiFloatAssembler, MultiStringAssembler, ProtocolError, deserialize_param, param_by_id,
    param_by_name, serialize_param,
};
use crate::types::{
    DecodedFrame, FrameKind, ParamDefinition, ParamType, ParamValue, ProtocolNode, RawCanFrame,
};
use std::collections::BTreeMap;

pub const PARAM_GET_ARB_ID_BIAS: u32 = 0x000;
pub const PARAM_SET_ARB_ID_BIAS: u32 = 0x080;
pub const PARAM_RET_ARB_ID_BIAS: u32 = 0x100;

#[derive(Debug, Default)]
pub struct BoardAssemblerState {
    pub strings: BTreeMap<u8, MultiStringAssembler>,
    pub floats: BTreeMap<u8, MultiFloatAssembler>,
}

pub trait BoardProtocol {
    fn node(&self) -> ProtocolNode;
    fn definitions(&self) -> &[ParamDefinition];
    fn state(&mut self) -> &mut BoardAssemblerState;

    fn inspect(&mut self, frame: &RawCanFrame) -> Option<DecodedFrame> {
        let node = self.node();
        let board_id = node.id as u32;

        if frame.can_id == board_id | PARAM_GET_ARB_ID_BIAS && frame.can_dlc == 1 {
            let param_id = frame.data[0];
            let definition = param_by_id(self.definitions(), param_id).copied();
            let name = definition
                .map(|definition| definition.name.to_owned())
                .unwrap_or_else(|| format!("param_{param_id:02X}"));
            return Some(DecodedFrame::ParamRequest {
                node,
                kind: FrameKind::GetParamReq,
                name,
                value: None,
            });
        }

        if frame.can_id == board_id | PARAM_SET_ARB_ID_BIAS && frame.can_dlc >= 2 {
            let param_id = frame.data[0];
            let definition = param_by_id(self.definitions(), param_id).copied();
            let name = definition
                .map(|definition| definition.name.to_owned())
                .unwrap_or_else(|| format!("param_{param_id:02X}"));

            let value = definition.and_then(|definition| {
                if definition.param_type == ParamType::String {
                    Some(ParamValue::String(
                        String::from_utf8_lossy(&frame.data[2..frame.can_dlc as usize]).to_string(),
                    ))
                } else if definition.param_type == ParamType::FloatLeMulti {
                    None
                } else {
                    let mut bytes = [0_u8; 4];
                    let payload = &frame.data[2..frame.can_dlc as usize];
                    bytes[..payload.len()].copy_from_slice(payload);
                    deserialize_param(&definition, bytes).ok()
                }
            });

            return Some(DecodedFrame::ParamRequest {
                node,
                kind: FrameKind::SetParamReq,
                name,
                value,
            });
        }

        if frame.can_id == board_id | PARAM_RET_ARB_ID_BIAS && frame.can_dlc >= 2 {
            let param_id = frame.data[0];
            let segment_index = frame.data[1];
            let mut bytes = [0_u8; 4];
            let payload = &frame.data[2..usize::min(frame.can_dlc as usize, 6)];
            bytes[..payload.len()].copy_from_slice(payload);
            let definition = param_by_id(self.definitions(), param_id).copied();
            let name = definition
                .map(|definition| definition.name.to_owned())
                .unwrap_or_else(|| format!("param_{param_id:02X}"));

            if let Some(definition) = definition {
                match definition.param_type {
                    ParamType::String => {
                        let assembler = self.state().strings.entry(param_id).or_default();
                        if let Some(value) = assembler.push(segment_index, definition.length, bytes)
                        {
                            let mut values = BTreeMap::new();
                            values.insert(name, ParamValue::String(value));
                            return Some(DecodedFrame::ParamResponse {
                                node,
                                kind: FrameKind::GetParamResp,
                                values,
                            });
                        }
                        return None;
                    }
                    ParamType::FloatLeMulti => {
                        let assembler = self.state().floats.entry(param_id).or_default();
                        if let Some(values_vec) = assembler.push(
                            segment_index,
                            definition.length,
                            f32::from_le_bytes(bytes),
                        ) {
                            let mut values = BTreeMap::new();
                            values.insert(name, ParamValue::FloatVec(values_vec));
                            return Some(DecodedFrame::ParamResponse {
                                node,
                                kind: FrameKind::GetParamResp,
                                values,
                            });
                        }
                        return None;
                    }
                    _ => {
                        let mut values = BTreeMap::new();
                        let value = deserialize_param(&definition, bytes)
                            .unwrap_or(ParamValue::Raw4(bytes));
                        values.insert(name.clone(), value.clone());
                        if name == "button" || name == "button_state" {
                            return Some(DecodedFrame::BoardEvent { node, name, value });
                        }
                        return Some(DecodedFrame::ParamResponse {
                            node,
                            kind: FrameKind::GetParamResp,
                            values,
                        });
                    }
                }
            }

            let mut values = BTreeMap::new();
            values.insert(name, ParamValue::Raw4(bytes));
            return Some(DecodedFrame::ParamResponse {
                node,
                kind: FrameKind::GetParamResp,
                values,
            });
        }

        None
    }

    fn generate_param_get(&self, name: &str) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let definition = param_by_name(self.definitions(), name)?;
        if !definition.readable {
            return Err(ProtocolError::UnknownParameter(name.to_owned()));
        }
        Ok(vec![
            RawCanFrame::new(
                self.node().id as u32 | PARAM_GET_ARB_ID_BIAS,
                &[definition.id],
            )
            .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    fn generate_param_set(
        &self,
        name: &str,
        value: &ParamValue,
    ) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let definition = param_by_name(self.definitions(), name)?;
        if !definition.writable {
            return Err(ProtocolError::ParameterNotWritable(name.to_owned()));
        }

        if definition.param_type == ParamType::String {
            let ParamValue::String(text) = value else {
                return Err(ProtocolError::ParameterTypeMismatch(name.to_owned()));
            };

            let expected_len = definition.length as usize * 4;
            if text.len() != expected_len {
                return Err(ProtocolError::InvalidFrame(format!(
                    "string param `{name}` expects {} bytes, got {}",
                    expected_len,
                    text.len()
                )));
            }

            let mut frames = Vec::with_capacity(definition.length as usize);
            for (index, chunk) in text.as_bytes().chunks(4).enumerate() {
                let mut payload = [0_u8; 6];
                payload[0] = definition.id;
                payload[1] = (index + 1) as u8;
                payload[2..2 + chunk.len()].copy_from_slice(chunk);
                frames.push(
                    RawCanFrame::new(self.node().id as u32 | PARAM_SET_ARB_ID_BIAS, &payload)
                        .map_err(ProtocolError::InvalidFrame)?,
                );
            }
            return Ok(frames);
        }

        if definition.param_type == ParamType::FloatLeMulti {
            return Err(ProtocolError::UnsupportedFrameKind);
        }

        let bytes = serialize_param(definition, value)?;
        let mut payload = vec![definition.id, 0x01];
        payload.extend_from_slice(&bytes[..param_payload_len(definition.param_type)]);
        Ok(vec![
            RawCanFrame::new(self.node().id as u32 | PARAM_SET_ARB_ID_BIAS, &payload)
                .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    fn generate_ping(&self) -> Result<Vec<RawCanFrame>, ProtocolError> {
        self.generate_param_get("board_id")
    }
}

fn param_payload_len(param_type: ParamType) -> usize {
    match param_type {
        ParamType::UInt8Le | ParamType::UInt8Be | ParamType::Int8Le => 1,
        ParamType::UInt16Le | ParamType::Int16Le | ParamType::UInt16Be => 2,
        ParamType::UInt32Le
        | ParamType::Int32Le
        | ParamType::Float32Le
        | ParamType::UInt32Be
        | ParamType::Float32Be
        | ParamType::Bytes4
        | ParamType::Untouched => 4,
        _ => 4,
    }
}
