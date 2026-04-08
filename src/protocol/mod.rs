pub mod board;
pub mod motor;

use crate::types::{DecodedFrame, ParamDefinition, ParamType, ParamValue, RawCanFrame};
use std::collections::BTreeMap;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ProtocolError {
    #[error("unsupported frame kind for this codec")]
    UnsupportedFrameKind,
    #[error("unknown parameter `{0}`")]
    UnknownParameter(String),
    #[error("parameter `{0}` is not writable")]
    ParameterNotWritable(String),
    #[error("parameter type mismatch for `{0}`")]
    ParameterTypeMismatch(String),
    #[error("invalid frame: {0}")]
    InvalidFrame(String),
}

pub trait FrameInspector {
    fn inspect(&mut self, frame: &RawCanFrame) -> Option<DecodedFrame>;
}

pub fn limit(value: f32, max_abs: f32) -> f32 {
    value.clamp(-max_abs, max_abs)
}

pub fn range_map_f32(value: f32, in_min: f32, in_max: f32, out_min: u32, out_max: u32) -> u32 {
    let span_in = in_max - in_min;
    let span_out = out_max as f32 - out_min as f32;
    if span_in.abs() < f32::EPSILON {
        return out_min;
    }

    let normalized = ((value - in_min) / span_in).clamp(0.0, 1.0);
    (out_min as f32 + normalized * span_out).round() as u32
}

pub fn inverse_range_map_f32(
    value: u32,
    in_min: u32,
    in_max: u32,
    out_min: f32,
    out_max: f32,
) -> f32 {
    let span_in = in_max as f32 - in_min as f32;
    let span_out = out_max - out_min;
    if span_in.abs() < f32::EPSILON {
        return out_min;
    }

    let normalized = (value as f32 - in_min as f32) / span_in;
    out_min + normalized * span_out
}

pub fn f32_to_le_bytes(value: f32) -> [u8; 4] {
    value.to_le_bytes()
}

pub fn f32_to_be_bytes(value: f32) -> [u8; 4] {
    value.to_be_bytes()
}

pub fn read_f32_le(bytes: [u8; 4]) -> f32 {
    f32::from_le_bytes(bytes)
}

pub fn read_f32_be(bytes: [u8; 4]) -> f32 {
    f32::from_be_bytes(bytes)
}

pub fn read_u16_le(bytes: [u8; 2]) -> u16 {
    u16::from_le_bytes(bytes)
}

pub fn read_u16_be(bytes: [u8; 2]) -> u16 {
    u16::from_be_bytes(bytes)
}

pub fn read_i16_le(bytes: [u8; 2]) -> i16 {
    i16::from_le_bytes(bytes)
}

pub fn read_u32_le(bytes: [u8; 4]) -> u32 {
    u32::from_le_bytes(bytes)
}

pub fn read_u32_be(bytes: [u8; 4]) -> u32 {
    u32::from_be_bytes(bytes)
}

pub fn read_i32_le(bytes: [u8; 4]) -> i32 {
    i32::from_le_bytes(bytes)
}

pub fn param_by_name<'a>(
    params: &'a [ParamDefinition],
    name: &str,
) -> Result<&'a ParamDefinition, ProtocolError> {
    params
        .iter()
        .find(|definition| definition.name == name)
        .ok_or_else(|| ProtocolError::UnknownParameter(name.to_owned()))
}

pub fn param_by_id(params: &[ParamDefinition], id: u8) -> Option<&ParamDefinition> {
    params.iter().find(|definition| definition.id == id)
}

pub fn unknown_param_value(data: [u8; 4]) -> ParamValue {
    ParamValue::Raw4(data)
}

pub fn unknown_param_name(id: u8) -> String {
    format!("param_{id:02X}")
}

pub fn deserialize_param(
    definition: &ParamDefinition,
    data: [u8; 4],
) -> Result<ParamValue, ProtocolError> {
    use ParamType::*;

    let value = match definition.param_type {
        UInt8Le | UInt8Be => ParamValue::U8(data[0]),
        Int8Le => ParamValue::I8(data[0] as i8),
        UInt16Le => ParamValue::U16(read_u16_le([data[0], data[1]])),
        Int16Le => ParamValue::I16(read_i16_le([data[0], data[1]])),
        UInt16Be => ParamValue::U16(read_u16_be([data[0], data[1]])),
        UInt32Le => ParamValue::U32(read_u32_le(data)),
        Int32Le => ParamValue::I32(read_i32_le(data)),
        UInt32Be => ParamValue::U32(read_u32_be(data)),
        Float32Le => ParamValue::F32(read_f32_le(data)),
        Float32Be => ParamValue::F32(read_f32_be(data)),
        Bytes4 => ParamValue::Bytes4(data),
        Untouched => ParamValue::Raw4(data),
        String | FloatLeMulti | Invalid => {
            return Err(ProtocolError::InvalidFrame(format!(
                "cannot deserialize {:?} with fixed 4-byte helper",
                definition.param_type
            )));
        }
    };

    Ok(value)
}

pub fn serialize_param(
    definition: &ParamDefinition,
    value: &ParamValue,
) -> Result<[u8; 4], ProtocolError> {
    if !value.type_matches(definition.param_type) {
        return Err(ProtocolError::ParameterTypeMismatch(
            definition.name.to_owned(),
        ));
    }

    use ParamType::*;
    let bytes = match (definition.param_type, value) {
        (UInt8Le | UInt8Be, ParamValue::U8(value)) => [*value, 0, 0, 0],
        (Int8Le, ParamValue::I8(value)) => [*value as u8, 0, 0, 0],
        (UInt16Le, ParamValue::U16(value)) => {
            let raw = value.to_le_bytes();
            [raw[0], raw[1], 0, 0]
        }
        (Int16Le, ParamValue::I16(value)) => {
            let raw = value.to_le_bytes();
            [raw[0], raw[1], 0, 0]
        }
        (UInt16Be, ParamValue::U16(value)) => {
            let raw = value.to_be_bytes();
            [raw[0], raw[1], 0, 0]
        }
        (UInt32Le, ParamValue::U32(value)) => value.to_le_bytes(),
        (Int32Le, ParamValue::I32(value)) => value.to_le_bytes(),
        (UInt32Be, ParamValue::U32(value)) => value.to_be_bytes(),
        (Float32Le, ParamValue::F32(value)) => value.to_le_bytes(),
        (Float32Be, ParamValue::F32(value)) => value.to_be_bytes(),
        (Bytes4, ParamValue::Bytes4(value)) => *value,
        (Untouched, ParamValue::Raw4(value)) => *value,
        _ => {
            return Err(ProtocolError::ParameterTypeMismatch(
                definition.name.to_owned(),
            ));
        }
    };

    Ok(bytes)
}

#[derive(Debug)]
pub struct MultiStringAssembler {
    received_mask: u32,
    buffer: [u8; 128],
}

impl MultiStringAssembler {
    pub fn new() -> Self {
        Self {
            received_mask: 0,
            buffer: [0_u8; 128],
        }
    }

    pub fn push(&mut self, index: u8, expected_segments: u8, data: [u8; 4]) -> Option<String> {
        let frame_index = index.saturating_sub(1);
        self.received_mask |= 1_u32 << frame_index;
        let start = frame_index as usize * 4;
        self.buffer[start..start + 4].copy_from_slice(&data);

        let expected_mask = if expected_segments >= 32 {
            u32::MAX
        } else {
            (1_u32 << expected_segments) - 1
        };

        if self.received_mask == expected_mask {
            self.received_mask = 0;
            Some(
                String::from_utf8_lossy(&self.buffer[..expected_segments as usize * 4])
                    .trim_end_matches('\0')
                    .to_string(),
            )
        } else {
            None
        }
    }
}

impl Default for MultiStringAssembler {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Default)]
pub struct MultiFloatAssembler {
    values: BTreeMap<u8, f32>,
}

impl MultiFloatAssembler {
    pub fn push(&mut self, index: u8, expected_segments: u8, value: f32) -> Option<Vec<f32>> {
        let frame_index = index.saturating_sub(1);
        self.values.insert(frame_index, value);
        if self.values.len() == expected_segments as usize {
            let mut result = Vec::with_capacity(self.values.len());
            for (_, value) in self.values.iter() {
                result.push(*value);
            }
            self.values.clear();
            Some(result)
        } else {
            None
        }
    }
}
