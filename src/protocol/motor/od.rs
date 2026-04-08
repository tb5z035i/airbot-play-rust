use super::MotorProtocol;
use crate::protocol::{
    MultiStringAssembler, ProtocolError, deserialize_param, inverse_range_map_f32, param_by_id,
    param_by_name, range_map_f32, serialize_param,
};
use crate::types::{
    DecodedFrame, FrameKind, MotorCommand, MotorState, ParamDefinition, ParamType, ParamValue,
    ProtocolNode, ProtocolNodeKind, RawCanFrame,
};
use std::collections::BTreeMap;
use std::f32::consts::PI;

const PARAM_GET_ARB_ID_BIAS: u32 = 0x000;
const PARAM_SET_ARB_ID_BIAS: u32 = 0x080;
const PARAM_RET_ARB_ID_BIAS: u32 = 0x100;

const KP_MIN: f32 = 0.0;
const KP_MAX: f32 = 500.0;
const KD_MIN: f32 = 0.0;
const KD_MAX: f32 = 50.0;
const POS_MIN: f32 = -12.5;
const POS_MAX: f32 = 12.5;
const VEL_MIN: f32 = -18.0;
const VEL_MAX: f32 = 18.0;
const TOR_MIN: f32 = -30.0;
const TOR_MAX: f32 = 30.0;

const PARAMS: &[ParamDefinition] = &[
    ParamDefinition::new(0x00, "board_id", ParamType::UInt32Be, 1, true, true),
    ParamDefinition::new(0x01, "hw_version", ParamType::Bytes4, 1, true, false),
    ParamDefinition::new(0x02, "fw_version", ParamType::Bytes4, 1, true, false),
    ParamDefinition::new(0x03, "board_sn", ParamType::String, 4, true, true),
    ParamDefinition::new(0x04, "product_sn", ParamType::String, 4, true, true),
    ParamDefinition::new(0x07, "board_name", ParamType::String, 5, true, false),
    ParamDefinition::new(0x14, "manufacture_flag", ParamType::UInt32Le, 1, true, true),
    ParamDefinition::new(0xE1, "position", ParamType::Float32Be, 1, true, false),
    ParamDefinition::new(0xE2, "velocity", ParamType::Float32Be, 1, true, false),
    ParamDefinition::new(0xE3, "torque", ParamType::Float32Be, 1, true, false),
    ParamDefinition::new(0xEB, "torque_factor", ParamType::UInt16Be, 1, true, false),
    ParamDefinition::new(0xED, "gear_ratio", ParamType::Float32Be, 1, true, false),
    ParamDefinition::new(0xF3, "timeout", ParamType::UInt32Be, 1, true, false),
];

#[derive(Debug, Default)]
pub struct OdProtocol {
    motor_id: u16,
    string_assemblers: BTreeMap<u8, MultiStringAssembler>,
    pub cur_thres_max: Option<f32>,
}

impl OdProtocol {
    pub fn new(motor_id: u16) -> Self {
        Self {
            motor_id,
            string_assemblers: BTreeMap::new(),
            cur_thres_max: Some(409.5),
        }
    }

    pub fn generate_mit(&self, command: &MotorCommand) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let kp_int = range_map_f32(command.mit_kp as f32, KP_MIN, KP_MAX, 0, 0x0FFF) as u16;
        let kd_int = range_map_f32(command.mit_kd as f32, KD_MIN, KD_MAX, 0, 0x01FF) as u16;
        let p_int = range_map_f32(command.pos as f32, POS_MIN, POS_MAX, 0, 0xFFFF) as u16;
        let v_int = range_map_f32(command.vel as f32, VEL_MIN, VEL_MAX, 0, 0x0FFF) as u16;
        let t_int = range_map_f32(command.eff as f32, TOR_MIN, TOR_MAX, 0, 0x0FFF) as u16;

        let payload = [
            ((kp_int >> 7) & 0x1F) as u8,
            (((kp_int & 0x7F) << 1) as u8) | (((kd_int & 0x100) >> 8) as u8),
            (kd_int & 0xFF) as u8,
            (p_int >> 8) as u8,
            (p_int & 0xFF) as u8,
            (v_int >> 4) as u8,
            (((v_int & 0x0F) << 4) as u8) | ((t_int >> 8) as u8),
            (t_int & 0xFF) as u8,
        ];

        Ok(vec![
            RawCanFrame::new(self.motor_id as u32, &payload)
                .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    pub fn generate_csv(&self, command: &MotorCommand) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let rpm = command.vel * 60.0 / 2.0 / std::f64::consts::PI;
        let vel_bytes = (rpm as f32).to_le_bytes();
        let cur_max = self.cur_thres_max.unwrap_or(409.5).clamp(0.0, 409.5);
        let cur_int = (cur_max * 10.0).floor() as u16;
        let payload = [
            0x41,
            vel_bytes[3],
            vel_bytes[2],
            vel_bytes[1],
            vel_bytes[0],
            (cur_int >> 8) as u8,
            (cur_int & 0xFF) as u8,
        ];
        Ok(vec![
            RawCanFrame::new(self.motor_id as u32, &payload)
                .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    pub fn generate_pv(&self, command: &MotorCommand) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let pos_le = ((command.pos as f32) * 180.0 / PI).to_le_bytes();
        let vel_rpm = (command.vel as f32) * 60.0 / PI / 2.0;
        let v_int = (vel_rpm * 10.0).floor().max(0.0) as u16;
        let cur_int = (command.current_threshold as f32)
            .clamp(0.0, 409.5)
            .mul_add(10.0, 0.0)
            .floor() as u16;

        let payload = [
            0x20 | (pos_le[3] >> 3),
            (pos_le[3] << 5) | (pos_le[2] >> 3),
            (pos_le[2] << 5) | (pos_le[1] >> 3),
            (pos_le[1] << 5) | (pos_le[0] >> 3),
            (pos_le[0] << 5) | ((v_int >> 10) as u8),
            ((v_int & 0x03FC) >> 2) as u8,
            (((v_int & 0x0003) << 6) as u8) | ((cur_int >> 6) as u8),
            (((cur_int & 0x003F) << 2) as u8) | 0x01,
        ];

        Ok(vec![
            RawCanFrame::new(self.motor_id as u32, &payload)
                .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    fn decode_mit_request(&self, frame: &RawCanFrame) -> DecodedFrame {
        let kp_int = (((frame.data[0] as u16) << 7) | ((frame.data[1] as u16) >> 1)) & 0x0FFF;
        let kd_int = ((((frame.data[1] & 0x01) as u16) << 8) | frame.data[2] as u16) & 0x01FF;
        let p_int = ((frame.data[3] as u16) << 8) | frame.data[4] as u16;
        let v_int = ((frame.data[5] as u16) << 4) | ((frame.data[6] as u16) >> 4);
        let t_int = (((frame.data[6] as u16) & 0x0F) << 8) | frame.data[7] as u16;

        DecodedFrame::MotionCommand {
            node: self.node(),
            kind: FrameKind::MitReq,
            command: MotorCommand {
                pos: inverse_range_map_f32(p_int as u32, 0, 0xFFFF, POS_MIN, POS_MAX) as f64,
                vel: inverse_range_map_f32(v_int as u32, 0, 0x0FFF, VEL_MIN, VEL_MAX) as f64,
                eff: inverse_range_map_f32(t_int as u32, 0, 0x0FFF, TOR_MIN, TOR_MAX) as f64,
                mit_kp: inverse_range_map_f32(kp_int as u32, 0, 0x0FFF, KP_MIN, KP_MAX) as f64,
                mit_kd: inverse_range_map_f32(kd_int as u32, 0, 0x01FF, KD_MIN, KD_MAX) as f64,
                current_threshold: 0.0,
            },
        }
    }

    fn decode_pv_request(&self, frame: &RawCanFrame) -> DecodedFrame {
        let pos_le = [
            ((frame.data[3] & 0x1F) << 3) | (frame.data[4] >> 5),
            ((frame.data[2] & 0x1F) << 3) | (frame.data[3] >> 5),
            ((frame.data[1] & 0x1F) << 3) | (frame.data[2] >> 5),
            ((frame.data[0] & 0x1F) << 3) | (frame.data[1] >> 5),
        ];
        let v_int = (((frame.data[4] & 0x03) as u16) << 10)
            | ((frame.data[5] as u16) << 2)
            | ((frame.data[6] as u16) >> 6);
        let cur_int = (((frame.data[6] & 0x3F) as u16) << 6) | ((frame.data[7] as u16) >> 2);

        DecodedFrame::MotionCommand {
            node: self.node(),
            kind: FrameKind::PvReq,
            command: MotorCommand {
                pos: (f32::from_le_bytes(pos_le) * PI / 180.0) as f64,
                vel: (v_int as f32 / 10.0 * 2.0 * PI / 60.0) as f64,
                eff: 0.0,
                mit_kp: 0.0,
                mit_kd: 0.0,
                current_threshold: (cur_int as f32 / 10.0) as f64,
            },
        }
    }
}

impl MotorProtocol for OdProtocol {
    fn node(&self) -> ProtocolNode {
        ProtocolNode {
            kind: ProtocolNodeKind::OdMotor,
            id: self.motor_id,
        }
    }

    fn definitions(&self) -> &[ParamDefinition] {
        PARAMS
    }

    fn inspect(&mut self, frame: &RawCanFrame) -> Option<DecodedFrame> {
        if frame.can_id == self.motor_id as u32
            && frame.can_dlc == 8
            && (frame.data[0] & 0xE0) == 0x00
        {
            return Some(self.decode_mit_request(frame));
        }

        if frame.can_id == self.motor_id as u32
            && frame.can_dlc == 8
            && (frame.data[0] & 0xE0) == 0x20
            && (frame.data[7] & 0x03) == 0x01
        {
            return Some(self.decode_pv_request(frame));
        }

        if frame.can_id == self.motor_id as u32
            && (frame.data[0] & 0xE0) == 0x20
            && frame.can_dlc >= 7
        {
            let error_code = frame.data[0] & 0x1F;
            let pos_int = ((frame.data[1] as u16) << 8) | frame.data[2] as u16;
            let spd_int = ((frame.data[3] as u16) << 4) | (((frame.data[4] & 0xF0) as u16) >> 4);
            let t_int = (((frame.data[4] & 0x0F) as u16) << 8) | frame.data[5] as u16;
            let motor_temp = ((frame.data[6] as f32 - 50.0) / 2.0).max(0.0) as u8;

            return Some(DecodedFrame::MotionFeedback {
                node: self.node(),
                state: MotorState {
                    is_valid: true,
                    joint_id: self.motor_id,
                    pos: inverse_range_map_f32(pos_int as u32, 0, 0xFFFF, POS_MIN, POS_MAX) as f64,
                    vel: inverse_range_map_f32(spd_int as u32, 0, 0x0FFF, VEL_MIN, VEL_MAX) as f64,
                    eff: inverse_range_map_f32(t_int as u32, 0, 0x0FFF, TOR_MIN, TOR_MAX) as f64,
                    motor_temp,
                    mos_temp: 0,
                    error_id: error_code,
                },
            });
        }

        if frame.can_id == self.motor_id as u32 | PARAM_GET_ARB_ID_BIAS && frame.can_dlc == 1 {
            let param_id = frame.data[0];
            let name = param_by_id(PARAMS, param_id)
                .map(|definition| definition.name.to_owned())
                .unwrap_or_else(|| format!("param_{param_id:02X}"));
            return Some(DecodedFrame::ParamRequest {
                node: self.node(),
                kind: FrameKind::GetParamReq,
                name,
                value: None,
            });
        }

        if frame.can_id == self.motor_id as u32 && frame.can_dlc == 2 && frame.data[0] == 0xE0 {
            let param_id = frame.data[1] + 0xE0;
            let name = param_by_id(PARAMS, param_id)
                .map(|definition| definition.name.to_owned())
                .unwrap_or_else(|| format!("param_{param_id:02X}"));
            return Some(DecodedFrame::ParamRequest {
                node: self.node(),
                kind: FrameKind::GetParamReq,
                name,
                value: None,
            });
        }

        if frame.can_id == self.motor_id as u32 | PARAM_SET_ARB_ID_BIAS && frame.can_dlc >= 2 {
            let param_id = frame.data[0];
            let definition = param_by_id(PARAMS, param_id);
            let name = definition
                .map(|definition| definition.name.to_owned())
                .unwrap_or_else(|| format!("param_{param_id:02X}"));
            let value = definition.and_then(|definition| {
                if definition.param_type == ParamType::String {
                    Some(ParamValue::String(
                        String::from_utf8_lossy(&frame.data[2..frame.can_dlc as usize]).to_string(),
                    ))
                } else {
                    let mut bytes = [0_u8; 4];
                    let payload = &frame.data[2..frame.can_dlc as usize];
                    bytes[..payload.len()].copy_from_slice(payload);
                    deserialize_param(definition, bytes).ok()
                }
            });
            return Some(DecodedFrame::ParamRequest {
                node: self.node(),
                kind: FrameKind::SetParamReq,
                name,
                value,
            });
        }

        if frame.can_id == self.motor_id as u32
            && frame.can_dlc == 6
            && (frame.data[0] & 0xE0) == 0xA0
        {
            let param_id = frame.data[1] + 0xE0;
            let mut bytes = [0_u8; 4];
            bytes.copy_from_slice(&frame.data[2..6]);
            let name = param_by_id(PARAMS, param_id)
                .map(|definition| definition.name.to_owned())
                .unwrap_or_else(|| format!("param_{param_id:02X}"));
            let value = param_by_id(PARAMS, param_id)
                .and_then(|definition| deserialize_param(definition, bytes).ok())
                .unwrap_or(ParamValue::Raw4(bytes));
            let mut values = BTreeMap::new();
            values.insert(name, value);
            return Some(DecodedFrame::ParamResponse {
                node: self.node(),
                kind: FrameKind::GetParamResp,
                values,
            });
        }

        if frame.can_id == self.motor_id as u32 | PARAM_RET_ARB_ID_BIAS && frame.can_dlc >= 2 {
            let param_id = frame.data[0];
            let segment_index = frame.data[1];
            let mut bytes = [0_u8; 4];
            let payload = &frame.data[2..usize::min(frame.can_dlc as usize, 6)];
            bytes[..payload.len()].copy_from_slice(payload);
            let definition = param_by_id(PARAMS, param_id);
            let name = definition
                .map(|definition| definition.name.to_owned())
                .unwrap_or_else(|| format!("param_{param_id:02X}"));

            if let Some(definition) = definition {
                if definition.param_type == ParamType::String {
                    let assembler = self.string_assemblers.entry(param_id).or_default();
                    if let Some(string_value) =
                        assembler.push(segment_index, definition.length, bytes)
                    {
                        let mut values = BTreeMap::new();
                        values.insert(name, ParamValue::String(string_value));
                        return Some(DecodedFrame::ParamResponse {
                            node: self.node(),
                            kind: FrameKind::GetParamResp,
                            values,
                        });
                    }
                    return None;
                }

                let value = deserialize_param(definition, bytes).unwrap_or(ParamValue::Raw4(bytes));
                let mut values = BTreeMap::new();
                values.insert(name, value);
                return Some(DecodedFrame::ParamResponse {
                    node: self.node(),
                    kind: FrameKind::GetParamResp,
                    values,
                });
            }

            let mut values = BTreeMap::new();
            values.insert(name, ParamValue::Raw4(bytes));
            return Some(DecodedFrame::ParamResponse {
                node: self.node(),
                kind: FrameKind::GetParamResp,
                values,
            });
        }

        None
    }

    fn generate_param_get(&self, name: &str) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let definition = param_by_name(PARAMS, name)?;
        if definition.id <= 0x80 {
            Ok(vec![
                RawCanFrame::new(
                    self.motor_id as u32 | PARAM_GET_ARB_ID_BIAS,
                    &[definition.id],
                )
                .map_err(ProtocolError::InvalidFrame)?,
            ])
        } else {
            Ok(vec![
                RawCanFrame::new(self.motor_id as u32, &[0xE0, definition.id - 0xE0])
                    .map_err(ProtocolError::InvalidFrame)?,
            ])
        }
    }

    fn generate_param_set(
        &self,
        name: &str,
        value: &ParamValue,
    ) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let definition = param_by_name(PARAMS, name)?;
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
                    RawCanFrame::new(self.motor_id as u32 | PARAM_SET_ARB_ID_BIAS, &payload)
                        .map_err(ProtocolError::InvalidFrame)?,
                );
            }
            return Ok(frames);
        }

        if definition.id > 0x80 {
            return Err(ProtocolError::UnsupportedFrameKind);
        }

        let bytes = serialize_param(definition, value)?;
        let mut payload = vec![definition.id, 0x01];
        payload.extend_from_slice(&bytes);
        Ok(vec![
            RawCanFrame::new(self.motor_id as u32 | PARAM_SET_ARB_ID_BIAS, &payload)
                .map_err(ProtocolError::InvalidFrame)?,
        ])
    }
}
