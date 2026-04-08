use super::MotorProtocol;
use crate::protocol::{
    ProtocolError, deserialize_param, inverse_range_map_f32, param_by_id, param_by_name,
    range_map_f32, serialize_param,
};
use crate::types::{
    DecodedFrame, FrameKind, MotorCommand, MotorState, ParamDefinition, ParamType, ParamValue,
    ProtocolNode, ProtocolNodeKind, RawCanFrame,
};
use std::collections::BTreeMap;

const CONFIG_ARB_ID: u32 = 0x7FF;
const PV_ARB_ID_BIAS: u32 = 0x100;
const V_ARB_ID_BIAS: u32 = 0x200;
const PVT_ARB_ID_BIAS: u32 = 0x300;
const RET_ARB_ID_BIAS: u32 = 0x700;

const KP_MIN: f32 = 0.0;
const KP_MAX: f32 = 500.0;
const KD_MIN: f32 = 0.0;
const KD_MAX: f32 = 5.0;

const PARAMS: &[ParamDefinition] = &[
    ParamDefinition::new(
        0x00,
        "under_voltage_thres",
        ParamType::Float32Le,
        1,
        true,
        true,
    ),
    ParamDefinition::new(0x01, "k_torque_out", ParamType::Float32Le, 1, true, true),
    ParamDefinition::new(0x04, "acceleration", ParamType::Float32Le, 1, true, true),
    ParamDefinition::new(0x05, "deceleration", ParamType::Float32Le, 1, true, true),
    ParamDefinition::new(0x06, "max_speed", ParamType::Float32Le, 1, true, true),
    ParamDefinition::new(0x07, "feedback_can_id", ParamType::UInt32Le, 1, true, true),
    ParamDefinition::new(0x08, "listen_can_id", ParamType::UInt32Le, 1, true, true),
    ParamDefinition::new(0x09, "timeout", ParamType::UInt32Le, 1, true, true),
    ParamDefinition::new(0x0A, "control_mode", ParamType::UInt32Le, 1, true, true),
    ParamDefinition::new(0x0B, "damp_eff", ParamType::Float32Le, 1, true, false),
    ParamDefinition::new(0x0C, "inertia", ParamType::Float32Le, 1, true, false),
    ParamDefinition::new(0x0D, "hardware_version", ParamType::String, 1, true, false),
    ParamDefinition::new(0x0E, "fw_version", ParamType::String, 1, true, false),
    ParamDefinition::new(
        0x10,
        "number_of_pole_pairs",
        ParamType::UInt32Le,
        1,
        true,
        false,
    ),
    ParamDefinition::new(
        0x11,
        "phase_resistance",
        ParamType::Float32Le,
        1,
        true,
        false,
    ),
    ParamDefinition::new(
        0x12,
        "phase_inductance",
        ParamType::Float32Le,
        1,
        true,
        false,
    ),
    ParamDefinition::new(0x13, "flux_linkage", ParamType::Float32Le, 1, true, false),
    ParamDefinition::new(0x14, "gear_ratio", ParamType::Float32Le, 1, true, false),
    ParamDefinition::new(0x15, "pos_max", ParamType::Float32Le, 1, true, true),
    ParamDefinition::new(0x16, "vel_max", ParamType::Float32Le, 1, true, true),
    ParamDefinition::new(0x17, "torque_max", ParamType::Float32Le, 1, true, true),
];

#[derive(Debug)]
pub struct DmProtocol {
    motor_id: u16,
    pos_max: Option<f32>,
    vel_max: Option<f32>,
    tor_max: Option<f32>,
}

impl Default for DmProtocol {
    fn default() -> Self {
        Self {
            motor_id: 0,
            pos_max: Some(12.5),
            vel_max: Some(30.0),
            tor_max: Some(10.0),
        }
    }
}

impl DmProtocol {
    pub fn new(motor_id: u16) -> Self {
        Self {
            motor_id,
            ..Self::default()
        }
    }

    pub fn set_pos_max(&mut self, pos_max: f32) {
        self.pos_max = Some(pos_max);
    }

    pub fn set_vel_max(&mut self, vel_max: f32) {
        self.vel_max = Some(vel_max);
    }

    pub fn set_tor_max(&mut self, tor_max: f32) {
        self.tor_max = Some(tor_max);
    }

    pub fn generate_pv(&self, command: &MotorCommand) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let mut payload = [0_u8; 8];
        payload[..4].copy_from_slice(&(command.pos as f32).to_le_bytes());
        payload[4..8].copy_from_slice(&(command.vel as f32).to_le_bytes());
        Ok(vec![
            RawCanFrame::new(self.motor_id as u32 | PV_ARB_ID_BIAS, &payload)
                .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    pub fn generate_csv(&self, command: &MotorCommand) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let payload = (command.vel as f32).to_le_bytes();
        Ok(vec![
            RawCanFrame::new(self.motor_id as u32 | V_ARB_ID_BIAS, &payload)
                .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    pub fn generate_pvt(&self, command: &MotorCommand) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let mut payload = [0_u8; 8];
        payload[..4].copy_from_slice(&(command.pos as f32).to_le_bytes());
        let vel = ((command.vel as f32) * 100.0).clamp(0.0, 10_000.0) as u16;
        let current =
            ((command.current_threshold as f32 / 10.261194) * 10_000.0).clamp(0.0, 10_000.0) as u16;
        payload[4..6].copy_from_slice(&vel.to_le_bytes());
        payload[6..8].copy_from_slice(&current.to_le_bytes());
        Ok(vec![
            RawCanFrame::new(self.motor_id as u32 | PVT_ARB_ID_BIAS, &payload)
                .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    pub fn generate_mit(&self, command: &MotorCommand) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let pos_max = self.pos_max.unwrap_or(12.5);
        let vel_max = self.vel_max.unwrap_or(30.0);
        let tor_max = self.tor_max.unwrap_or(10.0);

        let p_int = range_map_f32(command.pos as f32, -pos_max, pos_max, 0, 0xFFFF) as u16;
        let v_int = range_map_f32(command.vel as f32, -vel_max, vel_max, 0, 0x0FFF) as u16;
        let kp_int = range_map_f32(command.mit_kp as f32, KP_MIN, KP_MAX, 0, 0x0FFF) as u16;
        let kd_int = range_map_f32(command.mit_kd as f32, KD_MIN, KD_MAX, 0, 0x0FFF) as u16;
        let t_int = range_map_f32(command.eff as f32, -tor_max, tor_max, 0, 0x0FFF) as u16;

        let payload = [
            (p_int >> 8) as u8,
            (p_int & 0xFF) as u8,
            (v_int >> 4) as u8,
            (((v_int & 0x0F) << 4) as u8) | ((kp_int >> 8) as u8),
            (kp_int & 0xFF) as u8,
            (kd_int >> 4) as u8,
            (((kd_int & 0x0F) << 4) as u8) | ((t_int >> 8) as u8),
            (t_int & 0xFF) as u8,
        ];

        Ok(vec![
            RawCanFrame::new(self.motor_id as u32, &payload)
                .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    pub fn generate_ping(&self) -> Result<Vec<RawCanFrame>, ProtocolError> {
        Ok(vec![
            RawCanFrame::new(
                self.motor_id as u32,
                &[self.motor_id as u8, (self.motor_id >> 8) as u8, 0xCC, 0x00],
            )
            .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    pub fn generate_enable(&self) -> Result<Vec<RawCanFrame>, ProtocolError> {
        Ok(vec![
            RawCanFrame::new(
                self.motor_id as u32,
                &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
            )
            .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    pub fn generate_disable(&self) -> Result<Vec<RawCanFrame>, ProtocolError> {
        Ok(vec![
            RawCanFrame::new(
                self.motor_id as u32,
                &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
            )
            .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    pub fn generate_set_zero(&self) -> Result<Vec<RawCanFrame>, ProtocolError> {
        Ok(vec![
            RawCanFrame::new(
                self.motor_id as u32,
                &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE],
            )
            .map_err(ProtocolError::InvalidFrame)?,
        ])
    }

    pub fn generate_reset_err(&self) -> Result<Vec<RawCanFrame>, ProtocolError> {
        Ok(vec![
            RawCanFrame::new(
                self.motor_id as u32,
                &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB],
            )
            .map_err(ProtocolError::InvalidFrame)?,
        ])
    }
}

impl MotorProtocol for DmProtocol {
    fn node(&self) -> ProtocolNode {
        ProtocolNode {
            kind: ProtocolNodeKind::DmMotor,
            id: self.motor_id,
        }
    }

    fn definitions(&self) -> &[ParamDefinition] {
        PARAMS
    }

    fn inspect(&mut self, frame: &RawCanFrame) -> Option<DecodedFrame> {
        if frame.can_id == self.motor_id as u32 | PV_ARB_ID_BIAS && frame.can_dlc == 8 {
            return Some(DecodedFrame::MotionCommand {
                node: self.node(),
                kind: FrameKind::PvReq,
                command: MotorCommand {
                    pos: f32::from_le_bytes(frame.data[..4].try_into().ok()?) as f64,
                    vel: f32::from_le_bytes(frame.data[4..8].try_into().ok()?) as f64,
                    ..MotorCommand::default()
                },
            });
        }

        if frame.can_id == self.motor_id as u32 | V_ARB_ID_BIAS && frame.can_dlc == 4 {
            return Some(DecodedFrame::MotionCommand {
                node: self.node(),
                kind: FrameKind::CsvReq,
                command: MotorCommand {
                    vel: f32::from_le_bytes(frame.data[..4].try_into().ok()?) as f64,
                    ..MotorCommand::default()
                },
            });
        }

        if frame.can_id == self.motor_id as u32 | PVT_ARB_ID_BIAS && frame.can_dlc == 8 {
            let vel = u16::from_le_bytes([frame.data[4], frame.data[5]]) as f64 / 100.0;
            let current = u16::from_le_bytes([frame.data[6], frame.data[7]]) as f64 / 10_000.0
                * 10.261194_f64;
            return Some(DecodedFrame::MotionCommand {
                node: self.node(),
                kind: FrameKind::PvtReq,
                command: MotorCommand {
                    pos: f32::from_le_bytes(frame.data[..4].try_into().ok()?) as f64,
                    vel,
                    current_threshold: current,
                    ..MotorCommand::default()
                },
            });
        }

        if frame.can_id == self.motor_id as u32 && frame.can_dlc == 4 && frame.data[2] == 0xCC {
            return Some(DecodedFrame::LifecycleCommand {
                node: self.node(),
                kind: FrameKind::PingReq,
            });
        }

        if frame.can_id == self.motor_id as u32 && frame.can_dlc == 8 {
            let data = frame.payload();
            if data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC] {
                return Some(DecodedFrame::LifecycleCommand {
                    node: self.node(),
                    kind: FrameKind::EnableReq,
                });
            }
            if data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD] {
                return Some(DecodedFrame::LifecycleCommand {
                    node: self.node(),
                    kind: FrameKind::DisableReq,
                });
            }
            if data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE] {
                return Some(DecodedFrame::LifecycleCommand {
                    node: self.node(),
                    kind: FrameKind::SetZeroReq,
                });
            }
            if data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB] {
                return Some(DecodedFrame::LifecycleCommand {
                    node: self.node(),
                    kind: FrameKind::ResetErrReq,
                });
            }

            let pos_max = self.pos_max.unwrap_or(12.5);
            let vel_max = self.vel_max.unwrap_or(30.0);
            let tor_max = self.tor_max.unwrap_or(10.0);
            let p_int = ((frame.data[0] as u16) << 8) | frame.data[1] as u16;
            let v_int = ((frame.data[2] as u16) << 4) | ((frame.data[3] as u16) >> 4);
            let kp_int = (((frame.data[3] & 0x0F) as u16) << 8) | frame.data[4] as u16;
            let kd_int = ((frame.data[5] as u16) << 4) | ((frame.data[6] as u16) >> 4);
            let t_int = (((frame.data[6] & 0x0F) as u16) << 8) | frame.data[7] as u16;
            return Some(DecodedFrame::MotionCommand {
                node: self.node(),
                kind: FrameKind::MitReq,
                command: MotorCommand {
                    pos: inverse_range_map_f32(p_int as u32, 0, 0xFFFF, -pos_max, pos_max) as f64,
                    vel: inverse_range_map_f32(v_int as u32, 0, 0x0FFF, -vel_max, vel_max) as f64,
                    eff: inverse_range_map_f32(t_int as u32, 0, 0x0FFF, -tor_max, tor_max) as f64,
                    mit_kp: inverse_range_map_f32(kp_int as u32, 0, 0x0FFF, KP_MIN, KP_MAX) as f64,
                    mit_kd: inverse_range_map_f32(kd_int as u32, 0, 0x0FFF, KD_MIN, KD_MAX) as f64,
                    current_threshold: 0.0,
                },
            });
        }

        if frame.can_id == CONFIG_ARB_ID && frame.can_dlc >= 4 {
            let target_id = (frame.data[1] as u16) << 8 | frame.data[0] as u16;
            if target_id != self.motor_id {
                return None;
            }
            if frame.data[2] == 0x33 {
                let param_id = frame.data[3];
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
            if frame.data[2] == 0x55 && frame.can_dlc == 8 {
                let param_id = frame.data[3];
                let definition = param_by_id(PARAMS, param_id);
                let name = definition
                    .map(|definition| definition.name.to_owned())
                    .unwrap_or_else(|| format!("param_{param_id:02X}"));
                let mut bytes = [0_u8; 4];
                bytes.copy_from_slice(&frame.data[4..8]);
                let value = definition
                    .and_then(|definition| deserialize_param(definition, bytes).ok())
                    .unwrap_or(ParamValue::Raw4(bytes));
                return Some(DecodedFrame::ParamRequest {
                    node: self.node(),
                    kind: FrameKind::SetParamReq,
                    name,
                    value: Some(value),
                });
            }
        }

        if frame.can_id == self.motor_id as u32 | RET_ARB_ID_BIAS {
            let try_motor_id = ((frame.data[1] as u16) << 8) | frame.data[0] as u16;
            if try_motor_id == self.motor_id && frame.can_dlc == 8 && frame.data[2] == 0x33 {
                let param_id = frame.data[3];
                let definition = param_by_id(PARAMS, param_id);
                let name = definition
                    .map(|definition| definition.name.to_owned())
                    .unwrap_or_else(|| format!("param_{param_id:02X}"));
                let mut values = BTreeMap::new();
                if let Some(definition) = definition {
                    if definition.param_type == ParamType::String {
                        values.insert(
                            name,
                            ParamValue::String(
                                String::from_utf8_lossy(&frame.data[4..8])
                                    .trim_end_matches('\0')
                                    .to_owned(),
                            ),
                        );
                    } else {
                        let mut bytes = [0_u8; 4];
                        bytes.copy_from_slice(&frame.data[4..8]);
                        values.insert(
                            name,
                            deserialize_param(definition, bytes).unwrap_or(ParamValue::Raw4(bytes)),
                        );
                    }
                }
                return Some(DecodedFrame::ParamResponse {
                    node: self.node(),
                    kind: FrameKind::GetParamResp,
                    values,
                });
            }

            if try_motor_id == self.motor_id && frame.can_dlc == 8 && frame.data[2] == 0x55 {
                let param_id = frame.data[3];
                let definition = param_by_id(PARAMS, param_id);
                let name = definition
                    .map(|definition| definition.name.to_owned())
                    .unwrap_or_else(|| format!("param_{param_id:02X}"));
                let mut values = BTreeMap::new();
                if let Some(definition) = definition {
                    if definition.param_type == ParamType::String {
                        values.insert(
                            name,
                            ParamValue::String(
                                String::from_utf8_lossy(&frame.data[4..8])
                                    .trim_end_matches('\0')
                                    .to_owned(),
                            ),
                        );
                    } else {
                        let mut bytes = [0_u8; 4];
                        bytes.copy_from_slice(&frame.data[4..8]);
                        values.insert(
                            name,
                            deserialize_param(definition, bytes).unwrap_or(ParamValue::Raw4(bytes)),
                        );
                    }
                }
                return Some(DecodedFrame::ParamResponse {
                    node: self.node(),
                    kind: FrameKind::SetParamResp,
                    values,
                });
            }

            if try_motor_id == self.motor_id
                && frame.can_dlc == 4
                && frame.data[2] == 0xAA
                && frame.data[3] == 0x01
            {
                return Some(DecodedFrame::ParamResponse {
                    node: self.node(),
                    kind: FrameKind::PersistParamResp,
                    values: BTreeMap::new(),
                });
            }

            if frame.can_dlc == 8 {
                let pos_max = self.pos_max.unwrap_or(12.5);
                let vel_max = self.vel_max.unwrap_or(30.0);
                let tor_max = self.tor_max.unwrap_or(10.0);
                let error_code = (frame.data[0] & 0xF0) >> 4;
                let pos_int = ((frame.data[1] as u16) << 8) | frame.data[2] as u16;
                let spd_int =
                    ((frame.data[3] as u16) << 4) | (((frame.data[4] & 0xF0) as u16) >> 4);
                let t_int = (((frame.data[4] & 0x0F) as u16) << 8) | frame.data[5] as u16;
                return Some(DecodedFrame::MotionFeedback {
                    node: self.node(),
                    state: MotorState {
                        is_valid: true,
                        joint_id: self.motor_id,
                        pos: inverse_range_map_f32(pos_int as u32, 0, 0xFFFF, -pos_max, pos_max)
                            as f64,
                        vel: inverse_range_map_f32(spd_int as u32, 0, 0x0FFF, -vel_max, vel_max)
                            as f64,
                        eff: inverse_range_map_f32(t_int as u32, 0, 0x0FFF, -tor_max, tor_max)
                            as f64,
                        motor_temp: frame.data[6],
                        mos_temp: frame.data[7],
                        error_id: error_code,
                    },
                });
            }
        }

        None
    }

    fn generate_param_get(&self, name: &str) -> Result<Vec<RawCanFrame>, ProtocolError> {
        let definition = param_by_name(PARAMS, name)?;
        let payload = [
            (self.motor_id & 0xFF) as u8,
            (self.motor_id >> 8) as u8,
            0x33,
            definition.id,
        ];
        Ok(vec![
            RawCanFrame::new(CONFIG_ARB_ID, &payload).map_err(ProtocolError::InvalidFrame)?,
        ])
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

        let bytes = if definition.param_type == ParamType::String {
            let ParamValue::String(text) = value else {
                return Err(ProtocolError::ParameterTypeMismatch(name.to_owned()));
            };
            let mut padded = [0_u8; 4];
            for (dst, src) in padded.iter_mut().zip(text.as_bytes().iter().copied()) {
                *dst = src;
            }
            padded
        } else {
            serialize_param(definition, value)?
        };

        let payload = [
            (self.motor_id & 0xFF) as u8,
            (self.motor_id >> 8) as u8,
            0x55,
            definition.id,
            bytes[0],
            bytes[1],
            bytes[2],
            bytes[3],
        ];
        Ok(vec![
            RawCanFrame::new(CONFIG_ARB_ID, &payload).map_err(ProtocolError::InvalidFrame)?,
        ])
    }
}
