use super::{EefState, SingleEefCommand, SingleEefFeedback};
use crate::protocol::motor::MotorProtocol;
use crate::protocol::motor::dm::DmProtocol;
use crate::types::ParamValue;
use crate::types::{DecodedFrame, MotorCommand, MotorState, ProtocolNodeKind, RawCanFrame};
use std::sync::{Mutex, RwLock};
use std::time::{SystemTime, UNIX_EPOCH};
use thiserror::Error;
use tokio::sync::broadcast;

const MAPPING_L1: f64 = 0.022;
const MAPPING_L2: f64 = 0.036;
const MAPPING_L3: f64 = 0.01722;
const MAPPING_THETA0: f64 = 2.3190538837099055;
const MAX_MOTOR_VEL: f64 = 40.0;
const DEFAULT_MIT_KP: f64 = 30.0;
const DEFAULT_MIT_KD: f64 = 1.5;
const DEFAULT_MIT_EFFORT: f64 = 0.0;

#[derive(Debug, Error)]
pub enum G2Error {
    #[error("invalid G2 state command while disabled")]
    Disabled,
    #[error("protocol error: {0}")]
    Protocol(String),
}

#[derive(Debug)]
pub struct G2 {
    motor_id: u16,
    protocol: Mutex<DmProtocol>,
    state: RwLock<EefState>,
    target_command: Mutex<Option<SingleEefCommand>>,
    latest_feedback: RwLock<Option<SingleEefFeedback>>,
    feedback_tx: broadcast::Sender<SingleEefFeedback>,
}

impl G2 {
    pub fn new(motor_id: u16) -> Self {
        let (feedback_tx, _) = broadcast::channel(128);
        Self {
            motor_id,
            protocol: Mutex::new(DmProtocol::new(motor_id)),
            state: RwLock::new(EefState::Disabled),
            target_command: Mutex::new(None),
            latest_feedback: RwLock::new(None),
            feedback_tx,
        }
    }

    pub fn motor_id(&self) -> u16 {
        self.motor_id
    }

    pub fn state(&self) -> EefState {
        *self.state.read().expect("G2 state lock poisoned")
    }

    pub fn set_state(&self, state: EefState) -> Result<Vec<RawCanFrame>, G2Error> {
        let previous = self.state();
        if previous == state {
            return Ok(Vec::new());
        }
        let frames = {
            let protocol = self.protocol.lock().expect("G2 protocol lock poisoned");
            match state {
                EefState::Disabled => protocol
                    .generate_disable()
                    .map_err(|err| G2Error::Protocol(err.to_string()))?,
                EefState::Enabled => {
                    let mut frames = protocol
                        .generate_reset_err()
                        .map_err(|err| G2Error::Protocol(err.to_string()))?;
                    frames.extend(
                        protocol
                            .generate_enable()
                            .map_err(|err| G2Error::Protocol(err.to_string()))?,
                    );
                    frames.extend(
                        protocol
                            .generate_param_set("control_mode", &ParamValue::U32(0x01))
                            .map_err(|err| G2Error::Protocol(err.to_string()))?,
                    );
                    frames
                }
            }
        };
        *self.state.write().expect("G2 state lock poisoned") = state;
        match state {
            EefState::Disabled => self.clear_target(),
            EefState::Enabled => self.seed_target_from_latest_feedback_if_missing(),
        }
        Ok(frames)
    }

    pub fn latest_feedback(&self) -> Option<SingleEefFeedback> {
        self.latest_feedback
            .read()
            .expect("G2 feedback lock poisoned")
            .clone()
    }

    pub fn subscribe_feedback(&self) -> broadcast::Receiver<SingleEefFeedback> {
        self.feedback_tx.subscribe()
    }

    pub fn handle_raw_frame(&self, frame: &RawCanFrame) {
        let decoded = self
            .protocol
            .lock()
            .expect("G2 protocol lock poisoned")
            .inspect(frame);
        if let Some(DecodedFrame::MotionFeedback { node, state }) = decoded
            && node.kind == ProtocolNodeKind::DmMotor
            && node.id == self.motor_id
        {
            let feedback = self.feedback_from_motor_state(&state);
            *self
                .latest_feedback
                .write()
                .expect("G2 feedback lock poisoned") = Some(feedback.clone());
            if self.state() == EefState::Enabled {
                self.seed_target_from_feedback_if_missing(&feedback);
            }
            let _ = self.feedback_tx.send(feedback);
        }
    }

    pub fn feedback_from_motor_state(&self, state: &MotorState) -> SingleEefFeedback {
        let (position, velocity, effort) = Self::motor_to_eef(state.pos, state.vel, state.eff);
        SingleEefFeedback {
            position,
            velocity,
            effort,
            valid: state.is_valid,
            timestamp_millis: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis(),
        }
    }

    pub fn build_mit_command(
        &self,
        command: &SingleEefCommand,
    ) -> Result<Vec<RawCanFrame>, G2Error> {
        if self.state() != EefState::Enabled {
            return Err(G2Error::Disabled);
        }

        let (motor_pos, motor_vel, motor_eff) =
            Self::eef_to_motor(command.position, command.velocity, command.effort);
        let motor_command = MotorCommand {
            pos: motor_pos,
            vel: motor_vel,
            eff: motor_eff,
            mit_kp: command.mit_kp,
            mit_kd: command.mit_kd,
            current_threshold: command.current_threshold,
        };

        self.protocol
            .lock()
            .expect("G2 protocol lock poisoned")
            .generate_mit(&motor_command)
            .map_err(|err| G2Error::Protocol(err.to_string()))
    }

    pub fn submit_target(&self, command: &SingleEefCommand) -> Result<(), G2Error> {
        if self.state() != EefState::Enabled {
            return Err(G2Error::Disabled);
        }
        *self
            .target_command
            .lock()
            .expect("G2 target command lock poisoned") = Some(command.clone());
        Ok(())
    }

    pub fn latest_target(&self) -> Option<SingleEefCommand> {
        self.target_command
            .lock()
            .expect("G2 target command lock poisoned")
            .clone()
    }

    pub fn control_frames(&self) -> Result<Option<Vec<RawCanFrame>>, G2Error> {
        if self.state() != EefState::Enabled {
            return Ok(None);
        }
        self.seed_target_from_latest_feedback_if_missing();
        let Some(target) = self.latest_target() else {
            return Ok(None);
        };
        Ok(Some(self.build_mit_command(&target)?))
    }

    pub fn build_pvt_command(
        &self,
        command: &SingleEefCommand,
    ) -> Result<Vec<RawCanFrame>, G2Error> {
        if self.state() != EefState::Enabled {
            return Err(G2Error::Disabled);
        }

        let (motor_pos, _motor_vel, _motor_eff) =
            Self::eef_to_motor(command.position, command.velocity, command.effort);
        let motor_command = MotorCommand {
            pos: motor_pos,
            vel: MAX_MOTOR_VEL,
            eff: 0.0,
            mit_kp: 0.0,
            mit_kd: 0.0,
            current_threshold: command.current_threshold,
        };

        self.protocol
            .lock()
            .expect("G2 protocol lock poisoned")
            .generate_pvt(&motor_command)
            .map_err(|err| G2Error::Protocol(err.to_string()))
    }

    pub fn shutdown_frames(&self) -> Result<Vec<RawCanFrame>, G2Error> {
        self.clear_target();
        self.protocol
            .lock()
            .expect("G2 protocol lock poisoned")
            .generate_disable()
            .map_err(|err| G2Error::Protocol(err.to_string()))
    }

    pub fn motor_to_eef(motor_pos: f64, motor_vel: f64, motor_eff: f64) -> (f64, f64, f64) {
        let theta = MAPPING_THETA0 + motor_pos;
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        let root =
            (MAPPING_L2 * MAPPING_L2 - MAPPING_L1 * MAPPING_L1 * sin_theta * sin_theta).sqrt();

        let eef_pos = 2.0 * (MAPPING_L1 * cos_theta + root - MAPPING_L3);
        let eef_vel = -motor_vel * MAPPING_L1 * sin_theta * (1.0 + MAPPING_L1 * cos_theta / root);
        let eef_eff = motor_eff * root / (2.0 * MAPPING_L1 * MAPPING_L2);
        (eef_pos, eef_vel, eef_eff)
    }

    pub fn eef_to_motor(eef_pos: f64, eef_vel: f64, eef_eff: f64) -> (f64, f64, f64) {
        let slider = eef_pos / 2.0 + MAPPING_L3;
        let cos_term = (slider * slider + MAPPING_L1 * MAPPING_L1 - MAPPING_L2 * MAPPING_L2)
            / (2.0 * slider * MAPPING_L1);
        let motor_pos = cos_term.acos() - MAPPING_THETA0;

        let theta = MAPPING_THETA0 + motor_pos;
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        let root =
            (MAPPING_L2 * MAPPING_L2 - MAPPING_L1 * MAPPING_L1 * sin_theta * sin_theta).sqrt();

        let motor_vel =
            -(eef_vel * root) / (MAPPING_L1 * sin_theta * (root + MAPPING_L1 * cos_theta));
        let motor_eff = 2.0 * MAPPING_L1 * MAPPING_L2 * eef_eff / root;
        (motor_pos, motor_vel, motor_eff)
    }

    fn clear_target(&self) {
        self.target_command
            .lock()
            .expect("G2 target command lock poisoned")
            .take();
    }

    fn seed_target_from_latest_feedback_if_missing(&self) {
        let Some(feedback) = self.latest_feedback() else {
            return;
        };
        self.seed_target_from_feedback_if_missing(&feedback);
    }

    fn seed_target_from_feedback_if_missing(&self, feedback: &SingleEefFeedback) {
        let mut target = self
            .target_command
            .lock()
            .expect("G2 target command lock poisoned");
        if target.is_some() {
            return;
        }
        *target = Some(Self::hold_command_for_position(feedback.position));
    }

    fn hold_command_for_position(position: f64) -> SingleEefCommand {
        SingleEefCommand {
            position,
            velocity: 0.0,
            effort: DEFAULT_MIT_EFFORT,
            mit_kp: DEFAULT_MIT_KP,
            mit_kd: DEFAULT_MIT_KD,
            current_threshold: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::G2;
    use crate::eef::EefState;
    use crate::protocol::motor::MotorProtocol;
    use crate::protocol::motor::dm::DmProtocol;
    use crate::types::{MotorState, ParamValue};

    #[test]
    fn g2_transform_roundtrip() {
        let motor = G2::eef_to_motor(0.04, 0.25, 3.0);
        let eef = G2::motor_to_eef(motor.0, motor.1, motor.2);

        assert!((eef.0 - 0.04).abs() < 1e-6);
        assert!((eef.1 - 0.25).abs() < 1e-6);
        assert!((eef.2 - 3.0).abs() < 1e-6);
    }

    #[test]
    fn g2_enable_emits_dm_enable_frame() {
        let g2 = G2::new(7);
        let frames = g2
            .set_state(EefState::Enabled)
            .expect("state transition should succeed");

        let protocol = DmProtocol::new(7);
        let mut expected = protocol.generate_reset_err().expect("reset frames");
        expected.extend(protocol.generate_enable().expect("enable frames"));
        expected.extend(
            protocol
                .generate_param_set("control_mode", &ParamValue::U32(0x01))
                .expect("control mode frames"),
        );
        assert_eq!(frames, expected);
        assert_eq!(g2.state(), EefState::Enabled);
    }

    #[test]
    fn g2_shutdown_emits_dm_disable_frame() {
        let g2 = G2::new(7);
        let frames = g2.shutdown_frames().expect("shutdown frames should build");

        assert_eq!(frames.len(), 1);
        assert_eq!(
            frames[0].payload(),
            &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
        );
    }

    #[test]
    fn g2_control_frames_hold_latest_target() {
        let g2 = G2::new(7);
        g2.set_state(EefState::Enabled)
            .expect("state transition should succeed");
        g2.submit_target(&crate::eef::SingleEefCommand {
            position: 0.04,
            velocity: 0.0,
            effort: 0.0,
            mit_kp: 30.0,
            mit_kd: 1.5,
            current_threshold: 0.0,
        })
        .expect("target should be accepted");

        let frames = g2
            .control_frames()
            .expect("control frames should build")
            .expect("enabled G2 should emit control frames");
        assert_eq!(frames.len(), 1);
    }

    #[test]
    fn g2_seeds_hold_target_from_feedback_when_enabled() {
        let g2 = G2::new(7);
        g2.set_state(EefState::Enabled)
            .expect("state transition should succeed");

        let feedback = g2.feedback_from_motor_state(&MotorState {
            is_valid: true,
            joint_id: 7,
            pos: G2::eef_to_motor(0.02, 0.0, 0.0).0,
            vel: 0.0,
            eff: 0.0,
            motor_temp: 0,
            mos_temp: 0,
            error_id: 0,
        });
        g2.seed_target_from_feedback_if_missing(&feedback);

        let target = g2.latest_target().expect("hold target should be seeded");
        assert!((target.position - 0.02).abs() < 1e-6);
        assert_eq!(target.mit_kp, 30.0);
        assert_eq!(target.mit_kd, 1.5);
    }
}
