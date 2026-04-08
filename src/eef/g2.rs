use super::{EefState, SingleEefCommand, SingleEefFeedback};
use crate::protocol::motor::MotorProtocol;
use crate::protocol::motor::dm::DmProtocol;
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

    pub fn set_state(&self, state: EefState) {
        *self.state.write().expect("G2 state lock poisoned") = state;
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
}

#[cfg(test)]
mod tests {
    use super::G2;

    #[test]
    fn g2_transform_roundtrip() {
        let motor = G2::eef_to_motor(0.04, 0.25, 3.0);
        let eef = G2::motor_to_eef(motor.0, motor.1, motor.2);

        assert!((eef.0 - 0.04).abs() < 1e-6);
        assert!((eef.1 - 0.25).abs() < 1e-6);
        assert!((eef.2 - 3.0).abs() < 1e-6);
    }
}
