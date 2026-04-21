use super::{EefState, SingleEefCommand, SingleEefFeedback};
use crate::protocol::motor::MotorProtocol;
use crate::protocol::motor::od::OdProtocol;
use crate::types::{DecodedFrame, MotorCommand, MotorState, ProtocolNodeKind, RawCanFrame};
use std::sync::{Mutex, RwLock};
use std::time::{SystemTime, UNIX_EPOCH};
use thiserror::Error;
use tokio::sync::broadcast;

const POSITION_SCALE: f64 = 0.018;

#[derive(Debug, Error)]
pub enum E2Error {
    #[error("invalid E2 state command while disabled")]
    Disabled,
    #[error("protocol error: {0}")]
    Protocol(String),
}

#[derive(Debug)]
pub struct E2 {
    motor_id: u16,
    protocol: Mutex<OdProtocol>,
    state: RwLock<EefState>,
    latest_feedback: RwLock<Option<SingleEefFeedback>>,
    feedback_tx: broadcast::Sender<SingleEefFeedback>,
}

impl E2 {
    pub fn new(motor_id: u16) -> Self {
        let (feedback_tx, _) = broadcast::channel(128);
        Self {
            motor_id,
            protocol: Mutex::new(OdProtocol::new(motor_id)),
            state: RwLock::new(EefState::Disabled),
            latest_feedback: RwLock::new(None),
            feedback_tx,
        }
    }

    pub fn motor_id(&self) -> u16 {
        self.motor_id
    }

    pub fn state(&self) -> EefState {
        *self.state.read().expect("E2 state lock poisoned")
    }

    pub fn set_state(&self, state: EefState) -> Result<Vec<RawCanFrame>, E2Error> {
        *self.state.write().expect("E2 state lock poisoned") = state;
        Ok(Vec::new())
    }

    pub fn latest_feedback(&self) -> Option<SingleEefFeedback> {
        self.latest_feedback
            .read()
            .expect("E2 feedback lock poisoned")
            .clone()
    }

    pub fn subscribe_feedback(&self) -> broadcast::Receiver<SingleEefFeedback> {
        self.feedback_tx.subscribe()
    }

    pub fn handle_raw_frame(&self, frame: &RawCanFrame) {
        let decoded = self
            .protocol
            .lock()
            .expect("E2 protocol lock poisoned")
            .inspect(frame);
        if let Some(DecodedFrame::MotionFeedback { node, state }) = decoded
            && node.kind == ProtocolNodeKind::OdMotor
            && node.id == self.motor_id
        {
            let feedback = self.feedback_from_motor_state(&state);
            *self
                .latest_feedback
                .write()
                .expect("E2 feedback lock poisoned") = Some(feedback.clone());
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
            timestamp_micros: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros(),
        }
    }

    pub fn build_mit_command(
        &self,
        command: &SingleEefCommand,
    ) -> Result<Vec<RawCanFrame>, E2Error> {
        if self.state() != EefState::Enabled {
            return Err(E2Error::Disabled);
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
            .expect("E2 protocol lock poisoned")
            .generate_mit(&motor_command)
            .map_err(|err| E2Error::Protocol(err.to_string()))
    }

    pub fn build_feedback_poll_command(&self) -> Result<Vec<RawCanFrame>, E2Error> {
        self.build_mit_command(&SingleEefCommand::default())
    }

    pub fn shutdown_frames(&self) -> Result<Vec<RawCanFrame>, E2Error> {
        Ok(Vec::new())
    }

    pub fn motor_to_eef(motor_pos: f64, motor_vel: f64, _motor_eff: f64) -> (f64, f64, f64) {
        (-motor_pos * POSITION_SCALE, motor_vel, 0.0)
    }

    pub fn eef_to_motor(eef_pos: f64, eef_vel: f64, _eef_eff: f64) -> (f64, f64, f64) {
        (-eef_pos / POSITION_SCALE, eef_vel, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::E2;
    use crate::eef::EefState;

    #[test]
    fn e2_transform_roundtrip() {
        let motor = E2::eef_to_motor(0.012, 0.4, 0.0);
        let eef = E2::motor_to_eef(motor.0, motor.1, motor.2);

        assert!((eef.0 - 0.012).abs() < 1e-9);
        assert!((eef.1 - 0.4).abs() < 1e-9);
        assert_eq!(eef.2, 0.0);
    }

    #[test]
    fn e2_state_changes_do_not_emit_frames() {
        let e2 = E2::new(7);
        let frames = e2
            .set_state(EefState::Enabled)
            .expect("state transition should succeed");

        assert!(frames.is_empty());
        assert_eq!(e2.state(), EefState::Enabled);
    }

    #[test]
    fn e2_feedback_poll_requires_enabled_state() {
        let e2 = E2::new(7);
        let error = e2
            .build_feedback_poll_command()
            .expect_err("feedback polling should require enabled state");

        assert!(matches!(error, super::E2Error::Disabled));
    }

    #[test]
    fn e2_feedback_poll_builds_zero_mit_frames_when_enabled() {
        let e2 = E2::new(7);
        e2.set_state(EefState::Enabled)
            .expect("state transition should succeed");

        let frames = e2
            .build_feedback_poll_command()
            .expect("feedback poll frames should build");

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].can_id, 7);
    }
}
