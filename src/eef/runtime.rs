use super::{E2, E2Error, EefState, G2, G2Error, SingleEefCommand, SingleEefFeedback};
use crate::can::worker::{CanTxPriority, CanWorker};
use crate::model::MountedEefType;
use crate::types::RawCanFrame;
use std::future::pending;
use std::sync::Arc;
use std::time::Duration;
use thiserror::Error;
use tokio::sync::{broadcast, mpsc};
use tokio::task::JoinHandle;
use tokio::time::{MissedTickBehavior, interval};
use tracing::warn;

const DEFAULT_EEF_MOTOR_ID: u16 = 7;
const E2_FEEDBACK_PUMP_INTERVAL: Duration = Duration::from_millis(20);
const G2_CONTROL_INTERVAL: Duration = Duration::from_millis(4);

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum EefRuntimeProfile {
    #[default]
    Generic,
    E2,
    G2,
}

impl EefRuntimeProfile {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Generic => "generic",
            Self::E2 => "e2",
            Self::G2 => "g2",
        }
    }

    pub fn matches_mounted_eef(&self, mounted_eef: &MountedEefType) -> bool {
        match self {
            Self::Generic => true,
            Self::E2 => matches!(mounted_eef, MountedEefType::E2B),
            Self::G2 => matches!(mounted_eef, MountedEefType::G2),
        }
    }
}

#[derive(Debug, Error)]
pub enum EefRuntimeError {
    #[error("mounted end-effector `{mounted}` is not compatible with runtime profile `{profile}`")]
    IncompatibleProfile {
        profile: &'static str,
        mounted: String,
    },
    #[error("mounted end-effector `{mounted}` is not managed by the runtime")]
    UnsupportedMountedEef { mounted: String },
    #[error("mounted end-effector `{mounted}` does not accept `{command}` commands")]
    UnsupportedCommand {
        command: &'static str,
        mounted: String,
    },
    #[error(transparent)]
    E2(#[from] E2Error),
    #[error(transparent)]
    G2(#[from] G2Error),
}

#[derive(Debug)]
enum EefRuntimeInner {
    None,
    E2(E2),
    G2(G2),
}

#[derive(Debug)]
pub struct EefRuntime {
    mounted_eef: MountedEefType,
    inner: EefRuntimeInner,
}

impl EefRuntime {
    pub fn new(mounted_eef: MountedEefType) -> Self {
        Self::with_motor_id(mounted_eef, DEFAULT_EEF_MOTOR_ID)
    }

    pub fn with_motor_id(mounted_eef: MountedEefType, motor_id: u16) -> Self {
        let inner = match &mounted_eef {
            MountedEefType::E2B => EefRuntimeInner::E2(E2::new(motor_id)),
            MountedEefType::G2 => EefRuntimeInner::G2(G2::new(motor_id)),
            _ => EefRuntimeInner::None,
        };
        Self { mounted_eef, inner }
    }

    pub fn mounted_eef(&self) -> &MountedEefType {
        &self.mounted_eef
    }

    pub fn validate_profile(&self, profile: EefRuntimeProfile) -> Result<(), EefRuntimeError> {
        if profile.matches_mounted_eef(&self.mounted_eef) {
            Ok(())
        } else {
            Err(EefRuntimeError::IncompatibleProfile {
                profile: profile.label(),
                mounted: self.mounted_eef.as_label().to_owned(),
            })
        }
    }

    pub fn subscribe_feedback(&self) -> Option<broadcast::Receiver<SingleEefFeedback>> {
        match &self.inner {
            EefRuntimeInner::None => None,
            EefRuntimeInner::E2(runtime) => Some(runtime.subscribe_feedback()),
            EefRuntimeInner::G2(runtime) => Some(runtime.subscribe_feedback()),
        }
    }

    pub fn latest_feedback(&self) -> Option<SingleEefFeedback> {
        match &self.inner {
            EefRuntimeInner::None => None,
            EefRuntimeInner::E2(runtime) => runtime.latest_feedback(),
            EefRuntimeInner::G2(runtime) => runtime.latest_feedback(),
        }
    }

    pub fn handle_raw_frame(&self, frame: &RawCanFrame) {
        match &self.inner {
            EefRuntimeInner::None => {}
            EefRuntimeInner::E2(runtime) => runtime.handle_raw_frame(frame),
            EefRuntimeInner::G2(runtime) => runtime.handle_raw_frame(frame),
        }
    }

    pub fn set_state(&self, state: EefState) -> Result<Vec<RawCanFrame>, EefRuntimeError> {
        match &self.inner {
            EefRuntimeInner::None => Err(self.unsupported_mounted_eef()),
            EefRuntimeInner::E2(runtime) => Ok(runtime.set_state(state)?),
            EefRuntimeInner::G2(runtime) => Ok(runtime.set_state(state)?),
        }
    }

    pub fn build_e2_command(
        &self,
        command: &SingleEefCommand,
    ) -> Result<Vec<RawCanFrame>, EefRuntimeError> {
        match &self.inner {
            EefRuntimeInner::None => Err(self.unsupported_mounted_eef()),
            EefRuntimeInner::E2(runtime) => Ok(runtime.build_mit_command(command)?),
            EefRuntimeInner::G2(_) => Err(self.unsupported_command("E2")),
        }
    }

    pub fn build_g2_mit_command(
        &self,
        command: &SingleEefCommand,
    ) -> Result<Vec<RawCanFrame>, EefRuntimeError> {
        match &self.inner {
            EefRuntimeInner::None => Err(self.unsupported_mounted_eef()),
            EefRuntimeInner::E2(_) => Err(self.unsupported_command("G2 MIT")),
            EefRuntimeInner::G2(runtime) => Ok(runtime.build_mit_command(command)?),
        }
    }

    pub fn submit_g2_mit_target(
        &self,
        command: &SingleEefCommand,
    ) -> Result<(), EefRuntimeError> {
        match &self.inner {
            EefRuntimeInner::None => Err(self.unsupported_mounted_eef()),
            EefRuntimeInner::E2(_) => Err(self.unsupported_command("G2 MIT")),
            EefRuntimeInner::G2(runtime) => Ok(runtime.submit_target(command)?),
        }
    }

    pub fn build_g2_pvt_command(
        &self,
        command: &SingleEefCommand,
    ) -> Result<Vec<RawCanFrame>, EefRuntimeError> {
        match &self.inner {
            EefRuntimeInner::None => Err(self.unsupported_mounted_eef()),
            EefRuntimeInner::E2(_) => Err(self.unsupported_command("G2 PVT")),
            EefRuntimeInner::G2(runtime) => Ok(runtime.build_pvt_command(command)?),
        }
    }

    pub fn shutdown_frames(&self) -> Result<Vec<RawCanFrame>, EefRuntimeError> {
        match &self.inner {
            EefRuntimeInner::None => Ok(Vec::new()),
            EefRuntimeInner::E2(runtime) => Ok(runtime.shutdown_frames()?),
            EefRuntimeInner::G2(runtime) => Ok(runtime.shutdown_frames()?),
        }
    }

    pub fn feedback_pump_interval(&self) -> Option<Duration> {
        match &self.inner {
            EefRuntimeInner::E2(_) => Some(E2_FEEDBACK_PUMP_INTERVAL),
            EefRuntimeInner::None | EefRuntimeInner::G2(_) => None,
        }
    }

    pub fn realtime_control_interval(&self) -> Option<Duration> {
        match &self.inner {
            EefRuntimeInner::G2(_) => Some(G2_CONTROL_INTERVAL),
            EefRuntimeInner::None | EefRuntimeInner::E2(_) => None,
        }
    }

    pub fn feedback_pump_frames(&self) -> Result<Option<Vec<RawCanFrame>>, EefRuntimeError> {
        match &self.inner {
            EefRuntimeInner::None => Ok(None),
            EefRuntimeInner::G2(_) => Ok(None),
            EefRuntimeInner::E2(runtime) => {
                if runtime.state() != EefState::Enabled {
                    return Ok(None);
                }
                Ok(Some(runtime.build_feedback_poll_command()?))
            }
        }
    }

    pub fn realtime_control_frames(&self) -> Result<Option<Vec<RawCanFrame>>, EefRuntimeError> {
        match &self.inner {
            EefRuntimeInner::None => Ok(None),
            EefRuntimeInner::E2(_) => Ok(None),
            EefRuntimeInner::G2(runtime) => Ok(runtime.control_frames()?),
        }
    }

    fn unsupported_command(&self, command: &'static str) -> EefRuntimeError {
        EefRuntimeError::UnsupportedCommand {
            command,
            mounted: self.mounted_eef.as_label().to_owned(),
        }
    }

    fn unsupported_mounted_eef(&self) -> EefRuntimeError {
        EefRuntimeError::UnsupportedMountedEef {
            mounted: self.mounted_eef.as_label().to_owned(),
        }
    }
}

pub fn spawn_eef_runtime_task(
    mut eef_rx: Option<mpsc::Receiver<RawCanFrame>>,
    eef: Arc<EefRuntime>,
    worker: Arc<CanWorker>,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let mut feedback_tick = eef.feedback_pump_interval().map(interval);
        if let Some(tick) = feedback_tick.as_mut() {
            tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
        }

        let mut control_tick = eef.realtime_control_interval().map(interval);
        if let Some(tick) = control_tick.as_mut() {
            tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
        }

        loop {
            tokio::select! {
                frame = async {
                    match eef_rx.as_mut() {
                        Some(frames_rx) => frames_rx.recv().await,
                        None => pending().await,
                    }
                } => {
                    match frame {
                        Some(frame) => eef.handle_raw_frame(&frame),
                        None => {
                            eef_rx = None;
                            if feedback_tick.is_none() && control_tick.is_none() {
                                break;
                            }
                        }
                    }
                }
                _ = async {
                    match feedback_tick.as_mut() {
                        Some(tick) => tick.tick().await,
                        None => pending().await,
                    }
                }, if feedback_tick.is_some() => {
                    match eef.feedback_pump_frames() {
                        Ok(Some(frames)) if !frames.is_empty() => {
                            if let Err(err) = worker.send_frames(CanTxPriority::Control, frames).await {
                                warn!(error = %err, "failed to send EEF feedback pump frames");
                                break;
                            }
                        }
                        Ok(_) => {}
                        Err(err) => warn!(error = %err, "failed to build EEF feedback pump frames"),
                    }
                }
                _ = async {
                    match control_tick.as_mut() {
                        Some(tick) => tick.tick().await,
                        None => pending().await,
                    }
                }, if control_tick.is_some() => {
                    match eef.realtime_control_frames() {
                        Ok(Some(frames)) if !frames.is_empty() => {
                            if let Err(err) = worker.send_frames(CanTxPriority::Control, frames).await {
                                warn!(error = %err, "failed to send EEF realtime control frames");
                                break;
                            }
                        }
                        Ok(_) => {}
                        Err(err) => warn!(error = %err, "failed to build EEF realtime control frames"),
                    }
                }
                else => break,
            }
        }
    })
}

#[cfg(test)]
mod tests {
    use super::{EefRuntime, EefRuntimeError, EefRuntimeProfile};
    use crate::eef::{EefState, SingleEefCommand};
    use crate::model::MountedEefType;
    use std::time::Duration;

    #[test]
    fn generic_profile_accepts_any_mounted_eef() {
        let runtime = EefRuntime::new(MountedEefType::Other("custom".to_owned()));
        assert!(runtime.validate_profile(EefRuntimeProfile::Generic).is_ok());
    }

    #[test]
    fn profile_validation_rejects_mismatched_eef() {
        let runtime = EefRuntime::new(MountedEefType::G2);
        let error = runtime
            .validate_profile(EefRuntimeProfile::E2)
            .expect_err("G2 runtime should reject the E2 profile");

        match error {
            EefRuntimeError::IncompatibleProfile { profile, mounted } => {
                assert_eq!(profile, "e2");
                assert_eq!(mounted, "G2");
            }
            other => panic!("unexpected error: {other:?}"),
        }
    }

    #[test]
    fn unsupported_command_reports_mounted_eef() {
        let runtime = EefRuntime::new(MountedEefType::G2);
        let error = runtime
            .build_e2_command(&SingleEefCommand::default())
            .expect_err("G2 runtime should reject E2 commands");

        match error {
            EefRuntimeError::UnsupportedCommand { command, mounted } => {
                assert_eq!(command, "E2");
                assert_eq!(mounted, "G2");
            }
            other => panic!("unexpected error: {other:?}"),
        }
    }

    #[test]
    fn unsupported_eef_has_no_feedback_stream() {
        let runtime = EefRuntime::new(MountedEefType::None);
        assert!(runtime.subscribe_feedback().is_none());
        assert!(runtime.latest_feedback().is_none());
    }

    #[test]
    fn e2_feedback_pump_only_runs_when_enabled() {
        let runtime = EefRuntime::new(MountedEefType::E2B);

        assert_eq!(
            runtime.feedback_pump_interval(),
            Some(Duration::from_millis(20))
        );
        assert!(
            runtime
                .feedback_pump_frames()
                .expect("feedback pump should not fail")
                .is_none()
        );

        runtime
            .set_state(EefState::Enabled)
            .expect("state transition should succeed");
        let frames = runtime
            .feedback_pump_frames()
            .expect("feedback pump frames should build")
            .expect("enabled E2 should emit poll frames");
        assert_eq!(frames.len(), 1);
    }
}
