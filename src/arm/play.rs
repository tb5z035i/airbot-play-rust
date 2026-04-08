use super::command_slot::{ARM_DOF, CommandSlot, JointTarget};
use crate::can::realtime::{configure_sched_fifo, lock_memory, set_current_thread_affinity};
use crate::can::worker::{CanTxPriority, CanWorker, CanWorkerError};
use crate::model::{
    KinematicsDynamicsBackend, ModelError, MountedEefType, Pose, gravity_coefficients_for_eef,
};
use crate::motor::MotorRuntime;
use crate::protocol::board::BoardProtocol;
use crate::protocol::board::play_base::PlayBaseBoardProtocol;
use crate::protocol::board::play_end::PlayEndBoardProtocol;
use crate::protocol::motor::MotorProtocol;
use crate::protocol::motor::dm::DmProtocol;
use crate::protocol::motor::od::OdProtocol;
use crate::request_service::{RequestError, RequestOutcome, RequestService};
use crate::types::{DecodedFrame, MotorCommand, ParamValue, ProtocolNodeKind, RawCanFrame};
use crate::warning_bus::WarningBus;
use crate::warnings::{WarningEvent, WarningKind};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::thread::JoinHandle as StdJoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use thiserror::Error;
use tokio::sync::{broadcast, mpsc};
use tokio::task::JoinHandle;
use tracing::warn;

const CONTROL_HZ: u64 = 250;
const CONTROL_PERIOD: Duration = Duration::from_millis(1000 / CONTROL_HZ);
const FEEDBACK_TIMEOUT: Duration = Duration::from_millis(100);
const STALE_COMMAND_THRESHOLD: Duration = Duration::from_millis(250);
const FOLLOWING_KP: [f64; ARM_DOF] = [200.0, 200.0, 200.0, 50.0, 50.0, 50.0];
const FOLLOWING_KD: [f64; ARM_DOF] = [3.0, 3.0, 3.0, 1.0, 1.0, 1.0];

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ArmState {
    Disabled,
    FreeDrive,
    CommandFollowing,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArmJointFeedback {
    pub positions: [f64; ARM_DOF],
    pub velocities: [f64; ARM_DOF],
    pub torques: [f64; ARM_DOF],
    pub valid: bool,
    pub timestamp_millis: u128,
}

#[derive(Clone, Debug)]
pub struct ArmBootstrapInfo {
    pub mounted_eef: MountedEefType,
    pub gravity_coefficients: [f64; ARM_DOF],
}

#[derive(Debug, Error)]
pub enum PlayArmError {
    #[error("control permission is required for this operation")]
    PermissionDenied,
    #[error("arm is not in command-following mode")]
    InvalidControlState,
    #[error("missing complete arm feedback")]
    MissingFeedback,
    #[error("arm runtime already started")]
    AlreadyStarted,
    #[error("request error: {0}")]
    Request(#[from] RequestError),
    #[error("CAN worker error: {0}")]
    Worker(#[from] CanWorkerError),
    #[error("model error: {0}")]
    Model(#[from] ModelError),
}

#[derive(Debug)]
struct BoardRuntime {
    play_base: PlayBaseBoardProtocol,
    play_end: PlayEndBoardProtocol,
    params: BTreeMap<String, ParamValue>,
}

impl Default for BoardRuntime {
    fn default() -> Self {
        Self {
            play_base: PlayBaseBoardProtocol::new(),
            play_end: PlayEndBoardProtocol::new(),
            params: BTreeMap::new(),
        }
    }
}

impl BoardRuntime {
    fn handle_raw_frame(&mut self, frame: &RawCanFrame) {
        for event in [self.play_base.inspect(frame), self.play_end.inspect(frame)]
            .into_iter()
            .flatten()
        {
            if let DecodedFrame::ParamResponse { values, .. } = event {
                self.params.extend(values);
            }
        }
    }
}

#[derive(Default)]
struct PlayArmHandles {
    tasks: Vec<JoinHandle<()>>,
    control_thread: Option<StdJoinHandle<()>>,
}

pub struct PlayArm {
    interface: String,
    mounted_eef: MountedEefType,
    gravity_coefficients: RwLock<[f64; ARM_DOF]>,
    control_model: Arc<dyn KinematicsDynamicsBackend>,
    ik_model: Arc<dyn KinematicsDynamicsBackend>,
    worker: Arc<CanWorker>,
    motors: Vec<Arc<MotorRuntime>>,
    warning_bus: WarningBus,
    command_slot: CommandSlot,
    state: RwLock<ArmState>,
    latest_feedback: RwLock<Option<ArmJointFeedback>>,
    latest_feedback_at: Mutex<Option<Instant>>,
    last_feedback_timeout_warning: Mutex<Option<Instant>>,
    last_stale_command_warning: Mutex<Option<Instant>>,
    feedback_tx: broadcast::Sender<ArmJointFeedback>,
    handles: Mutex<PlayArmHandles>,
    stop_flag: Arc<AtomicBool>,
}

impl PlayArm {
    pub fn new(
        interface: impl Into<String>,
        mounted_eef: MountedEefType,
        control_model: Arc<dyn KinematicsDynamicsBackend>,
        ik_model: Arc<dyn KinematicsDynamicsBackend>,
        worker: Arc<CanWorker>,
        motors: Vec<Arc<MotorRuntime>>,
        warning_bus: WarningBus,
    ) -> Self {
        let (feedback_tx, _) = broadcast::channel(256);
        Self {
            interface: interface.into(),
            gravity_coefficients: RwLock::new(gravity_coefficients_for_eef(&mounted_eef)),
            mounted_eef,
            control_model,
            ik_model,
            worker,
            motors,
            warning_bus,
            command_slot: CommandSlot::new(),
            state: RwLock::new(ArmState::Disabled),
            latest_feedback: RwLock::new(None),
            latest_feedback_at: Mutex::new(None),
            last_feedback_timeout_warning: Mutex::new(None),
            last_stale_command_warning: Mutex::new(None),
            feedback_tx,
            handles: Mutex::new(PlayArmHandles::default()),
            stop_flag: Arc::new(AtomicBool::new(false)),
        }
    }

    pub async fn bootstrap(worker: &CanWorker) -> Result<ArmBootstrapInfo, PlayArmError> {
        let request_service = RequestService::default();

        let mut end_protocol = PlayEndBoardProtocol::new();
        let end_frames = end_protocol
            .generate_param_get("eef_type")
            .map_err(|err| PlayArmError::Model(ModelError::Backend(err.to_string())))?;
        let end_outcome = request_service
            .exchange_via_worker(worker, &end_frames, |frame| end_protocol.inspect(frame))
            .await?;
        let mounted_eef = MountedEefType::from_code(extract_u32(&end_outcome).unwrap_or(0));

        let mut base_protocol = PlayBaseBoardProtocol::new();
        let gravity_frames = base_protocol
            .generate_param_get("gravity_comp_param")
            .map_err(|err| PlayArmError::Model(ModelError::Backend(err.to_string())))?;
        let gravity_outcome = request_service
            .exchange_via_worker(worker, &gravity_frames, |frame| {
                base_protocol.inspect(frame)
            })
            .await?;
        let gravity_coefficients = extract_gravity_coefficients(&gravity_outcome)
            .and_then(|values| values.get(mounted_eef.as_label()).copied())
            .unwrap_or_else(|| gravity_coefficients_for_eef(&mounted_eef));

        Ok(ArmBootstrapInfo {
            mounted_eef,
            gravity_coefficients,
        })
    }

    pub fn interface(&self) -> &str {
        &self.interface
    }

    pub fn mounted_eef(&self) -> &MountedEefType {
        &self.mounted_eef
    }

    pub fn gravity_coefficients(&self) -> [f64; ARM_DOF] {
        *self
            .gravity_coefficients
            .read()
            .expect("gravity coefficient lock poisoned")
    }

    pub fn set_gravity_coefficients(&self, coefficients: [f64; ARM_DOF]) {
        *self
            .gravity_coefficients
            .write()
            .expect("gravity coefficient lock poisoned") = coefficients;
    }

    pub fn state(&self) -> ArmState {
        *self.state.read().expect("arm state lock poisoned")
    }

    pub async fn set_state(&self, state: ArmState) -> Result<(), PlayArmError> {
        let previous = self.state();
        if previous == state {
            return Ok(());
        }

        if previous == ArmState::Disabled && state != ArmState::Disabled {
            let activation_frames = self.build_dm_activation_frames()?;
            if !activation_frames.is_empty() {
                self.worker
                    .send_frames(CanTxPriority::Lifecycle, activation_frames)
                    .await?;
            }
        }

        if state == ArmState::CommandFollowing {
            self.seed_command_target_from_feedback();
        }

        *self.state.write().expect("arm state lock poisoned") = state;
        if state == ArmState::Disabled {
            self.command_slot.clear();
            let disable_frames = self.build_dm_disable_frames()?;
            if !disable_frames.is_empty() {
                self.worker
                    .send_frames(CanTxPriority::Lifecycle, disable_frames)
                    .await?;
            }
        }

        Ok(())
    }

    pub fn subscribe_feedback(&self) -> broadcast::Receiver<ArmJointFeedback> {
        self.feedback_tx.subscribe()
    }

    pub fn latest_feedback(&self) -> Option<ArmJointFeedback> {
        self.latest_feedback
            .read()
            .expect("latest feedback lock poisoned")
            .clone()
    }

    pub fn latest_joint_target(&self) -> Option<JointTarget> {
        self.command_slot.latest().map(|snapshot| snapshot.target)
    }

    fn seed_command_target_from_feedback(&self) {
        if let Some(feedback) = self.latest_feedback() {
            self.command_slot.set(JointTarget::new(feedback.positions));
        } else {
            self.command_slot.clear();
        }
    }

    fn build_dm_activation_frames(&self) -> Result<Vec<RawCanFrame>, PlayArmError> {
        let mut frames = Vec::new();
        for motor in self
            .motors
            .iter()
            .filter(|motor| motor.kind() == ProtocolNodeKind::DmMotor)
        {
            let protocol = DmProtocol::new(motor.joint_id());
            frames.extend(
                protocol
                    .generate_enable()
                    .map_err(|err| PlayArmError::Model(ModelError::Backend(err.to_string())))?,
            );
            frames.extend(
                protocol
                    .generate_param_set("control_mode", &ParamValue::U32(0x01))
                    .map_err(|err| PlayArmError::Model(ModelError::Backend(err.to_string())))?,
            );
        }
        Ok(frames)
    }

    fn build_dm_disable_frames(&self) -> Result<Vec<RawCanFrame>, PlayArmError> {
        let mut frames = Vec::new();
        for motor in self
            .motors
            .iter()
            .filter(|motor| motor.kind() == ProtocolNodeKind::DmMotor)
        {
            frames.extend(
                DmProtocol::new(motor.joint_id())
                    .generate_disable()
                    .map_err(|err| PlayArmError::Model(ModelError::Backend(err.to_string())))?,
            );
        }
        Ok(frames)
    }

    pub fn submit_joint_target(
        &self,
        positions: [f64; ARM_DOF],
    ) -> Result<JointTarget, PlayArmError> {
        if self.state() != ArmState::CommandFollowing {
            return Err(PlayArmError::InvalidControlState);
        }

        let target = JointTarget::new(positions);
        self.command_slot.set(target.clone());
        Ok(target)
    }

    pub fn submit_task_target(&self, pose: &Pose) -> Result<JointTarget, PlayArmError> {
        if self.state() != ArmState::CommandFollowing {
            return Err(PlayArmError::InvalidControlState);
        }

        let seed = self
            .latest_feedback()
            .map(|feedback| feedback.positions.to_vec());
        let seed = seed.as_deref();
        let joints = self.ik_model.inverse_kinematics(pose, seed)?;
        let target = JointTarget::from_slice(&joints)
            .map_err(|err| PlayArmError::Model(ModelError::Backend(err.to_string())))?;
        self.command_slot.set(target.clone());
        Ok(target)
    }

    pub fn current_pose(&self) -> Result<Pose, PlayArmError> {
        let feedback = self
            .latest_feedback()
            .ok_or(PlayArmError::MissingFeedback)?;
        Ok(self.control_model.forward_kinematics(&feedback.positions)?)
    }

    pub fn start(
        self: &Arc<Self>,
        mut arm_rx: mpsc::Receiver<RawCanFrame>,
    ) -> Result<(), PlayArmError> {
        let mut handles = self.handles.lock().expect("arm handle lock poisoned");
        if !handles.tasks.is_empty() || handles.control_thread.is_some() {
            return Err(PlayArmError::AlreadyStarted);
        }

        handles.tasks.push(tokio::spawn(async move {
            let mut board_runtime = BoardRuntime::default();
            while let Some(frame) = arm_rx.recv().await {
                board_runtime.handle_raw_frame(&frame);
            }
        }));

        for motor in self.motors.iter().take(ARM_DOF) {
            let feedback_arm = Arc::clone(self);
            let mut snapshot_rx = motor.subscribe_snapshot();
            handles.tasks.push(tokio::spawn(async move {
                while snapshot_rx.changed().await.is_ok() {
                    feedback_arm.refresh_feedback_from_motors();
                }
            }));
        }

        self.refresh_feedback_from_motors();

        self.stop_flag.store(false, Ordering::Relaxed);
        let control_arm = Arc::clone(self);
        let stop_flag = Arc::clone(&self.stop_flag);
        handles.control_thread = Some(
            std::thread::Builder::new()
                .name("airbot-arm-control".to_owned())
                .spawn(move || control_arm.control_loop(stop_flag))
                .map_err(|err| PlayArmError::Model(ModelError::Backend(err.to_string())))?,
        );

        Ok(())
    }

    pub fn stop(&self) {
        self.stop_flag.store(true, Ordering::Relaxed);
        let (thread, tasks) = {
            let mut handles = self.handles.lock().expect("arm handle lock poisoned");
            let thread = handles.control_thread.take();
            let tasks = handles.tasks.drain(..).collect::<Vec<_>>();
            (thread, tasks)
        };

        if let Some(thread) = thread {
            let _ = thread.join();
        }

        for task in tasks {
            task.abort();
        }
    }

    fn refresh_feedback_from_motors(&self) {
        if let Some(feedback) = self.rebuild_feedback() {
            let mut latest = self
                .latest_feedback
                .write()
                .expect("latest feedback lock poisoned");
            let changed = latest.as_ref() != Some(&feedback);
            *latest = Some(feedback.clone());
            *self
                .latest_feedback_at
                .lock()
                .expect("feedback timestamp lock poisoned") = Some(Instant::now());

            if changed && self.state() != ArmState::Disabled {
                let _ = self.feedback_tx.send(feedback);
            }
        }
    }

    fn rebuild_feedback(&self) -> Option<ArmJointFeedback> {
        let mut positions = [0.0; ARM_DOF];
        let mut velocities = [0.0; ARM_DOF];
        let mut torques = [0.0; ARM_DOF];
        let mut valid = true;

        for (index, motor) in self.motors.iter().take(ARM_DOF).enumerate() {
            let state = motor.ready_state()?;
            positions[index] = state.pos;
            velocities[index] = state.vel;
            torques[index] = state.eff;
            valid &= state.is_valid;
        }

        Some(ArmJointFeedback {
            positions,
            velocities,
            torques,
            valid,
            timestamp_millis: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis(),
        })
    }

    fn control_loop(self: Arc<Self>, stop_flag: Arc<AtomicBool>) {
        if let Err(err) = configure_sched_fifo(70) {
            warn!(error = %err, "failed to configure arm control thread priority");
        }
        if let Err(err) = lock_memory() {
            warn!(error = %err, "failed to lock memory for arm control thread");
        }
        let _ = set_current_thread_affinity(0);

        let mut previous_tick = Instant::now();
        while !stop_flag.load(Ordering::Relaxed) {
            let tick_start = Instant::now();
            let observed_period = tick_start.saturating_duration_since(previous_tick);
            previous_tick = tick_start;

            if observed_period > CONTROL_PERIOD.mul_f64(1.25) {
                let observed_rate = 1.0 / observed_period.as_secs_f64();
                self.publish_warning(
                    WarningEvent::new(
                        WarningKind::ControlRateLow,
                        format!(
                            "control loop observed {:.2} Hz below the expected {} Hz",
                            observed_rate, CONTROL_HZ
                        ),
                    )
                    .with_interface(self.interface.clone())
                    .with_detail("observed_rate_hz", format!("{observed_rate:.2}"))
                    .with_detail("target_rate_hz", CONTROL_HZ.to_string()),
                );
            }

            if let Err(err) = self.tick_once() {
                match err {
                    PlayArmError::MissingFeedback => self.maybe_publish_feedback_timeout(),
                    other => self.publish_warning(
                        WarningEvent::new(
                            WarningKind::MalformedFrame,
                            format!("arm control tick failed: {other}"),
                        )
                        .with_interface(self.interface.clone()),
                    ),
                }
            }

            let elapsed = tick_start.elapsed();
            if elapsed > CONTROL_PERIOD {
                self.publish_warning(
                    WarningEvent::new(
                        WarningKind::ControlTickOverrun,
                        format!(
                            "control tick took {:.3} ms, above the {:.3} ms budget",
                            elapsed.as_secs_f64() * 1000.0,
                            CONTROL_PERIOD.as_secs_f64() * 1000.0,
                        ),
                    )
                    .with_interface(self.interface.clone())
                    .with_detail(
                        "elapsed_ms",
                        format!("{:.3}", elapsed.as_secs_f64() * 1000.0),
                    )
                    .with_detail(
                        "budget_ms",
                        format!("{:.3}", CONTROL_PERIOD.as_secs_f64() * 1000.0),
                    ),
                );
            }

            let sleep_for = CONTROL_PERIOD.saturating_sub(tick_start.elapsed());
            if !sleep_for.is_zero() {
                std::thread::sleep(sleep_for);
            }
        }
    }

    fn tick_once(&self) -> Result<(), PlayArmError> {
        let frames = match self.state() {
            ArmState::Disabled => Vec::new(),
            ArmState::FreeDrive => self.free_drive_frames()?,
            ArmState::CommandFollowing => self.command_following_frames()?,
        };

        if !frames.is_empty() {
            self.worker
                .blocking_send_frames(CanTxPriority::Control, frames)?;
        }

        Ok(())
    }

    fn free_drive_frames(&self) -> Result<Vec<RawCanFrame>, PlayArmError> {
        let feedback = self
            .latest_feedback()
            .ok_or(PlayArmError::MissingFeedback)?;
        let gravity = self.gravity_compensation(&feedback.positions)?;

        self.encode_mit_commands((0..ARM_DOF).map(|index| MotorCommand {
            pos: 0.0,
            vel: 0.0,
            eff: gravity[index],
            mit_kp: 0.0,
            mit_kd: 0.0,
            current_threshold: 0.0,
        }))
    }

    fn command_following_frames(&self) -> Result<Vec<RawCanFrame>, PlayArmError> {
        let feedback = self
            .latest_feedback()
            .ok_or(PlayArmError::MissingFeedback)?;
        let gravity = self.gravity_compensation(&feedback.positions)?;
        let target = self
            .command_slot
            .latest()
            .ok_or(PlayArmError::MissingFeedback)?;
        let target_positions = if target.age > STALE_COMMAND_THRESHOLD {
            self.maybe_publish_stale_command_warning(target.age);
            feedback.positions
        } else {
            target.target.positions
        };

        self.encode_mit_commands((0..ARM_DOF).map(|index| MotorCommand {
            pos: target_positions[index],
            vel: 0.0,
            eff: gravity[index],
            mit_kp: FOLLOWING_KP[index],
            mit_kd: FOLLOWING_KD[index],
            current_threshold: 0.0,
        }))
    }

    fn gravity_compensation(
        &self,
        joints: &[f64; ARM_DOF],
    ) -> Result<[f64; ARM_DOF], PlayArmError> {
        let velocities = [0.0; ARM_DOF];
        let accelerations = [0.0; ARM_DOF];
        let torques = self
            .control_model
            .inverse_dynamics(joints, &velocities, &accelerations)?;
        let coeffs = self.gravity_coefficients();

        let mut compensated = [0.0; ARM_DOF];
        for index in 0..ARM_DOF {
            compensated[index] = torques[index] * coeffs[index];
        }
        Ok(compensated)
    }

    fn encode_mit_commands<I>(&self, commands: I) -> Result<Vec<RawCanFrame>, PlayArmError>
    where
        I: IntoIterator<Item = MotorCommand>,
    {
        let mut frames = Vec::with_capacity(ARM_DOF);
        for (index, command) in commands.into_iter().enumerate() {
            let motor_id = (index + 1) as u16;
            let mut encoded = if index < 3 {
                OdProtocol::new(motor_id)
                    .generate_mit(&command)
                    .map_err(|err| PlayArmError::Model(ModelError::Backend(err.to_string())))?
            } else {
                DmProtocol::new(motor_id)
                    .generate_mit(&command)
                    .map_err(|err| PlayArmError::Model(ModelError::Backend(err.to_string())))?
            };
            frames.append(&mut encoded);
        }
        Ok(frames)
    }

    fn maybe_publish_feedback_timeout(&self) {
        let now = Instant::now();
        let latest_feedback = *self
            .latest_feedback_at
            .lock()
            .expect("feedback timestamp lock poisoned");
        let is_timed_out = latest_feedback
            .map(|timestamp| now.saturating_duration_since(timestamp) > FEEDBACK_TIMEOUT)
            .unwrap_or(true);

        if !is_timed_out {
            return;
        }

        let mut last_warning = self
            .last_feedback_timeout_warning
            .lock()
            .expect("feedback timeout warning lock poisoned");
        if last_warning
            .map(|timestamp| now.saturating_duration_since(timestamp) < FEEDBACK_TIMEOUT)
            .unwrap_or(false)
        {
            return;
        }

        *last_warning = Some(now);
        self.publish_warning(
            WarningEvent::new(
                WarningKind::RealtimeFeedbackTimeout,
                format!(
                    "arm feedback has not been refreshed within {} ms",
                    FEEDBACK_TIMEOUT.as_millis()
                ),
            )
            .with_interface(self.interface.clone())
            .with_detail("timeout_ms", FEEDBACK_TIMEOUT.as_millis().to_string()),
        );
    }

    fn maybe_publish_stale_command_warning(&self, age: Duration) {
        let now = Instant::now();
        let mut last_warning = self
            .last_stale_command_warning
            .lock()
            .expect("stale command warning lock poisoned");
        if last_warning
            .map(|timestamp| now.saturating_duration_since(timestamp) < STALE_COMMAND_THRESHOLD)
            .unwrap_or(false)
        {
            return;
        }

        *last_warning = Some(now);
        self.publish_warning(
            WarningEvent::new(
                WarningKind::StaleCommandReplay,
                format!(
                    "command slot is stale after {:.1} ms; holding current positions until a fresh command arrives",
                    age.as_secs_f64() * 1000.0
                ),
            )
            .with_interface(self.interface.clone())
            .with_detail("age_ms", format!("{:.1}", age.as_secs_f64() * 1000.0)),
        );
    }

    fn publish_warning(&self, warning: WarningEvent) {
        self.warning_bus.publish(warning);
    }
}

impl Drop for PlayArm {
    fn drop(&mut self) {
        self.stop_flag.store(true, Ordering::Relaxed);
        let handles = self.handles.get_mut().expect("arm handle lock poisoned");
        if let Some(thread) = handles.control_thread.take() {
            let _ = thread.join();
        }
        for task in handles.tasks.drain(..) {
            task.abort();
        }
    }
}

fn extract_u32(outcome: &RequestOutcome) -> Option<u32> {
    outcome
        .decoded_frames
        .iter()
        .find_map(|decoded| match decoded {
            DecodedFrame::ParamResponse { values, .. } => {
                values.values().find_map(|value| match value {
                    ParamValue::U32(value) => Some(*value),
                    _ => None,
                })
            }
            _ => None,
        })
}

fn extract_gravity_coefficients(outcome: &RequestOutcome) -> Option<BTreeMap<String, [f64; 6]>> {
    let values = outcome
        .decoded_frames
        .iter()
        .find_map(|decoded| match decoded {
            DecodedFrame::ParamResponse { values, .. } => {
                values.values().find_map(|value| match value {
                    ParamValue::FloatVec(values) if values.len() >= 24 => Some(values.clone()),
                    _ => None,
                })
            }
            _ => None,
        })?;

    let mut by_eef = BTreeMap::new();
    for (label, chunk) in [
        ("none", &values[0..6]),
        ("E2B", &values[6..12]),
        ("G2", &values[12..18]),
        ("other", &values[18..24]),
    ] {
        let mut coeffs = [0.0; 6];
        for (index, value) in chunk.iter().enumerate() {
            coeffs[index] = f64::from(*value);
        }
        by_eef.insert(label.to_owned(), coeffs);
    }
    Some(by_eef)
}

#[cfg(test)]
mod tests {
    use super::{
        ArmBootstrapInfo, ArmJointFeedback, ArmState, PlayArm, extract_gravity_coefficients,
    };
    use crate::arm::command_slot::JointTarget;
    use crate::can::worker::CanWorker;
    use crate::model::{KinematicsDynamicsBackend, ModelError, MountedEefType, Pose};
    use crate::motor::MotorRuntime;
    use crate::protocol::motor::MotorProtocol;
    use crate::protocol::motor::dm::DmProtocol;
    use crate::protocol::motor::od::OdProtocol;
    use crate::request_service::RequestOutcome;
    use crate::types::{
        DecodedFrame, FrameKind, ParamValue, ProtocolNode, ProtocolNodeKind, RawCanFrame,
    };
    use crate::warning_bus::WarningBus;
    use std::collections::BTreeMap;
    use std::sync::{Arc, Mutex};

    #[derive(Default)]
    struct DummyBackend {
        ik_result: Mutex<Vec<f64>>,
    }

    impl DummyBackend {
        fn with_ik_result(values: [f64; 6]) -> Self {
            Self {
                ik_result: Mutex::new(values.to_vec()),
            }
        }
    }

    impl KinematicsDynamicsBackend for DummyBackend {
        fn backend_name(&self) -> &'static str {
            "dummy"
        }

        fn dof(&self) -> usize {
            6
        }

        fn forward_kinematics(&self, joints: &[f64]) -> Result<Pose, ModelError> {
            Pose::from_slice(&[joints[0], joints[1], joints[2], 0.0, 0.0, 0.0, 1.0])
        }

        fn inverse_kinematics(
            &self,
            _target: &Pose,
            _seed: Option<&[f64]>,
        ) -> Result<Vec<f64>, ModelError> {
            Ok(self.ik_result.lock().expect("dummy IK lock").clone())
        }

        fn forward_dynamics(
            &self,
            _joints: &[f64],
            _vel: &[f64],
            torque: &[f64],
        ) -> Result<Vec<f64>, ModelError> {
            Ok(torque.to_vec())
        }

        fn inverse_dynamics(
            &self,
            joints: &[f64],
            _vel: &[f64],
            _acc: &[f64],
        ) -> Result<Vec<f64>, ModelError> {
            Ok(joints.to_vec())
        }
    }

    fn arm() -> Option<Arc<PlayArm>> {
        let worker = CanWorker::dummy_for_tests();
        let motors = (1_u16..=3)
            .map(|id| MotorRuntime::new_od("can0", id, Arc::clone(&worker), WarningBus::default()))
            .chain((4_u16..=6).map(|id| {
                MotorRuntime::new_dm("can0", id, Arc::clone(&worker), WarningBus::default())
            }))
            .collect();

        Some(Arc::new(PlayArm::new(
            "can0",
            MountedEefType::E2B,
            Arc::new(DummyBackend::with_ik_result([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])),
            Arc::new(DummyBackend::with_ik_result([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])),
            worker,
            motors,
            WarningBus::default(),
        )))
    }

    fn decode_command_positions(frames: &[RawCanFrame]) -> [f64; 6] {
        let mut positions = [0.0; 6];
        for (index, frame) in frames.iter().enumerate() {
            let motor_id = (index + 1) as u16;
            let decoded = if index < 3 {
                let mut protocol = OdProtocol::new(motor_id);
                protocol.inspect(frame)
            } else {
                let mut protocol = DmProtocol::new(motor_id);
                protocol.inspect(frame)
            }
            .expect("frame should decode into a motion command");
            match decoded {
                DecodedFrame::MotionCommand { command, .. } => positions[index] = command.pos,
                other => panic!("unexpected decoded frame: {other:?}"),
            }
        }
        positions
    }

    fn assert_positions_close(actual: [f64; 6], expected: [f64; 6]) {
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert!(
                (actual - expected).abs() < 0.002,
                "expected {expected}, got {actual}"
            );
        }
    }

    #[tokio::test]
    async fn realtime_commands_are_rejected_when_not_following() {
        let Some(arm) = arm() else {
            return;
        };
        let result = arm.submit_joint_target([0.0; 6]);
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn task_targets_are_normalized_into_joint_targets() {
        let Some(arm) = arm() else {
            return;
        };
        arm.set_state(ArmState::CommandFollowing)
            .await
            .expect("state transition should succeed");

        let pose = Pose::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]).unwrap();
        let target = arm
            .submit_task_target(&pose)
            .expect("task target should succeed");

        assert_eq!(target, JointTarget::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]));
        assert_eq!(arm.latest_joint_target().unwrap().positions[5], 6.0);
    }

    #[tokio::test]
    async fn entering_command_following_seeds_hold_position_target() {
        let Some(arm) = arm() else {
            return;
        };
        let feedback = ArmJointFeedback {
            positions: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            velocities: [0.0; 6],
            torques: [0.0; 6],
            valid: true,
            timestamp_millis: 0,
        };
        *arm.latest_feedback
            .write()
            .expect("latest feedback lock poisoned") = Some(feedback.clone());

        arm.set_state(ArmState::CommandFollowing)
            .await
            .expect("state transition should succeed");

        assert_eq!(
            arm.latest_joint_target().unwrap(),
            JointTarget::new(feedback.positions)
        );
    }

    #[test]
    fn current_pose_uses_latest_feedback_positions() {
        let Some(arm) = arm() else {
            return;
        };
        let feedback = ArmJointFeedback {
            positions: [0.11, -0.22, 0.33, 0.0, 0.0, 0.0],
            velocities: [0.0; 6],
            torques: [0.0; 6],
            valid: true,
            timestamp_millis: 0,
        };
        *arm.latest_feedback
            .write()
            .expect("latest feedback lock poisoned") = Some(feedback);

        let pose = arm
            .current_pose()
            .expect("current pose should be available");

        assert_eq!(pose.translation, [0.11, -0.22, 0.33]);
        assert_eq!(pose.rotation_xyzw, [0.0, 0.0, 0.0, 1.0]);
    }

    #[tokio::test]
    async fn stale_command_following_holds_current_positions() {
        let Some(arm) = arm() else {
            return;
        };
        let feedback = ArmJointFeedback {
            positions: [0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
            velocities: [0.0; 6],
            torques: [0.0; 6],
            valid: true,
            timestamp_millis: 0,
        };
        *arm.latest_feedback
            .write()
            .expect("latest feedback lock poisoned") = Some(feedback.clone());

        arm.set_state(ArmState::CommandFollowing)
            .await
            .expect("state transition should succeed");
        arm.submit_joint_target([1.0, 1.1, 1.2, 1.3, 1.4, 1.5])
            .expect("joint target should be accepted");

        std::thread::sleep(super::STALE_COMMAND_THRESHOLD + std::time::Duration::from_millis(20));

        let frames = arm
            .command_following_frames()
            .expect("command-following frames should build");

        assert_positions_close(decode_command_positions(&frames), feedback.positions);
    }

    #[test]
    fn dm_activation_frames_enable_each_joint_and_restore_control_mode() {
        let Some(arm) = arm() else {
            return;
        };
        let frames = arm
            .build_dm_activation_frames()
            .expect("DM activation frames should build");

        let expected = (4_u16..=6_u16)
            .flat_map(|motor_id| {
                let protocol = DmProtocol::new(motor_id);
                let mut frames = protocol
                    .generate_enable()
                    .expect("enable frame should build");
                frames.extend(
                    protocol
                        .generate_param_set("control_mode", &ParamValue::U32(0x01))
                        .expect("control mode frame should build"),
                );
                frames
            })
            .collect::<Vec<_>>();

        assert_eq!(frames, expected);
    }

    #[test]
    fn dm_disable_frames_target_only_dm_joints() {
        let Some(arm) = arm() else {
            return;
        };
        let frames = arm
            .build_dm_disable_frames()
            .expect("DM disable frames should build");
        let expected = vec![
            RawCanFrame::new(4, &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]).unwrap(),
            RawCanFrame::new(5, &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]).unwrap(),
            RawCanFrame::new(6, &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]).unwrap(),
        ];

        assert_eq!(frames, expected);
    }

    #[tokio::test]
    async fn arm_runtime_can_start_stop_and_restart() {
        let Some(arm) = arm() else {
            return;
        };

        let (_tx1, rx1) = tokio::sync::mpsc::channel(8);
        arm.start(rx1).expect("first start should succeed");
        arm.stop();

        let (_tx2, rx2) = tokio::sync::mpsc::channel(8);
        arm.start(rx2).expect("second start should succeed");
        arm.stop();
    }

    #[test]
    fn gravity_coefficients_are_chunked_by_eef() {
        let mut values = BTreeMap::new();
        values.insert(
            "gravity_comp_param".to_owned(),
            ParamValue::FloatVec((0..24).map(|value| value as f32).collect()),
        );
        let outcome = RequestOutcome {
            raw_frames: Vec::new(),
            decoded_frames: vec![DecodedFrame::ParamResponse {
                node: ProtocolNode {
                    kind: ProtocolNodeKind::PlayBaseBoard,
                    id: 0,
                },
                kind: FrameKind::GetParamResp,
                values,
            }],
            warnings: Vec::new(),
        };

        let by_eef = extract_gravity_coefficients(&outcome).expect("expected coefficients");
        assert_eq!(by_eef["none"], [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]);
        assert_eq!(by_eef["E2B"], [6.0, 7.0, 8.0, 9.0, 10.0, 11.0]);
        assert_eq!(by_eef["G2"], [12.0, 13.0, 14.0, 15.0, 16.0, 17.0]);
        assert_eq!(by_eef["other"], [18.0, 19.0, 20.0, 21.0, 22.0, 23.0]);
    }

    #[test]
    fn bootstrap_info_is_constructible() {
        let info = ArmBootstrapInfo {
            mounted_eef: MountedEefType::E2B,
            gravity_coefficients: [0.6, 0.6, 0.6, 1.0, 1.0, 1.0],
        };
        assert_eq!(info.gravity_coefficients[0], 0.6);
    }
}
