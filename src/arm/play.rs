use super::command_slot::{ARM_DOF, CommandSlot, JointTarget};
use crate::model::{KinematicsDynamicsBackend, ModelError, MountedEefType, Pose, gravity_coefficients_for_eef};
use crate::protocol::motor::dm::DmProtocol;
use crate::protocol::motor::od::OdProtocol;
use crate::protocol::motor::MotorProtocol;
use crate::session::{RoutedFrame, SessionRouter};
use crate::types::{DecodedFrame, MotorCommand, MotorState, ParamValue, ProtocolNodeKind, RawCanFrame};
use crate::warning_bus::WarningBus;
use crate::warnings::{WarningEvent, WarningKind};
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use thiserror::Error;
use tokio::sync::broadcast;
use tokio::task::JoinHandle;

const CONTROL_HZ: u64 = 250;
const CONTROL_PERIOD: Duration = Duration::from_millis(1000 / CONTROL_HZ);
const FEEDBACK_TIMEOUT: Duration = Duration::from_millis(100);
const STALE_COMMAND_THRESHOLD: Duration = Duration::from_millis(250);
const FOLLOWING_KP: [f64; ARM_DOF] = [200.0, 200.0, 200.0, 50.0, 50.0, 50.0];
const FOLLOWING_KD: [f64; ARM_DOF] = [3.0, 3.0, 3.0, 1.0, 1.0, 1.0];
/// `MotorControlMode::MIT` in airbot_hardware (airbot/hardware/include/airbot_hardware/utils.hpp).
const MIT_CONTROL_MODE_U32: u32 = 0x01;

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

#[derive(Debug, Error)]
pub enum PlayArmError {
    #[error("arm is not in command-following mode")]
    InvalidControlState,
    #[error("missing complete arm feedback")]
    MissingFeedback,
    #[error("arm runtime already started")]
    AlreadyStarted,
    #[error("model error: {0}")]
    Model(#[from] ModelError),
}

pub struct PlayArm {
    interface: String,
    mounted_eef: MountedEefType,
    gravity_coefficients: RwLock<[f64; ARM_DOF]>,
    model: Arc<dyn KinematicsDynamicsBackend>,
    warning_bus: WarningBus,
    command_slot: CommandSlot,
    state: RwLock<ArmState>,
    motor_states: Mutex<Vec<Option<MotorState>>>,
    latest_feedback: RwLock<Option<ArmJointFeedback>>,
    latest_feedback_at: Mutex<Option<Instant>>,
    last_feedback_timeout_warning: Mutex<Option<Instant>>,
    last_stale_command_warning: Mutex<Option<Instant>>,
    feedback_tx: broadcast::Sender<ArmJointFeedback>,
    tasks: Mutex<Vec<JoinHandle<()>>>,
}

impl PlayArm {
    pub fn new(
        interface: impl Into<String>,
        mounted_eef: MountedEefType,
        model: Arc<dyn KinematicsDynamicsBackend>,
        warning_bus: WarningBus,
    ) -> Self {
        let (feedback_tx, _) = broadcast::channel(256);
        Self {
            interface: interface.into(),
            gravity_coefficients: RwLock::new(gravity_coefficients_for_eef(&mounted_eef)),
            mounted_eef,
            model,
            warning_bus,
            command_slot: CommandSlot::new(),
            state: RwLock::new(ArmState::Disabled),
            motor_states: Mutex::new(vec![None; ARM_DOF]),
            latest_feedback: RwLock::new(None),
            latest_feedback_at: Mutex::new(None),
            last_feedback_timeout_warning: Mutex::new(None),
            last_stale_command_warning: Mutex::new(None),
            feedback_tx,
            tasks: Mutex::new(Vec::new()),
        }
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

    pub fn set_state(&self, state: ArmState) {
        *self.state.write().expect("arm state lock poisoned") = state;
        if state == ArmState::Disabled {
            self.command_slot.clear();
        }
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

    pub fn submit_joint_target(&self, positions: [f64; ARM_DOF]) -> Result<JointTarget, PlayArmError> {
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

        let seed = self.latest_feedback().map(|feedback| feedback.positions.to_vec());
        let seed = seed.as_deref();
        let joints = self.model.inverse_kinematics(pose, seed)?;
        let target = JointTarget::from_slice(&joints)
            .map_err(|err| PlayArmError::Model(ModelError::Backend(err.to_string())))?;
        self.command_slot.set(target.clone());
        Ok(target)
    }

    pub fn handle_routed_frame(&self, routed: &RoutedFrame) {
        let mut changed = false;
        {
            let mut states = self.motor_states.lock().expect("motor states lock poisoned");
            for decoded in &routed.decoded_frames {
                if let DecodedFrame::MotionFeedback { node, state } = decoded {
                    let joint_index = Self::joint_index(node.kind, node.id);
                    if let Some(index) = joint_index {
                        states[index] = Some(state.clone());
                        changed = true;
                    }
                }
            }
        }

        if changed {
            *self
                .latest_feedback_at
                .lock()
                .expect("feedback timestamp lock poisoned") = Some(Instant::now());

            if let Some(feedback) = self.rebuild_feedback() {
                *self
                    .latest_feedback
                    .write()
                    .expect("latest feedback lock poisoned") = Some(feedback.clone());

                if self.state() != ArmState::Disabled {
                    let _ = self.feedback_tx.send(feedback);
                }
            }
        }
    }

    pub fn start(self: &Arc<Self>, session: Arc<SessionRouter>) -> Result<(), PlayArmError> {
        let mut tasks = self.tasks.lock().expect("arm task lock poisoned");
        if !tasks.is_empty() {
            return Err(PlayArmError::AlreadyStarted);
        }

        let feedback_arm = Arc::clone(self);
        let mut frames_rx = session.subscribe_frames();
        tasks.push(tokio::spawn(async move {
            loop {
                match frames_rx.recv().await {
                    Ok(routed) => feedback_arm.handle_routed_frame(&routed),
                    Err(broadcast::error::RecvError::Closed) => break,
                    Err(broadcast::error::RecvError::Lagged(skipped)) => {
                        feedback_arm.publish_warning(
                            WarningEvent::new(
                                WarningKind::UnmatchedFrame,
                                format!("arm feedback consumer lagged by {skipped} routed frames"),
                            )
                            .with_interface(feedback_arm.interface.clone()),
                        );
                    }
                }
            }
        }));

        let startup_arm = Arc::clone(self);
        let startup_io = session.io().clone();
        tasks.push(tokio::spawn(async move {
            startup_arm.bring_up_dm_motors(startup_io.as_ref()).await;
        }));

        let control_arm = Arc::clone(self);
        tasks.push(tokio::spawn(async move {
            let mut interval = tokio::time::interval(CONTROL_PERIOD);
            let mut previous_tick = Instant::now();
            loop {
                interval.tick().await;
                let tick_start = Instant::now();
                let observed_period = tick_start.saturating_duration_since(previous_tick);
                previous_tick = tick_start;

                if observed_period > CONTROL_PERIOD.mul_f64(1.25) {
                    let observed_rate = 1.0 / observed_period.as_secs_f64();
                    control_arm.publish_warning(
                        WarningEvent::new(
                            WarningKind::ControlRateLow,
                            format!(
                                "control loop observed {:.2} Hz below the expected {} Hz",
                                observed_rate, CONTROL_HZ
                            ),
                        )
                        .with_interface(control_arm.interface.clone())
                        .with_detail("observed_rate_hz", format!("{observed_rate:.2}"))
                        .with_detail("target_rate_hz", CONTROL_HZ.to_string()),
                    );
                }

                if let Err(err) = control_arm.tick_once(session.io().as_ref()).await {
                    match err {
                        PlayArmError::MissingFeedback => {
                            control_arm.maybe_publish_feedback_timeout();
                        }
                        other => {
                            control_arm.publish_warning(
                                WarningEvent::new(
                                    WarningKind::MalformedFrame,
                                    format!("arm control tick failed: {other}"),
                                )
                                .with_interface(control_arm.interface.clone()),
                            );
                        }
                    }
                }

                let elapsed = tick_start.elapsed();
                if elapsed > CONTROL_PERIOD {
                    control_arm.publish_warning(
                        WarningEvent::new(
                            WarningKind::ControlTickOverrun,
                            format!(
                                "control tick took {:.3} ms, above the {:.3} ms budget",
                                elapsed.as_secs_f64() * 1000.0,
                                CONTROL_PERIOD.as_secs_f64() * 1000.0,
                            ),
                        )
                        .with_interface(control_arm.interface.clone())
                        .with_detail("elapsed_ms", format!("{:.3}", elapsed.as_secs_f64() * 1000.0))
                        .with_detail(
                            "budget_ms",
                            format!("{:.3}", CONTROL_PERIOD.as_secs_f64() * 1000.0),
                        ),
                    );
                }
            }
        }));

        Ok(())
    }

    /// Match [`airbot::hardware::Arm::enable`] + `arm.set_param("arm.control_mode", MIT)` for DM joints
    /// (motors 4–6): enable drives, then set `control_mode` to MIT. OD joints are left to the firmware /
    /// MIT stream, same as the reference stack.
    async fn bring_up_dm_motors(&self, io: &crate::can::socketcan_io::SocketCanIo) {
        for motor_id in 4_u16..=6_u16 {
            let proto = DmProtocol::new(motor_id);
            let frames = match proto.generate_enable() {
                Ok(f) => f,
                Err(err) => {
                    self.publish_warning(
                        WarningEvent::new(
                            WarningKind::MalformedFrame,
                            format!("DM motor {motor_id} enable encode failed: {err}"),
                        )
                        .with_interface(self.interface.clone()),
                    );
                    return;
                }
            };
            for frame in frames {
                if let Err(err) = io.send(&frame).await {
                    self.publish_warning(
                        WarningEvent::new(
                            WarningKind::MalformedFrame,
                            format!("DM motor {motor_id} enable send failed: {err}"),
                        )
                        .with_interface(self.interface.clone()),
                    );
                    return;
                }
            }
        }

        for motor_id in 4_u16..=6_u16 {
            let proto = DmProtocol::new(motor_id);
            let frames = match proto.generate_param_set(
                "control_mode",
                &ParamValue::U32(MIT_CONTROL_MODE_U32),
            ) {
                Ok(f) => f,
                Err(err) => {
                    self.publish_warning(
                        WarningEvent::new(
                            WarningKind::MalformedFrame,
                            format!("DM motor {motor_id} control_mode MIT encode failed: {err}"),
                        )
                        .with_interface(self.interface.clone()),
                    );
                    return;
                }
            };
            for frame in frames {
                if let Err(err) = io.send(&frame).await {
                    self.publish_warning(
                        WarningEvent::new(
                            WarningKind::MalformedFrame,
                            format!("DM motor {motor_id} control_mode MIT send failed: {err}"),
                        )
                        .with_interface(self.interface.clone()),
                    );
                    return;
                }
            }
        }
    }

    pub fn stop(&self) {
        let mut tasks = self.tasks.lock().expect("arm task lock poisoned");
        for task in tasks.drain(..) {
            task.abort();
        }
    }

    async fn tick_once(&self, io: &crate::can::socketcan_io::SocketCanIo) -> Result<(), PlayArmError> {
        let frames = match self.state() {
            ArmState::Disabled => Vec::new(),
            ArmState::FreeDrive => self.free_drive_frames()?,
            ArmState::CommandFollowing => self.command_following_frames()?,
        };

        for frame in frames {
            io.send(&frame).await.map_err(|err| {
                PlayArmError::Model(ModelError::Backend(format!("failed to send control frame: {err}")))
            })?;
        }

        Ok(())
    }

    fn rebuild_feedback(&self) -> Option<ArmJointFeedback> {
        let states = self.motor_states.lock().expect("motor states lock poisoned");
        if states.iter().any(Option::is_none) {
            return None;
        }

        let mut positions = [0.0; ARM_DOF];
        let mut velocities = [0.0; ARM_DOF];
        let mut torques = [0.0; ARM_DOF];
        let mut valid = true;

        for (index, state) in states.iter().enumerate() {
            let state = state.as_ref().expect("guarded by any(None) check");
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

    fn free_drive_frames(&self) -> Result<Vec<RawCanFrame>, PlayArmError> {
        let positions = self
            .latest_feedback()
            .map(|fb| fb.positions)
            .unwrap_or([0.0; ARM_DOF]);
        let gravity = self.gravity_compensation(&positions)?;

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
        let positions = self
            .latest_feedback()
            .map(|fb| fb.positions)
            .unwrap_or([0.0; ARM_DOF]);
        let gravity = self.gravity_compensation(&positions)?;
        let target = self.command_slot.latest().ok_or(PlayArmError::MissingFeedback)?;

        if target.age > STALE_COMMAND_THRESHOLD {
            self.maybe_publish_stale_command_warning(target.age);
        }

        self.encode_mit_commands((0..ARM_DOF).map(|index| MotorCommand {
            pos: target.target.positions[index],
            vel: 0.0,
            eff: gravity[index],
            mit_kp: FOLLOWING_KP[index],
            mit_kd: FOLLOWING_KD[index],
            current_threshold: 0.0,
        }))
    }

    fn gravity_compensation(&self, joints: &[f64; ARM_DOF]) -> Result<[f64; ARM_DOF], PlayArmError> {
        let velocities = [0.0; ARM_DOF];
        let accelerations = [0.0; ARM_DOF];
        let torques = self.model.inverse_dynamics(joints, &velocities, &accelerations)?;
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
                    "command slot is stale after {:.1} ms but is still being replayed",
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

    fn joint_index(kind: ProtocolNodeKind, id: u16) -> Option<usize> {
        match (kind, id) {
            (ProtocolNodeKind::OdMotor, 1..=3) => Some((id - 1) as usize),
            (ProtocolNodeKind::DmMotor, 4..=6) => Some((id - 1) as usize),
            _ => None,
        }
    }
}

impl Drop for PlayArm {
    fn drop(&mut self) {
        let mut tasks = self.tasks.lock().expect("arm task lock poisoned");
        for task in tasks.drain(..) {
            task.abort();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{ArmState, PlayArm};
    use crate::arm::command_slot::JointTarget;
    use crate::model::{KinematicsDynamicsBackend, ModelError, MountedEefType, Pose};
    use crate::session::RoutedFrame;
    use crate::types::{DecodedFrame, MotorState, ProtocolNode, ProtocolNodeKind, RawCanFrame};
    use crate::warning_bus::WarningBus;
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

        fn inverse_kinematics(&self, _target: &Pose, _seed: Option<&[f64]>) -> Result<Vec<f64>, ModelError> {
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

    fn arm() -> PlayArm {
        PlayArm::new(
            "can0",
            MountedEefType::E2B,
            Arc::new(DummyBackend::with_ik_result([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])),
            WarningBus::default(),
        )
    }

    fn feedback_frame(joint_id: u16, kind: ProtocolNodeKind, pos: f64) -> DecodedFrame {
        DecodedFrame::MotionFeedback {
            node: ProtocolNode { kind, id: joint_id },
            state: MotorState {
                joint_id,
                pos,
                vel: 0.1 * joint_id as f64,
                eff: 0.2 * joint_id as f64,
                ..MotorState::default()
            },
        }
    }

    #[test]
    fn realtime_commands_are_rejected_when_not_following() {
        let arm = arm();
        let result = arm.submit_joint_target([0.0; 6]);
        assert!(result.is_err());
    }

    #[test]
    fn task_targets_are_normalized_into_joint_targets() {
        let arm = arm();
        arm.set_state(ArmState::CommandFollowing);

        let pose = Pose::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]).unwrap();
        let target = arm.submit_task_target(&pose).expect("task target should succeed");

        assert_eq!(target, JointTarget::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]));
        assert_eq!(arm.latest_joint_target().unwrap().positions[5], 6.0);
    }

    #[test]
    fn feedback_is_gated_by_arm_state() {
        let arm = arm();
        let mut rx = arm.subscribe_feedback();
        let routed = RoutedFrame {
            interface: "can0".to_owned(),
            raw_frame: RawCanFrame::new(0x001, &[0x00]).unwrap(),
            decoded_frames: vec![
                feedback_frame(1, ProtocolNodeKind::OdMotor, 1.0),
                feedback_frame(2, ProtocolNodeKind::OdMotor, 2.0),
                feedback_frame(3, ProtocolNodeKind::OdMotor, 3.0),
                feedback_frame(4, ProtocolNodeKind::DmMotor, 4.0),
                feedback_frame(5, ProtocolNodeKind::DmMotor, 5.0),
                feedback_frame(6, ProtocolNodeKind::DmMotor, 6.0),
            ],
        };

        arm.handle_routed_frame(&routed);
        assert!(rx.try_recv().is_err());

        arm.set_state(ArmState::FreeDrive);
        arm.handle_routed_frame(&routed);
        let feedback = rx.try_recv().expect("feedback should be published once active");
        assert_eq!(feedback.positions, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }
}
