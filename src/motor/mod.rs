use crate::can::worker::{CanTxPriority, CanWorker, CanWorkerError};
use crate::protocol::motor::MotorProtocol;
use crate::protocol::motor::dm::DmProtocol;
use crate::protocol::motor::od::OdProtocol;
use crate::types::{DecodedFrame, MotorState, ParamValue, ProtocolNodeKind, RawCanFrame};
use crate::warning_bus::WarningBus;
use crate::warnings::{WarningEvent, WarningKind};
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use thiserror::Error;
use tokio::sync::{mpsc, watch};
use tokio::task::JoinHandle;

const DEFAULT_BOOTSTRAP_TIMEOUT: Duration = Duration::from_millis(500);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotorLifecyclePhase {
    Uninitialized,
    Disabling,
    Bootstrapping,
    ResettingError,
    Enabling,
    SettingMode,
    Ready,
    Faulted,
}

#[derive(Clone, Debug)]
pub struct MotorSnapshot {
    pub joint_id: u16,
    pub kind: ProtocolNodeKind,
    pub phase: MotorLifecyclePhase,
    pub state: Option<MotorState>,
    pub params: BTreeMap<String, ParamValue>,
}

impl MotorSnapshot {
    fn new(joint_id: u16, kind: ProtocolNodeKind) -> Self {
        Self {
            joint_id,
            kind,
            phase: MotorLifecyclePhase::Uninitialized,
            state: None,
            params: BTreeMap::new(),
        }
    }
}

#[derive(Debug, Error)]
pub enum MotorRuntimeError {
    #[error("CAN worker error: {0}")]
    Worker(#[from] CanWorkerError),
    #[error("bootstrap timed out waiting for `{0}`")]
    Timeout(String),
    #[error("protocol error: {0}")]
    Protocol(String),
    #[error("motor runtime already started")]
    AlreadyStarted,
    #[error("motor frame channel closed")]
    Closed,
}

#[derive(Debug)]
enum RuntimeProtocol {
    Od(OdProtocol),
    Dm(DmProtocol),
}

impl RuntimeProtocol {
    fn inspect(&mut self, frame: &RawCanFrame) -> Option<DecodedFrame> {
        match self {
            Self::Od(protocol) => protocol.inspect(frame),
            Self::Dm(protocol) => protocol.inspect(frame),
        }
    }

    fn bootstrap_params(&self) -> Vec<String> {
        match self {
            // The OD motors on current hardware do not reliably answer every readable diagnostic
            // register (for example `torque_factor`), but these three are sufficient to seed the
            // runtime state used by feedback display and gravity compensation.
            Self::Od(_) => ["position", "velocity", "torque"]
                .into_iter()
                .map(str::to_owned)
                .collect(),
            // These bounds affect DM feedback decoding; keep bootstrap limited to the values the
            // runtime actually uses instead of stalling on optional diagnostics.
            Self::Dm(_) => ["pos_max", "vel_max", "torque_max"]
                .into_iter()
                .map(str::to_owned)
                .collect(),
        }
    }

    fn generate_param_get(&self, name: &str) -> Result<Vec<RawCanFrame>, MotorRuntimeError> {
        match self {
            Self::Od(protocol) => protocol.generate_param_get(name),
            Self::Dm(protocol) => protocol.generate_param_get(name),
        }
        .map_err(|err| MotorRuntimeError::Protocol(err.to_string()))
    }

    fn generate_param_set(
        &self,
        name: &str,
        value: &ParamValue,
    ) -> Result<Vec<RawCanFrame>, MotorRuntimeError> {
        match self {
            Self::Od(protocol) => protocol.generate_param_set(name, value),
            Self::Dm(protocol) => protocol.generate_param_set(name, value),
        }
        .map_err(|err| MotorRuntimeError::Protocol(err.to_string()))
    }

    fn generate_disable(&self) -> Result<Vec<RawCanFrame>, MotorRuntimeError> {
        match self {
            Self::Od(_) => Ok(Vec::new()),
            Self::Dm(protocol) => protocol
                .generate_disable()
                .map_err(|err| MotorRuntimeError::Protocol(err.to_string())),
        }
    }

    fn generate_enable(&self) -> Result<Vec<RawCanFrame>, MotorRuntimeError> {
        match self {
            Self::Od(_) => Ok(Vec::new()),
            Self::Dm(protocol) => protocol
                .generate_enable()
                .map_err(|err| MotorRuntimeError::Protocol(err.to_string())),
        }
    }

    fn generate_reset_err(&self) -> Result<Vec<RawCanFrame>, MotorRuntimeError> {
        match self {
            Self::Od(_) => Ok(Vec::new()),
            Self::Dm(protocol) => protocol
                .generate_reset_err()
                .map_err(|err| MotorRuntimeError::Protocol(err.to_string())),
        }
    }

    fn apply_param_update(&mut self, name: &str, value: &ParamValue) {
        match (self, name, value) {
            (Self::Dm(protocol), "pos_max", ParamValue::F32(value)) => protocol.set_pos_max(*value),
            (Self::Dm(protocol), "vel_max", ParamValue::F32(value)) => protocol.set_vel_max(*value),
            (Self::Dm(protocol), "torque_max", ParamValue::F32(value)) => protocol.set_tor_max(*value),
            _ => {}
        }
    }
}

#[derive(Debug)]
pub struct MotorRuntime {
    interface: String,
    joint_id: u16,
    kind: ProtocolNodeKind,
    worker: Arc<CanWorker>,
    warning_bus: WarningBus,
    snapshot_tx: watch::Sender<MotorSnapshot>,
    snapshot_rx: watch::Receiver<MotorSnapshot>,
    tasks: Mutex<Vec<JoinHandle<()>>>,
}

impl MotorRuntime {
    pub fn new_od(
        interface: impl Into<String>,
        joint_id: u16,
        worker: Arc<CanWorker>,
        warning_bus: WarningBus,
    ) -> Arc<Self> {
        let snapshot = MotorSnapshot::new(joint_id, ProtocolNodeKind::OdMotor);
        let (snapshot_tx, snapshot_rx) = watch::channel(snapshot);
        Arc::new(Self {
            interface: interface.into(),
            joint_id,
            kind: ProtocolNodeKind::OdMotor,
            worker,
            warning_bus,
            snapshot_tx,
            snapshot_rx,
            tasks: Mutex::new(Vec::new()),
        })
    }

    pub fn new_dm(
        interface: impl Into<String>,
        joint_id: u16,
        worker: Arc<CanWorker>,
        warning_bus: WarningBus,
    ) -> Arc<Self> {
        let snapshot = MotorSnapshot::new(joint_id, ProtocolNodeKind::DmMotor);
        let (snapshot_tx, snapshot_rx) = watch::channel(snapshot);
        Arc::new(Self {
            interface: interface.into(),
            joint_id,
            kind: ProtocolNodeKind::DmMotor,
            worker,
            warning_bus,
            snapshot_tx,
            snapshot_rx,
            tasks: Mutex::new(Vec::new()),
        })
    }

    pub fn joint_id(&self) -> u16 {
        self.joint_id
    }

    pub fn kind(&self) -> ProtocolNodeKind {
        self.kind
    }

    pub fn phase(&self) -> MotorLifecyclePhase {
        self.snapshot_rx.borrow().phase
    }

    pub fn subscribe_snapshot(&self) -> watch::Receiver<MotorSnapshot> {
        self.snapshot_rx.clone()
    }

    pub fn latest_state(&self) -> Option<MotorState> {
        self.snapshot_rx.borrow().state.clone()
    }

    pub fn ready_state(&self) -> Option<MotorState> {
        let snapshot = self.snapshot_rx.borrow();
        if snapshot.phase == MotorLifecyclePhase::Ready {
            snapshot.state.clone()
        } else {
            None
        }
    }

    pub fn param(&self, name: &str) -> Option<ParamValue> {
        self.snapshot_rx.borrow().params.get(name).cloned()
    }

    pub fn snapshot(&self) -> MotorSnapshot {
        self.snapshot_rx.borrow().clone()
    }

    pub fn is_ready(&self) -> bool {
        self.phase() == MotorLifecyclePhase::Ready && self.latest_state().is_some()
    }

    pub fn start(
        self: &Arc<Self>,
        frames_rx: mpsc::Receiver<RawCanFrame>,
    ) -> Result<(), MotorRuntimeError> {
        let mut tasks = self.tasks.lock().expect("motor task lock poisoned");
        if !tasks.is_empty() {
            return Err(MotorRuntimeError::AlreadyStarted);
        }

        let interface = self.interface.clone();
        let joint_id = self.joint_id;
        let kind = self.kind;
        let worker = Arc::clone(&self.worker);
        let warning_bus = self.warning_bus.clone();
        let snapshot_tx = self.snapshot_tx.clone();
        tasks.push(tokio::spawn(async move {
            let protocol = match kind {
                ProtocolNodeKind::OdMotor => RuntimeProtocol::Od(OdProtocol::new(joint_id)),
                ProtocolNodeKind::DmMotor => RuntimeProtocol::Dm(DmProtocol::new(joint_id)),
                _ => return,
            };
            let mut actor = MotorActor {
                interface: interface.clone(),
                joint_id,
                kind,
                worker,
                warning_bus,
                protocol,
                snapshot: MotorSnapshot::new(joint_id, kind),
                snapshot_tx,
                frame_rx: frames_rx,
            };
            actor.publish_snapshot();

            if let Err(err) = actor.run().await {
                actor.set_phase(MotorLifecyclePhase::Faulted);
                actor.publish_warning(
                    WarningEvent::new(
                        WarningKind::MalformedFrame,
                        format!(
                            "{} motor {} bootstrap failed: {err}",
                            match actor.kind {
                                ProtocolNodeKind::OdMotor => "OD",
                                ProtocolNodeKind::DmMotor => "DM",
                                _ => "motor",
                            },
                            actor.joint_id
                        ),
                    )
                    .with_interface(actor.interface.clone()),
                );
            }
        }));

        Ok(())
    }

    pub fn stop(&self) {
        let mut tasks = self.tasks.lock().expect("motor task lock poisoned");
        for task in tasks.drain(..) {
            task.abort();
        }
    }

}

impl Drop for MotorRuntime {
    fn drop(&mut self) {
        let mut tasks = self.tasks.lock().expect("motor task lock poisoned");
        for task in tasks.drain(..) {
            task.abort();
        }
    }
}

struct MotorActor {
    interface: String,
    joint_id: u16,
    kind: ProtocolNodeKind,
    worker: Arc<CanWorker>,
    warning_bus: WarningBus,
    protocol: RuntimeProtocol,
    snapshot: MotorSnapshot,
    snapshot_tx: watch::Sender<MotorSnapshot>,
    frame_rx: mpsc::Receiver<RawCanFrame>,
}

impl MotorActor {
    async fn run(&mut self) -> Result<(), MotorRuntimeError> {
        self.bootstrap().await?;
        while let Some(frame) = self.frame_rx.recv().await {
            self.handle_raw_frame(&frame);
        }
        Ok(())
    }

    fn publish_snapshot(&self) {
        let _ = self.snapshot_tx.send(self.snapshot.clone());
    }

    fn handle_raw_frame(&mut self, frame: &RawCanFrame) {
        let Some(decoded) = self.protocol.inspect(frame) else {
            return;
        };

        let mut changed = false;
        match decoded {
            DecodedFrame::MotionFeedback { node, state } if node.id == self.joint_id && node.kind == self.kind => {
                self.snapshot.state = Some(state);
                changed = true;
            }
            DecodedFrame::ParamResponse { node, values, .. } if node.id == self.joint_id && node.kind == self.kind => {
                for (name, value) in values {
                    self.protocol.apply_param_update(&name, &value);
                    self.snapshot.params.insert(name, value);
                }
                if self.kind == ProtocolNodeKind::OdMotor {
                    self.maybe_seed_od_state();
                }
                changed = true;
            }
            _ => {}
        }

        if changed {
            self.publish_snapshot();
        }
    }

    fn maybe_seed_od_state(&mut self) {
        if self.snapshot.state.is_some() {
            return;
        }

        let Some(ParamValue::F32(pos)) = self.snapshot.params.get("position") else {
            return;
        };
        let Some(ParamValue::F32(vel)) = self.snapshot.params.get("velocity") else {
            return;
        };
        let Some(ParamValue::F32(torque)) = self.snapshot.params.get("torque") else {
            return;
        };

        self.snapshot.state = Some(MotorState {
            is_valid: true,
            joint_id: self.joint_id,
            pos: f64::from(*pos),
            vel: f64::from(*vel),
            eff: f64::from(*torque),
            motor_temp: 0,
            mos_temp: 0,
            error_id: 0,
        });
    }

    async fn bootstrap(&mut self) -> Result<(), MotorRuntimeError> {
        match self.kind {
            ProtocolNodeKind::OdMotor => self.bootstrap_od().await,
            ProtocolNodeKind::DmMotor => self.bootstrap_dm().await,
            _ => Ok(()),
        }
    }

    async fn bootstrap_od(&mut self) -> Result<(), MotorRuntimeError> {
        self.set_phase(MotorLifecyclePhase::Bootstrapping);
        let readable = self.protocol.bootstrap_params();

        for name in readable {
            let frames = self.protocol.generate_param_get(&name)?;
            self.worker
                .send_frames(CanTxPriority::Lifecycle, frames)
                .await?;
            self.wait_until(
                |snapshot| snapshot.params.contains_key(&name),
                DEFAULT_BOOTSTRAP_TIMEOUT,
                format!("param `{name}`"),
            )
            .await?;
        }

        self.maybe_seed_od_state();
        self.publish_snapshot();
        self.wait_until(
            |snapshot| snapshot.state.is_some(),
            DEFAULT_BOOTSTRAP_TIMEOUT,
            format!("state for motor {}", self.joint_id),
        )
        .await?;
        self.set_phase(MotorLifecyclePhase::Ready);
        Ok(())
    }

    async fn bootstrap_dm(&mut self) -> Result<(), MotorRuntimeError> {
        self.set_phase(MotorLifecyclePhase::Disabling);
        let disable_frames = self.protocol.generate_disable()?;
        self.worker
            .send_frames(CanTxPriority::Lifecycle, disable_frames)
            .await?;
        self.wait_until(
            |snapshot| snapshot.state.is_some(),
            DEFAULT_BOOTSTRAP_TIMEOUT,
            format!("state for motor {}", self.joint_id),
        )
        .await?;

        self.set_phase(MotorLifecyclePhase::Bootstrapping);
        let readable = self.protocol.bootstrap_params();
        for name in readable {
            let frames = self.protocol.generate_param_get(&name)?;
            self.worker
                .send_frames(CanTxPriority::Lifecycle, frames)
                .await?;
            self.wait_until(
                |snapshot| snapshot.params.contains_key(&name),
                DEFAULT_BOOTSTRAP_TIMEOUT,
                format!("param `{name}`"),
            )
            .await?;
        }

        self.set_phase(MotorLifecyclePhase::ResettingError);
        let reset_frames = self.protocol.generate_reset_err()?;
        self.worker
            .send_frames(CanTxPriority::Lifecycle, reset_frames)
            .await?;

        self.set_phase(MotorLifecyclePhase::Enabling);
        let enable_frames = self.protocol.generate_enable()?;
        self.worker
            .send_frames(CanTxPriority::Lifecycle, enable_frames)
            .await?;

        self.set_phase(MotorLifecyclePhase::SettingMode);
        let mode_frames = self.protocol.generate_param_set("control_mode", &ParamValue::U32(0x01))?;
        self.worker
            .send_frames(CanTxPriority::Lifecycle, mode_frames)
            .await?;
        self.wait_until(
            |snapshot| snapshot.params.get("control_mode") == Some(&ParamValue::U32(0x01)),
            DEFAULT_BOOTSTRAP_TIMEOUT,
            "param `control_mode` = 1".to_owned(),
        )
        .await?;

        self.set_phase(MotorLifecyclePhase::Ready);
        Ok(())
    }

    async fn wait_until<F>(
        &mut self,
        mut predicate: F,
        timeout: Duration,
        label: String,
    ) -> Result<(), MotorRuntimeError>
    where
        F: FnMut(&MotorSnapshot) -> bool,
    {
        let deadline = tokio::time::Instant::now() + timeout;
        loop {
            if predicate(&self.snapshot) {
                return Ok(());
            }

            match tokio::time::timeout_at(deadline, self.frame_rx.recv()).await {
                Ok(Some(frame)) => self.handle_raw_frame(&frame),
                Ok(None) => return Err(MotorRuntimeError::Closed),
                Err(_) => return Err(MotorRuntimeError::Timeout(label)),
            }
        }
    }

    fn set_phase(&mut self, phase: MotorLifecyclePhase) {
        self.snapshot.phase = phase;
        self.publish_snapshot();
    }

    fn publish_warning(&self, warning: WarningEvent) {
        self.warning_bus.publish(warning);
    }
}

#[cfg(test)]
mod tests {
    use super::{MotorActor, MotorSnapshot, RuntimeProtocol};
    use crate::can::worker::CanWorker;
    use crate::protocol::motor::dm::DmProtocol;
    use crate::protocol::motor::od::OdProtocol;
    use crate::types::ParamValue;
    use crate::types::RawCanFrame;
    use crate::warning_bus::WarningBus;
    use std::sync::Arc;
    use tokio::sync::{mpsc, watch};

    fn worker() -> Arc<CanWorker> {
        CanWorker::dummy_for_tests()
    }

    fn od_actor(joint_id: u16) -> MotorActor {
        let (snapshot_tx, _snapshot_rx) = watch::channel(MotorSnapshot::new(
            joint_id,
            crate::types::ProtocolNodeKind::OdMotor,
        ));
        let (_frame_tx, frame_rx) = mpsc::channel(1);
        MotorActor {
            interface: "can0".to_owned(),
            joint_id,
            kind: crate::types::ProtocolNodeKind::OdMotor,
            worker: worker(),
            warning_bus: WarningBus::default(),
            protocol: RuntimeProtocol::Od(OdProtocol::new(joint_id)),
            snapshot: MotorSnapshot::new(joint_id, crate::types::ProtocolNodeKind::OdMotor),
            snapshot_tx,
            frame_rx,
        }
    }

    fn dm_actor(joint_id: u16) -> MotorActor {
        let (snapshot_tx, _snapshot_rx) = watch::channel(MotorSnapshot::new(
            joint_id,
            crate::types::ProtocolNodeKind::DmMotor,
        ));
        let (_frame_tx, frame_rx) = mpsc::channel(1);
        MotorActor {
            interface: "can0".to_owned(),
            joint_id,
            kind: crate::types::ProtocolNodeKind::DmMotor,
            worker: worker(),
            warning_bus: WarningBus::default(),
            protocol: RuntimeProtocol::Dm(DmProtocol::new(joint_id)),
            snapshot: MotorSnapshot::new(joint_id, crate::types::ProtocolNodeKind::DmMotor),
            snapshot_tx,
            frame_rx,
        }
    }

    fn od_param_response_frame(motor_id: u16, param_id: u8, value: f32) -> RawCanFrame {
        let bytes = value.to_be_bytes();
        RawCanFrame::new(
            motor_id as u32 | 0x100,
            &[param_id, 0x01, bytes[0], bytes[1], bytes[2], bytes[3]],
        )
        .unwrap()
    }

    fn dm_param_response_frame(motor_id: u16, param_id: u8, value: f32) -> RawCanFrame {
        let bytes = value.to_le_bytes();
        RawCanFrame::new(
            motor_id as u32 | 0x700,
            &[
                (motor_id & 0xFF) as u8,
                (motor_id >> 8) as u8,
                0x33,
                param_id,
                bytes[0],
                bytes[1],
                bytes[2],
                bytes[3],
            ],
        )
        .unwrap()
    }

    #[test]
    fn od_runtime_seeds_initial_state_from_param_responses() {
        let mut actor = od_actor(1);

        actor.handle_raw_frame(&od_param_response_frame(1, 0xE1, 1.5));
        actor.handle_raw_frame(&od_param_response_frame(1, 0xE2, -0.25));
        actor.handle_raw_frame(&od_param_response_frame(1, 0xE3, 0.75));

        let state = actor.snapshot.state.expect("OD runtime should seed initial state");
        assert!((state.pos - 1.5).abs() < 1e-6);
        assert!((state.vel + 0.25).abs() < 1e-6);
        assert!((state.eff - 0.75).abs() < 1e-6);
        assert_eq!(actor.snapshot.params.get("position"), Some(&ParamValue::F32(1.5)));
        assert_eq!(actor.snapshot.params.get("velocity"), Some(&ParamValue::F32(-0.25)));
        assert_eq!(actor.snapshot.params.get("torque"), Some(&ParamValue::F32(0.75)));
    }

    #[test]
    fn dm_disable_trace_frame_seeds_state() {
        let mut actor = dm_actor(4);
        let frame = RawCanFrame::new(0x704, &[0x04, 0x80, 0x08, 0x7F, 0xD7, 0xFF, 0x1E, 0x1C]).unwrap();

        actor.handle_raw_frame(&frame);

        let state = actor
            .snapshot
            .state
            .expect("DM disable reply should seed current state");
        assert!((state.pos - 0.003242).abs() < 1e-3);
        assert!((state.vel - (-0.036631)).abs() < 1e-3);
        assert!((state.eff - (-0.002442)).abs() < 1e-3);
        assert_eq!(state.motor_temp, 30);
        assert_eq!(state.mos_temp, 28);
        assert_eq!(state.error_id, 0);
    }

    #[test]
    fn dm_param_updates_adjust_feedback_decode_ranges() {
        let mut actor = dm_actor(4);

        actor.handle_raw_frame(&dm_param_response_frame(4, 0x15, 6.0));
        actor.handle_raw_frame(&dm_param_response_frame(4, 0x16, 12.0));
        actor.handle_raw_frame(&dm_param_response_frame(4, 0x17, 5.0));

        let frame = RawCanFrame::new(0x704, &[0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00]).unwrap();
        actor.handle_raw_frame(&frame);

        let state = actor
            .snapshot
            .state
            .expect("DM state should decode using updated limits");
        assert!((state.pos - 6.0).abs() < 1e-3);
        assert!((state.vel - 12.0).abs() < 1e-3);
        assert!((state.eff - 5.0).abs() < 1e-3);
        assert_eq!(actor.snapshot.params.get("pos_max"), Some(&ParamValue::F32(6.0)));
        assert_eq!(actor.snapshot.params.get("vel_max"), Some(&ParamValue::F32(12.0)));
        assert_eq!(actor.snapshot.params.get("torque_max"), Some(&ParamValue::F32(5.0)));
    }

    #[test]
    fn bootstrap_param_sets_are_trimmed_to_runtime_essentials() {
        let od = RuntimeProtocol::Od(OdProtocol::new(1));
        let dm = RuntimeProtocol::Dm(DmProtocol::new(4));

        assert_eq!(
            od.bootstrap_params(),
            vec!["position".to_owned(), "velocity".to_owned(), "torque".to_owned()]
        );
        assert_eq!(
            dm.bootstrap_params(),
            vec!["pos_max".to_owned(), "vel_max".to_owned(), "torque_max".to_owned()]
        );
    }
}

