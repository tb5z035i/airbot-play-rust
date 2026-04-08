use crate::arm::{ArmJointFeedback, ArmState, JointTarget, PlayArm, PlayArmError};
use crate::can::router::CanFrameRouter;
use crate::can::worker::{
    CanTxPriority, CanWorker, CanWorkerBackend, CanWorkerConfig, CanWorkerError,
};
use crate::eef::{E2, EefState, G2, SingleEefCommand, SingleEefFeedback};
use crate::model::{ModelBackendKind, ModelError, ModelRegistry, MountedEefType, Pose};
use crate::motor::{MotorRuntime, MotorRuntimeError};
use crate::protocol::board::BoardProtocol;
use crate::protocol::board::play_base::PlayBaseBoardProtocol;
use crate::protocol::board::play_end::PlayEndBoardProtocol;
use crate::protocol::motor::MotorProtocol;
use crate::protocol::motor::dm::DmProtocol;
use crate::protocol::motor::od::OdProtocol;
use crate::request_service::{RequestError, RequestOutcome, RequestService};
use crate::types::RawCanFrame;
use crate::types::{DecodedFrame, ParamValue, ProtocolNodeKind};
use crate::warning_bus::WarningBus;
use crate::warnings::WarningEvent;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use thiserror::Error;
use tokio::sync::broadcast;
use tokio::task::JoinHandle;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AccessMode {
    Readonly,
    Control,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum RequestTarget {
    PlayBaseBoard,
    PlayEndBoard,
    OdMotor(u16),
    DmMotor(u16),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConnectedRobotInfo {
    pub interface: String,
    pub access_mode: AccessMode,
    pub mounted_eef: MountedEefType,
    pub model_backend: ModelBackendKind,
    pub gravity_coefficients: [f64; 6],
}

#[derive(Debug, Error)]
pub enum ClientError {
    #[error("control permission is required for this operation")]
    PermissionDenied,
    #[error("CAN worker error: {0}")]
    Worker(#[from] CanWorkerError),
    #[error("request error: {0}")]
    Request(#[from] RequestError),
    #[error("motor runtime error: {0}")]
    Motor(#[from] MotorRuntimeError),
    #[error("arm runtime error: {0}")]
    Arm(#[from] PlayArmError),
    #[error("model error: {0}")]
    Model(#[from] ModelError),
    #[error("unsupported request target `{0:?}`")]
    UnsupportedRequestTarget(RequestTarget),
}

pub struct AirbotPlayClient {
    info: ConnectedRobotInfo,
    request_service: RequestService,
    worker: Arc<CanWorker>,
    frame_router: Arc<CanFrameRouter>,
    arm: Arc<PlayArm>,
    e2: Arc<E2>,
    g2: Arc<G2>,
    warning_bus: WarningBus,
    eef_task: Mutex<Option<JoinHandle<()>>>,
}

impl AirbotPlayClient {
    pub async fn connect_readonly(interface: impl Into<String>) -> Result<Self, ClientError> {
        Self::connect_with_backends(
            interface,
            AccessMode::Readonly,
            CanWorkerBackend::AsyncFd,
            ModelBackendKind::PlayAnalytical,
        )
        .await
    }

    pub async fn connect_control(interface: impl Into<String>) -> Result<Self, ClientError> {
        Self::connect_with_backends(
            interface,
            AccessMode::Control,
            CanWorkerBackend::AsyncFd,
            ModelBackendKind::PlayAnalytical,
        )
        .await
    }

    pub async fn connect_readonly_with_backend(
        interface: impl Into<String>,
        backend: CanWorkerBackend,
    ) -> Result<Self, ClientError> {
        Self::connect_with_backends(
            interface,
            AccessMode::Readonly,
            backend,
            ModelBackendKind::PlayAnalytical,
        )
        .await
    }

    pub async fn connect_control_with_backend(
        interface: impl Into<String>,
        backend: CanWorkerBackend,
    ) -> Result<Self, ClientError> {
        Self::connect_with_backends(
            interface,
            AccessMode::Control,
            backend,
            ModelBackendKind::PlayAnalytical,
        )
        .await
    }

    pub async fn connect_readonly_with_backends(
        interface: impl Into<String>,
        can_backend: CanWorkerBackend,
        model_backend: ModelBackendKind,
    ) -> Result<Self, ClientError> {
        Self::connect_with_backends(interface, AccessMode::Readonly, can_backend, model_backend)
            .await
    }

    pub async fn connect_control_with_backends(
        interface: impl Into<String>,
        can_backend: CanWorkerBackend,
        model_backend: ModelBackendKind,
    ) -> Result<Self, ClientError> {
        Self::connect_with_backends(interface, AccessMode::Control, can_backend, model_backend)
            .await
    }

    pub async fn connect(
        interface: impl Into<String>,
        access_mode: AccessMode,
    ) -> Result<Self, ClientError> {
        Self::connect_with_backends(
            interface,
            access_mode,
            CanWorkerBackend::AsyncFd,
            ModelBackendKind::PlayAnalytical,
        )
        .await
    }

    pub async fn connect_with_backends(
        interface: impl Into<String>,
        access_mode: AccessMode,
        can_backend: CanWorkerBackend,
        model_backend: ModelBackendKind,
    ) -> Result<Self, ClientError> {
        let interface = interface.into();
        let request_service = RequestService::default();
        let worker = CanWorker::open(CanWorkerConfig {
            interface: interface.clone(),
            backend: can_backend,
            ..CanWorkerConfig::default()
        })?;
        let bootstrap = PlayArm::bootstrap(worker.as_ref()).await?;

        let control_model = ModelRegistry::load(model_backend, bootstrap.mounted_eef.clone())?;
        let ik_model = ModelRegistry::load(model_backend, bootstrap.mounted_eef.clone())?;
        let warning_bus = WarningBus::default();
        let (frame_router, mut routes) = CanFrameRouter::new(
            Arc::clone(&worker),
            1_u16..=6_u16,
            matches!(
                bootstrap.mounted_eef,
                MountedEefType::E2B | MountedEefType::G2
            )
            .then_some(7),
        );
        let motors = (1_u16..=3)
            .map(|id| {
                MotorRuntime::new_od(
                    interface.clone(),
                    id,
                    Arc::clone(&worker),
                    warning_bus.clone(),
                )
            })
            .chain((4_u16..=6).map(|id| {
                MotorRuntime::new_dm(
                    interface.clone(),
                    id,
                    Arc::clone(&worker),
                    warning_bus.clone(),
                )
            }))
            .collect::<Vec<_>>();
        frame_router
            .start()
            .map_err(|err| ClientError::Model(ModelError::Backend(err.to_string())))?;
        for motor in &motors {
            let frame_rx = routes
                .motor_rxs
                .remove(&motor.joint_id())
                .expect("missing motor frame receiver");
            motor.start(frame_rx)?;
        }

        let arm = Arc::new(PlayArm::new(
            interface.clone(),
            bootstrap.mounted_eef.clone(),
            control_model,
            ik_model,
            Arc::clone(&worker),
            motors,
            warning_bus.clone(),
        ));
        arm.set_gravity_coefficients(bootstrap.gravity_coefficients);

        let e2 = Arc::new(E2::new(7));
        let g2 = Arc::new(G2::new(7));
        arm.start(routes.arm_rx)?;

        let eef_task = spawn_eef_dispatcher(routes.eef_rx, Arc::clone(&e2), Arc::clone(&g2));

        Ok(Self {
            info: ConnectedRobotInfo {
                interface,
                access_mode,
                mounted_eef: bootstrap.mounted_eef,
                model_backend,
                gravity_coefficients: bootstrap.gravity_coefficients,
            },
            request_service,
            worker,
            frame_router,
            arm,
            e2,
            g2,
            warning_bus,
            eef_task: Mutex::new(Some(eef_task)),
        })
    }

    pub fn info(&self) -> &ConnectedRobotInfo {
        &self.info
    }

    pub fn worker(&self) -> &Arc<CanWorker> {
        &self.worker
    }

    pub fn arm(&self) -> &Arc<PlayArm> {
        &self.arm
    }

    pub fn mounted_eef(&self) -> &MountedEefType {
        &self.info.mounted_eef
    }

    pub fn subscribe_arm_feedback(&self) -> broadcast::Receiver<ArmJointFeedback> {
        self.arm.subscribe_feedback()
    }

    pub fn subscribe_warnings(&self) -> broadcast::Receiver<WarningEvent> {
        self.warning_bus.subscribe()
    }

    pub fn subscribe_e2_feedback(&self) -> Option<broadcast::Receiver<SingleEefFeedback>> {
        matches!(self.info.mounted_eef, MountedEefType::E2B).then(|| self.e2.subscribe_feedback())
    }

    pub fn subscribe_g2_feedback(&self) -> Option<broadcast::Receiver<SingleEefFeedback>> {
        matches!(self.info.mounted_eef, MountedEefType::G2).then(|| self.g2.subscribe_feedback())
    }

    pub async fn query_param(
        &self,
        target: RequestTarget,
        name: &str,
    ) -> Result<RequestOutcome, ClientError> {
        let frames = match target {
            RequestTarget::PlayBaseBoard => PlayBaseBoardProtocol::new().generate_param_get(name),
            RequestTarget::PlayEndBoard => PlayEndBoardProtocol::new().generate_param_get(name),
            RequestTarget::OdMotor(id) => OdProtocol::new(id).generate_param_get(name),
            RequestTarget::DmMotor(id) => DmProtocol::new(id).generate_param_get(name),
        }
        .map_err(|err| ClientError::Model(ModelError::Backend(err.to_string())))?;

        let outcome = match target {
            RequestTarget::PlayBaseBoard => {
                let mut protocol = PlayBaseBoardProtocol::new();
                self.request_service
                    .exchange_via_worker(&self.worker, &frames, |frame| {
                        match protocol.inspect(frame) {
                            Some(
                                ref decoded @ DecodedFrame::ParamResponse {
                                    node, ref values, ..
                                },
                            ) if request_target_matches(target, node.kind, node.id)
                                && values.contains_key(name) =>
                            {
                                Some(decoded.clone())
                            }
                            _ => None,
                        }
                    })
                    .await?
            }
            RequestTarget::PlayEndBoard => {
                let mut protocol = PlayEndBoardProtocol::new();
                self.request_service
                    .exchange_via_worker(&self.worker, &frames, |frame| {
                        match protocol.inspect(frame) {
                            Some(
                                ref decoded @ DecodedFrame::ParamResponse {
                                    node, ref values, ..
                                },
                            ) if request_target_matches(target, node.kind, node.id)
                                && values.contains_key(name) =>
                            {
                                Some(decoded.clone())
                            }
                            _ => None,
                        }
                    })
                    .await?
            }
            RequestTarget::OdMotor(id) => {
                let mut protocol = OdProtocol::new(id);
                self.request_service
                    .exchange_via_worker(&self.worker, &frames, |frame| {
                        match protocol.inspect(frame) {
                            Some(
                                ref decoded @ DecodedFrame::ParamResponse {
                                    node, ref values, ..
                                },
                            ) if request_target_matches(target, node.kind, node.id)
                                && values.contains_key(name) =>
                            {
                                Some(decoded.clone())
                            }
                            _ => None,
                        }
                    })
                    .await?
            }
            RequestTarget::DmMotor(id) => {
                let mut protocol = DmProtocol::new(id);
                self.request_service
                    .exchange_via_worker(&self.worker, &frames, |frame| {
                        match protocol.inspect(frame) {
                            Some(
                                ref decoded @ DecodedFrame::ParamResponse {
                                    node, ref values, ..
                                },
                            ) if request_target_matches(target, node.kind, node.id)
                                && values.contains_key(name) =>
                            {
                                Some(decoded.clone())
                            }
                            _ => None,
                        }
                    })
                    .await?
            }
        };

        Ok(outcome)
    }

    pub async fn query_mounted_eef(&self) -> Result<MountedEefType, ClientError> {
        let outcome = self
            .query_param(RequestTarget::PlayEndBoard, "eef_type")
            .await?;
        let code = extract_u32(&outcome).unwrap_or(0);
        Ok(MountedEefType::from_code(code))
    }

    pub async fn query_gravity_coefficients(
        &self,
    ) -> Result<BTreeMap<String, [f64; 6]>, ClientError> {
        let outcome = self
            .query_param(RequestTarget::PlayBaseBoard, "gravity_comp_param")
            .await?;
        Ok(extract_gravity_coefficients(&outcome).unwrap_or_else(|| {
            let mut fallback = BTreeMap::new();
            fallback.insert(
                self.info.mounted_eef.as_label().to_owned(),
                self.info.gravity_coefficients,
            );
            fallback
        }))
    }

    pub fn query_current_pose(&self) -> Result<Pose, ClientError> {
        Ok(self.arm.current_pose()?)
    }

    pub async fn set_arm_state(&self, state: ArmState) -> Result<(), ClientError> {
        self.require_control()?;
        self.arm.set_state(state).await?;
        Ok(())
    }

    pub fn submit_joint_target(&self, positions: [f64; 6]) -> Result<JointTarget, ClientError> {
        self.require_control()?;
        Ok(self.arm.submit_joint_target(positions)?)
    }

    pub fn submit_task_target(&self, pose: &Pose) -> Result<JointTarget, ClientError> {
        self.require_control()?;
        Ok(self.arm.submit_task_target(pose)?)
    }

    pub fn set_eef_state(&self, state: EefState) -> Result<(), ClientError> {
        self.require_control()?;
        match self.info.mounted_eef {
            MountedEefType::E2B => self.e2.set_state(state),
            MountedEefType::G2 => self.g2.set_state(state),
            _ => {}
        }
        Ok(())
    }

    pub async fn submit_e2_command(
        &self,
        command: &SingleEefCommand,
    ) -> Result<Vec<RawCanFrame>, ClientError> {
        self.require_control()?;
        let frames = self
            .e2
            .build_mit_command(command)
            .map_err(|err| ClientError::Model(ModelError::Backend(err.to_string())))?;
        self.send_frames(&frames).await?;
        Ok(frames)
    }

    pub async fn submit_g2_mit_command(
        &self,
        command: &SingleEefCommand,
    ) -> Result<Vec<RawCanFrame>, ClientError> {
        self.require_control()?;
        let frames = self
            .g2
            .build_mit_command(command)
            .map_err(|err| ClientError::Model(ModelError::Backend(err.to_string())))?;
        self.send_frames(&frames).await?;
        Ok(frames)
    }

    pub async fn submit_g2_pvt_command(
        &self,
        command: &SingleEefCommand,
    ) -> Result<Vec<RawCanFrame>, ClientError> {
        self.require_control()?;
        let frames = self
            .g2
            .build_pvt_command(command)
            .map_err(|err| ClientError::Model(ModelError::Backend(err.to_string())))?;
        self.send_frames(&frames).await?;
        Ok(frames)
    }

    pub async fn shutdown_gracefully(&self) -> Result<(), ClientError> {
        self.arm.stop();

        let disable_frames = build_shutdown_disable_frames()?;
        if !disable_frames.is_empty() {
            self.worker
                .send_frames(CanTxPriority::Lifecycle, disable_frames)
                .await?;
        }

        self.frame_router.stop();
        if let Some(task) = self.eef_task.lock().expect("eef task lock poisoned").take() {
            task.abort();
        }
        self.worker.shutdown().await;
        Ok(())
    }

    pub fn shutdown(&self) {
        self.arm.stop();
        self.frame_router.stop();
        if let Some(task) = self.eef_task.lock().expect("eef task lock poisoned").take() {
            task.abort();
        }
    }

    fn require_control(&self) -> Result<(), ClientError> {
        match self.info.access_mode {
            AccessMode::Readonly => Err(ClientError::PermissionDenied),
            AccessMode::Control => Ok(()),
        }
    }

    async fn send_frames(&self, frames: &[RawCanFrame]) -> Result<(), ClientError> {
        self.worker
            .send_frames(CanTxPriority::Control, frames.to_vec())
            .await
            .map_err(ClientError::from)
    }
}

impl Drop for AirbotPlayClient {
    fn drop(&mut self) {
        self.arm.stop();
        self.frame_router.stop();
        if let Some(task) = self
            .eef_task
            .get_mut()
            .expect("eef task lock poisoned")
            .take()
        {
            task.abort();
        }
    }
}

fn spawn_eef_dispatcher(
    mut eef_rx: Option<tokio::sync::mpsc::Receiver<RawCanFrame>>,
    e2: Arc<E2>,
    g2: Arc<G2>,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let Some(ref mut frames_rx) = eef_rx else {
            return;
        };
        while let Some(frame) = frames_rx.recv().await {
            e2.handle_raw_frame(&frame);
            g2.handle_raw_frame(&frame);
        }
    })
}

fn build_shutdown_disable_frames() -> Result<Vec<RawCanFrame>, ClientError> {
    let mut frames = Vec::new();
    for motor_id in 4_u16..=6_u16 {
        frames.extend(
            DmProtocol::new(motor_id)
                .generate_disable()
                .map_err(|err| ClientError::Model(ModelError::Backend(err.to_string())))?,
        );
    }

    Ok(frames)
}

fn request_target_matches(target: RequestTarget, kind: ProtocolNodeKind, id: u16) -> bool {
    match (target, kind, id) {
        (RequestTarget::PlayBaseBoard, ProtocolNodeKind::PlayBaseBoard, _) => true,
        (RequestTarget::PlayEndBoard, ProtocolNodeKind::PlayEndBoard, _) => true,
        (RequestTarget::OdMotor(expected), ProtocolNodeKind::OdMotor, actual) => expected == actual,
        (RequestTarget::DmMotor(expected), ProtocolNodeKind::DmMotor, actual) => expected == actual,
        _ => false,
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
        AccessMode, ConnectedRobotInfo, build_shutdown_disable_frames, extract_gravity_coefficients,
    };
    use crate::model::ModelBackendKind;
    use crate::request_service::RequestOutcome;
    use crate::types::{DecodedFrame, FrameKind, ParamValue, ProtocolNode, ProtocolNodeKind};
    use std::collections::BTreeMap;

    #[test]
    fn readonly_and_control_modes_compare_as_expected() {
        assert_ne!(AccessMode::Readonly, AccessMode::Control);
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
    fn connected_info_is_serializable_shape() {
        let info = ConnectedRobotInfo {
            interface: "can0".to_owned(),
            access_mode: AccessMode::Control,
            mounted_eef: crate::model::MountedEefType::E2B,
            model_backend: ModelBackendKind::Pinocchio,
            gravity_coefficients: [0.6, 0.6, 0.6, 1.338, 1.236, 0.893],
        };

        assert_eq!(info.interface, "can0");
        assert_eq!(info.gravity_coefficients[5], 0.893);
    }

    #[test]
    fn graceful_shutdown_disables_only_dm_arm_motors() {
        let frames = build_shutdown_disable_frames().expect("disable frames should build");
        let ids = frames.iter().map(|frame| frame.can_id).collect::<Vec<_>>();
        assert_eq!(ids, vec![4, 5, 6]);
        for frame in frames {
            assert_eq!(
                frame.payload(),
                &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
            );
        }
    }
}
