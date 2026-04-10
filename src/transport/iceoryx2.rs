use crate::arm::{ARM_DOF, ArmJointFeedback, ArmState, JointTarget};
use crate::can::worker::CanWorkerBackend;
use crate::client::{
    AccessMode, AirbotPlayClient, ClientError, ConnectedRobotInfo, RequestTarget,
};
use crate::eef::{EefState, SingleEefCommand, SingleEefFeedback};
use crate::model::{ModelBackendKind, MountedEefType, Pose};
use crate::request_service::RequestOutcome;
use crate::warnings::WarningEvent;
use async_trait::async_trait;
use iceoryx2::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::future::Future;
use std::sync::Arc;
use std::time::Duration;
use thiserror::Error;
use tokio::sync::broadcast;
use tokio::time::{MissedTickBehavior, interval};

type EventPublisher = iceoryx2::port::publisher::Publisher<ipc::Service, [u8], ()>;
type RequestSubscriber = iceoryx2::port::subscriber::Subscriber<ipc::Service, [u8], ()>;

const DEFAULT_MAX_MESSAGE_SIZE: usize = 16 * 1024;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SerializableArmFeedback {
    pub positions: [f64; ARM_DOF],
    pub velocities: [f64; ARM_DOF],
    pub torques: [f64; ARM_DOF],
    pub valid: bool,
    pub timestamp_millis: u64,
}

impl From<ArmJointFeedback> for SerializableArmFeedback {
    fn from(value: ArmJointFeedback) -> Self {
        Self {
            positions: value.positions,
            velocities: value.velocities,
            torques: value.torques,
            valid: value.valid,
            timestamp_millis: value.timestamp_millis.min(u64::MAX as u128) as u64,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SerializableEefFeedback {
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
    pub valid: bool,
    pub timestamp_millis: u64,
}

impl From<SingleEefFeedback> for SerializableEefFeedback {
    fn from(value: SingleEefFeedback) -> Self {
        Self {
            position: value.position,
            velocity: value.velocity,
            effort: value.effort,
            valid: value.valid,
            timestamp_millis: value.timestamp_millis.min(u64::MAX as u128) as u64,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Iceoryx2TransportConfig {
    pub service_root: String,
    pub interface: String,
    pub allow_control: bool,
    pub can_backend: CanWorkerBackend,
    pub model_backend: ModelBackendKind,
    pub poll_interval: Duration,
    pub max_message_size: usize,
}

impl Default for Iceoryx2TransportConfig {
    fn default() -> Self {
        Self {
            service_root: "airbot-play/can0".to_owned(),
            interface: "can0".to_owned(),
            allow_control: true,
            can_backend: CanWorkerBackend::AsyncFd,
            model_backend: ModelBackendKind::PlayAnalytical,
            poll_interval: Duration::from_millis(10),
            max_message_size: DEFAULT_MAX_MESSAGE_SIZE,
        }
    }
}

impl Iceoryx2TransportConfig {
    pub fn request_service_name(&self) -> String {
        format!("{}/requests", self.service_root)
    }

    pub fn event_service_name(&self) -> String {
        format!("{}/events", self.service_root)
    }
}

#[derive(Debug, Error)]
pub enum Iceoryx2TransportError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("client error: {0}")]
    Client(#[from] ClientError),
    #[error("json error: {0}")]
    Json(#[from] serde_json::Error),
    #[error("service_root must not be empty")]
    InvalidServiceRoot,
    #[error("poll_interval must be greater than zero")]
    InvalidPollInterval,
    #[error("max_message_size must be greater than zero")]
    InvalidMaxMessageSize,
    #[error("message payload size {actual} exceeds max_message_size {limit}")]
    PayloadTooLarge { actual: usize, limit: usize },
    #[error("iceoryx2 error: {0}")]
    Ipc(String),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Iceoryx2Request {
    Hello {
        access_mode: AccessMode,
    },
    SubscribeArmFeedback,
    SubscribeWarnings,
    SubscribeEefFeedback,
    QueryParam {
        request_id: String,
        target: RequestTarget,
        name: String,
    },
    QueryMountedEef {
        request_id: String,
    },
    QueryGravityCoefficients {
        request_id: String,
    },
    QueryCurrentPose {
        request_id: String,
    },
    SetArmState {
        state: ArmState,
    },
    SubmitJointTarget {
        positions: [f64; ARM_DOF],
    },
    SubmitTaskTarget {
        pose: Pose,
    },
    SetEefState {
        state: EefState,
    },
    SubmitE2Command {
        command: SingleEefCommand,
    },
    SubmitG2MitCommand {
        command: SingleEefCommand,
    },
    SubmitG2PvtCommand {
        command: SingleEefCommand,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Iceoryx2Event {
    Connected {
        info: ConnectedRobotInfo,
        connection_mode: AccessMode,
        control_allowed: bool,
    },
    Ack {
        message: String,
    },
    ArmFeedback {
        feedback: SerializableArmFeedback,
    },
    EefFeedback {
        feedback: SerializableEefFeedback,
    },
    Warning {
        warning: WarningEvent,
    },
    RequestOutcome {
        request_id: String,
        outcome: RequestOutcome,
    },
    MountedEef {
        request_id: String,
        mounted_eef: MountedEefType,
    },
    GravityCoefficients {
        request_id: String,
        coefficients: BTreeMap<String, [f64; 6]>,
    },
    CurrentPose {
        request_id: String,
        pose: Pose,
    },
    JointTargetAccepted {
        target: JointTarget,
    },
    Error {
        request_id: Option<String>,
        message: String,
    },
}

struct EventSink<'a> {
    publisher: &'a EventPublisher,
    max_message_size: usize,
}

impl EventSink<'_> {
    fn send(&self, message: &Iceoryx2Event) -> Result<(), Iceoryx2TransportError> {
        let payload = serde_json::to_vec(message)?;
        if payload.len() > self.max_message_size {
            return Err(Iceoryx2TransportError::PayloadTooLarge {
                actual: payload.len(),
                limit: self.max_message_size,
            });
        }

        let sample = self
            .publisher
            .loan_slice_uninit(payload.len())
            .map_err(map_iceoryx_error)?;
        let sample = sample.write_from_slice(&payload);
        sample.send().map_err(map_iceoryx_error)?;
        Ok(())
    }
}

#[async_trait(?Send)]
trait TransportClient: Send + Sync {
    fn info(&self) -> ConnectedRobotInfo;
    fn subscribe_arm_feedback(&self) -> broadcast::Receiver<ArmJointFeedback>;
    fn subscribe_warnings(&self) -> broadcast::Receiver<WarningEvent>;
    fn subscribe_eef_feedback(&self) -> Option<broadcast::Receiver<SingleEefFeedback>>;
    async fn query_param(
        &self,
        target: RequestTarget,
        name: &str,
    ) -> Result<RequestOutcome, ClientError>;
    async fn query_mounted_eef(&self) -> Result<MountedEefType, ClientError>;
    async fn query_gravity_coefficients(
        &self,
    ) -> Result<BTreeMap<String, [f64; 6]>, ClientError>;
    fn query_current_pose(&self) -> Result<Pose, ClientError>;
    async fn set_arm_state(&self, state: ArmState) -> Result<(), ClientError>;
    fn submit_joint_target(&self, positions: [f64; ARM_DOF]) -> Result<JointTarget, ClientError>;
    fn submit_task_target(&self, pose: &Pose) -> Result<JointTarget, ClientError>;
    fn set_eef_state(&self, state: EefState) -> Result<(), ClientError>;
    async fn submit_e2_command(&self, command: &SingleEefCommand) -> Result<(), ClientError>;
    async fn submit_g2_mit_command(&self, command: &SingleEefCommand) -> Result<(), ClientError>;
    async fn submit_g2_pvt_command(&self, command: &SingleEefCommand) -> Result<(), ClientError>;
    async fn shutdown_gracefully(&self) -> Result<(), ClientError>;
}

#[async_trait(?Send)]
impl TransportClient for AirbotPlayClient {
    fn info(&self) -> ConnectedRobotInfo {
        self.info().clone()
    }

    fn subscribe_arm_feedback(&self) -> broadcast::Receiver<ArmJointFeedback> {
        AirbotPlayClient::subscribe_arm_feedback(self)
    }

    fn subscribe_warnings(&self) -> broadcast::Receiver<WarningEvent> {
        AirbotPlayClient::subscribe_warnings(self)
    }

    fn subscribe_eef_feedback(&self) -> Option<broadcast::Receiver<SingleEefFeedback>> {
        self.subscribe_e2_feedback()
            .or_else(|| self.subscribe_g2_feedback())
    }

    async fn query_param(
        &self,
        target: RequestTarget,
        name: &str,
    ) -> Result<RequestOutcome, ClientError> {
        AirbotPlayClient::query_param(self, target, name).await
    }

    async fn query_mounted_eef(&self) -> Result<MountedEefType, ClientError> {
        AirbotPlayClient::query_mounted_eef(self).await
    }

    async fn query_gravity_coefficients(
        &self,
    ) -> Result<BTreeMap<String, [f64; 6]>, ClientError> {
        AirbotPlayClient::query_gravity_coefficients(self).await
    }

    fn query_current_pose(&self) -> Result<Pose, ClientError> {
        AirbotPlayClient::query_current_pose(self)
    }

    async fn set_arm_state(&self, state: ArmState) -> Result<(), ClientError> {
        AirbotPlayClient::set_arm_state(self, state).await
    }

    fn submit_joint_target(&self, positions: [f64; ARM_DOF]) -> Result<JointTarget, ClientError> {
        AirbotPlayClient::submit_joint_target(self, positions)
    }

    fn submit_task_target(&self, pose: &Pose) -> Result<JointTarget, ClientError> {
        AirbotPlayClient::submit_task_target(self, pose)
    }

    fn set_eef_state(&self, state: EefState) -> Result<(), ClientError> {
        AirbotPlayClient::set_eef_state(self, state)
    }

    async fn submit_e2_command(&self, command: &SingleEefCommand) -> Result<(), ClientError> {
        AirbotPlayClient::submit_e2_command(self, command)
            .await
            .map(|_| ())
    }

    async fn submit_g2_mit_command(&self, command: &SingleEefCommand) -> Result<(), ClientError> {
        AirbotPlayClient::submit_g2_mit_command(self, command)
            .await
            .map(|_| ())
    }

    async fn submit_g2_pvt_command(&self, command: &SingleEefCommand) -> Result<(), ClientError> {
        AirbotPlayClient::submit_g2_pvt_command(self, command)
            .await
            .map(|_| ())
    }

    async fn shutdown_gracefully(&self) -> Result<(), ClientError> {
        AirbotPlayClient::shutdown_gracefully(self).await
    }
}

pub async fn run_iceoryx2_transport(
    config: Iceoryx2TransportConfig,
) -> Result<(), Iceoryx2TransportError> {
    let client = Arc::new(if config.allow_control {
        AirbotPlayClient::connect_control_with_backends(
            config.interface.clone(),
            config.can_backend,
            config.model_backend,
        )
        .await?
    } else {
        AirbotPlayClient::connect_readonly_with_backends(
            config.interface.clone(),
            config.can_backend,
            config.model_backend,
        )
        .await?
    });

    run_iceoryx2_transport_with_client_and_shutdown(config, client, shutdown_signal()).await
}

async fn run_iceoryx2_transport_with_client_and_shutdown<C, S>(
    config: Iceoryx2TransportConfig,
    client: Arc<C>,
    shutdown: S,
) -> Result<(), Iceoryx2TransportError>
where
    C: TransportClient + 'static,
    S: Future<Output = Result<(), std::io::Error>>,
{
    validate_config(&config)?;

    let node = NodeBuilder::new()
        .signal_handling_mode(SignalHandlingMode::Disabled)
        .create::<ipc::Service>()
        .map_err(map_iceoryx_error)?;

    let (event_publisher, request_subscriber) = open_ports(&node, &config)?;
    let event_sink = EventSink {
        publisher: &event_publisher,
        max_message_size: config.max_message_size,
    };

    let mut connection_mode = AccessMode::Readonly;
    let mut arm_rx: Option<broadcast::Receiver<ArmJointFeedback>> = None;
    let mut warning_rx: Option<broadcast::Receiver<WarningEvent>> = None;
    let mut eef_rx: Option<broadcast::Receiver<SingleEefFeedback>> = None;
    let mut poll = interval(config.poll_interval);
    poll.set_missed_tick_behavior(MissedTickBehavior::Skip);

    let shutdown = shutdown;
    tokio::pin!(shutdown);

    let run_result = async {
        loop {
            tokio::select! {
                shutdown_result = &mut shutdown => {
                    shutdown_result?;
                    break;
                }
                _ = poll.tick() => {
                    for payload in drain_request_payloads(&request_subscriber)? {
                        let request = match serde_json::from_slice::<Iceoryx2Request>(&payload) {
                            Ok(request) => request,
                            Err(error) => {
                                event_sink.send(&Iceoryx2Event::Error {
                                    request_id: None,
                                    message: format!("invalid request payload: {error}"),
                                })?;
                                continue;
                            }
                        };

                        let request_id = extract_request_id(&request).map(str::to_owned);
                        if let Err(error) = process_request(
                            &event_sink,
                            &client,
                            &mut connection_mode,
                            config.allow_control,
                            &mut arm_rx,
                            &mut warning_rx,
                            &mut eef_rx,
                            request,
                        ).await {
                            event_sink.send(&Iceoryx2Event::Error {
                                request_id,
                                message: error.to_string(),
                            })?;
                        }
                    }

                    drain_broadcast_receiver(&event_sink, &mut arm_rx, |feedback| {
                        Iceoryx2Event::ArmFeedback {
                            feedback: feedback.into(),
                        }
                    })?;
                    drain_broadcast_receiver(&event_sink, &mut warning_rx, |warning| {
                        Iceoryx2Event::Warning { warning }
                    })?;
                    drain_broadcast_receiver(&event_sink, &mut eef_rx, |feedback| {
                        Iceoryx2Event::EefFeedback {
                            feedback: feedback.into(),
                        }
                    })?;
                }
            }
        }

        Ok::<(), Iceoryx2TransportError>(())
    }
    .await;

    let shutdown_result = client.shutdown_gracefully().await;
    run_result?;
    shutdown_result?;
    Ok(())
}

fn validate_config(config: &Iceoryx2TransportConfig) -> Result<(), Iceoryx2TransportError> {
    if config.service_root.trim().is_empty() {
        return Err(Iceoryx2TransportError::InvalidServiceRoot);
    }
    if config.poll_interval.is_zero() {
        return Err(Iceoryx2TransportError::InvalidPollInterval);
    }
    if config.max_message_size == 0 {
        return Err(Iceoryx2TransportError::InvalidMaxMessageSize);
    }
    Ok(())
}

fn open_ports(
    node: &Node<ipc::Service>,
    config: &Iceoryx2TransportConfig,
) -> Result<(EventPublisher, RequestSubscriber), Iceoryx2TransportError> {
    let event_service_name: ServiceName = config
        .event_service_name()
        .as_str()
        .try_into()
        .map_err(map_iceoryx_error)?;
    let event_service = node
        .service_builder(&event_service_name)
        .publish_subscribe::<[u8]>()
        .open_or_create()
        .map_err(map_iceoryx_error)?;
    let event_publisher = event_service
        .publisher_builder()
        .initial_max_slice_len(config.max_message_size)
        .create()
        .map_err(map_iceoryx_error)?;

    let request_service_name: ServiceName = config
        .request_service_name()
        .as_str()
        .try_into()
        .map_err(map_iceoryx_error)?;
    let request_service = node
        .service_builder(&request_service_name)
        .publish_subscribe::<[u8]>()
        .open_or_create()
        .map_err(map_iceoryx_error)?;
    let request_subscriber = request_service
        .subscriber_builder()
        .create()
        .map_err(map_iceoryx_error)?;

    Ok((event_publisher, request_subscriber))
}

fn drain_request_payloads(
    subscriber: &RequestSubscriber,
) -> Result<Vec<Vec<u8>>, Iceoryx2TransportError> {
    let mut payloads = Vec::new();

    loop {
        match subscriber.receive().map_err(map_iceoryx_error)? {
            Some(sample) => payloads.push(sample.payload().to_vec()),
            None => return Ok(payloads),
        }
    }
}

async fn process_request<C>(
    event_sink: &EventSink<'_>,
    client: &Arc<C>,
    connection_mode: &mut AccessMode,
    allow_control: bool,
    arm_rx: &mut Option<broadcast::Receiver<ArmJointFeedback>>,
    warning_rx: &mut Option<broadcast::Receiver<WarningEvent>>,
    eef_rx: &mut Option<broadcast::Receiver<SingleEefFeedback>>,
    request: Iceoryx2Request,
) -> Result<(), Iceoryx2TransportError>
where
    C: TransportClient + 'static,
{
    match request {
        Iceoryx2Request::Hello { access_mode } => {
            *connection_mode = if allow_control {
                access_mode
            } else {
                AccessMode::Readonly
            };
            event_sink.send(&Iceoryx2Event::Connected {
                info: client.info(),
                connection_mode: *connection_mode,
                control_allowed: allow_control,
            })?;
        }
        Iceoryx2Request::SubscribeArmFeedback => {
            *arm_rx = Some(client.subscribe_arm_feedback());
            event_sink.send(&Iceoryx2Event::Ack {
                message: "subscribed to arm feedback".to_owned(),
            })?;
        }
        Iceoryx2Request::SubscribeWarnings => {
            *warning_rx = Some(client.subscribe_warnings());
            event_sink.send(&Iceoryx2Event::Ack {
                message: "subscribed to warnings".to_owned(),
            })?;
        }
        Iceoryx2Request::SubscribeEefFeedback => {
            *eef_rx = client.subscribe_eef_feedback();
            event_sink.send(&Iceoryx2Event::Ack {
                message: "subscribed to end-effector feedback".to_owned(),
            })?;
        }
        Iceoryx2Request::QueryParam {
            request_id,
            target,
            name,
        } => {
            let outcome = client.query_param(target, &name).await?;
            event_sink.send(&Iceoryx2Event::RequestOutcome {
                request_id,
                outcome,
            })?;
        }
        Iceoryx2Request::QueryMountedEef { request_id } => {
            let mounted_eef = client.query_mounted_eef().await?;
            event_sink.send(&Iceoryx2Event::MountedEef {
                request_id,
                mounted_eef,
            })?;
        }
        Iceoryx2Request::QueryGravityCoefficients { request_id } => {
            let coefficients = client.query_gravity_coefficients().await?;
            event_sink.send(&Iceoryx2Event::GravityCoefficients {
                request_id,
                coefficients,
            })?;
        }
        Iceoryx2Request::QueryCurrentPose { request_id } => {
            let pose = client.query_current_pose()?;
            event_sink.send(&Iceoryx2Event::CurrentPose { request_id, pose })?;
        }
        Iceoryx2Request::SetArmState { state } => {
            require_control(*connection_mode)?;
            client.set_arm_state(state).await?;
            event_sink.send(&Iceoryx2Event::Ack {
                message: format!("arm state set to {state:?}"),
            })?;
        }
        Iceoryx2Request::SubmitJointTarget { positions } => {
            require_control(*connection_mode)?;
            let target = client.submit_joint_target(positions)?;
            event_sink.send(&Iceoryx2Event::JointTargetAccepted { target })?;
        }
        Iceoryx2Request::SubmitTaskTarget { pose } => {
            require_control(*connection_mode)?;
            let target = client.submit_task_target(&pose)?;
            event_sink.send(&Iceoryx2Event::JointTargetAccepted { target })?;
        }
        Iceoryx2Request::SetEefState { state } => {
            require_control(*connection_mode)?;
            client.set_eef_state(state)?;
            event_sink.send(&Iceoryx2Event::Ack {
                message: "end-effector state updated".to_owned(),
            })?;
        }
        Iceoryx2Request::SubmitE2Command { command } => {
            require_control(*connection_mode)?;
            client.submit_e2_command(&command).await?;
            event_sink.send(&Iceoryx2Event::Ack {
                message: "submitted E2 command".to_owned(),
            })?;
        }
        Iceoryx2Request::SubmitG2MitCommand { command } => {
            require_control(*connection_mode)?;
            client.submit_g2_mit_command(&command).await?;
            event_sink.send(&Iceoryx2Event::Ack {
                message: "submitted G2 MIT command".to_owned(),
            })?;
        }
        Iceoryx2Request::SubmitG2PvtCommand { command } => {
            require_control(*connection_mode)?;
            client.submit_g2_pvt_command(&command).await?;
            event_sink.send(&Iceoryx2Event::Ack {
                message: "submitted G2 PVT command".to_owned(),
            })?;
        }
    }

    Ok(())
}

fn drain_broadcast_receiver<T, F>(
    event_sink: &EventSink<'_>,
    receiver: &mut Option<broadcast::Receiver<T>>,
    mut build_event: F,
) -> Result<(), Iceoryx2TransportError>
where
    T: Clone,
    F: FnMut(T) -> Iceoryx2Event,
{
    let Some(active_receiver) = receiver.as_mut() else {
        return Ok(());
    };

    loop {
        match active_receiver.try_recv() {
            Ok(value) => event_sink.send(&build_event(value))?,
            Err(broadcast::error::TryRecvError::Empty) => return Ok(()),
            Err(broadcast::error::TryRecvError::Lagged(_)) => continue,
            Err(broadcast::error::TryRecvError::Closed) => {
                *receiver = None;
                return Ok(());
            }
        }
    }
}

fn extract_request_id(request: &Iceoryx2Request) -> Option<&str> {
    match request {
        Iceoryx2Request::QueryParam { request_id, .. }
        | Iceoryx2Request::QueryMountedEef { request_id }
        | Iceoryx2Request::QueryGravityCoefficients { request_id }
        | Iceoryx2Request::QueryCurrentPose { request_id } => Some(request_id.as_str()),
        _ => None,
    }
}

fn require_control(mode: AccessMode) -> Result<(), ClientError> {
    match mode {
        AccessMode::Readonly => Err(ClientError::PermissionDenied),
        AccessMode::Control => Ok(()),
    }
}

fn map_iceoryx_error(error: impl std::fmt::Display) -> Iceoryx2TransportError {
    Iceoryx2TransportError::Ipc(error.to_string())
}

async fn shutdown_signal() -> Result<(), std::io::Error> {
    #[cfg(unix)]
    {
        let mut terminate =
            tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
        tokio::select! {
            result = tokio::signal::ctrl_c() => result,
            _ = terminate.recv() => Ok(()),
        }
    }

    #[cfg(not(unix))]
    {
        tokio::signal::ctrl_c().await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::request_service::RequestOutcome;
    use crate::warnings::WarningKind;
    use std::error::Error;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Mutex, OnceLock};
    use tokio::sync::oneshot;
    use tokio::time::{sleep, timeout};

    struct TestPorts {
        _node: Node<ipc::Service>,
        request_publisher: RequestPublisher,
        event_subscriber: EventSubscriber,
    }

    type RequestPublisher = iceoryx2::port::publisher::Publisher<ipc::Service, [u8], ()>;
    type EventSubscriber = iceoryx2::port::subscriber::Subscriber<ipc::Service, [u8], ()>;

    struct FakeClient {
        info: ConnectedRobotInfo,
        arm_feedback_tx: broadcast::Sender<ArmJointFeedback>,
        warning_tx: broadcast::Sender<WarningEvent>,
        pose: Mutex<Pose>,
        gravity_coefficients: Mutex<BTreeMap<String, [f64; 6]>>,
        request_outcome: Mutex<RequestOutcome>,
        state_calls: Mutex<Vec<ArmState>>,
        joint_targets: Mutex<Vec<[f64; ARM_DOF]>>,
        shutdown_called: AtomicBool,
    }

    impl FakeClient {
        fn new() -> Self {
            let (arm_feedback_tx, _) = broadcast::channel(16);
            let (warning_tx, _) = broadcast::channel(16);
            let mut gravity_coefficients = BTreeMap::new();
            gravity_coefficients.insert("G2".to_owned(), [0.6, 0.6, 0.6, 1.3, 1.18, 1.5]);
            Self {
                info: ConnectedRobotInfo {
                    interface: "can0".to_owned(),
                    access_mode: AccessMode::Control,
                    mounted_eef: MountedEefType::G2,
                    model_backend: ModelBackendKind::PlayAnalytical,
                    gravity_coefficients: [0.6, 0.6, 0.6, 1.3, 1.18, 1.5],
                },
                arm_feedback_tx,
                warning_tx,
                pose: Mutex::new(
                    Pose::from_slice(&[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
                        .expect("pose should be valid"),
                ),
                gravity_coefficients: Mutex::new(gravity_coefficients),
                request_outcome: Mutex::new(RequestOutcome::default()),
                state_calls: Mutex::new(Vec::new()),
                joint_targets: Mutex::new(Vec::new()),
                shutdown_called: AtomicBool::new(false),
            }
        }

        fn emit_arm_feedback(&self, feedback: ArmJointFeedback) {
            let _ = self.arm_feedback_tx.send(feedback);
        }

        fn emit_warning(&self, warning: WarningEvent) {
            let _ = self.warning_tx.send(warning);
        }
    }

    #[async_trait(?Send)]
    impl TransportClient for FakeClient {
        fn info(&self) -> ConnectedRobotInfo {
            self.info.clone()
        }

        fn subscribe_arm_feedback(&self) -> broadcast::Receiver<ArmJointFeedback> {
            self.arm_feedback_tx.subscribe()
        }

        fn subscribe_warnings(&self) -> broadcast::Receiver<WarningEvent> {
            self.warning_tx.subscribe()
        }

        fn subscribe_eef_feedback(&self) -> Option<broadcast::Receiver<SingleEefFeedback>> {
            None
        }

        async fn query_param(
            &self,
            _target: RequestTarget,
            _name: &str,
        ) -> Result<RequestOutcome, ClientError> {
            Ok(self
                .request_outcome
                .lock()
                .expect("request outcome lock poisoned")
                .clone())
        }

        async fn query_mounted_eef(&self) -> Result<MountedEefType, ClientError> {
            Ok(self.info.mounted_eef.clone())
        }

        async fn query_gravity_coefficients(
            &self,
        ) -> Result<BTreeMap<String, [f64; 6]>, ClientError> {
            Ok(self
                .gravity_coefficients
                .lock()
                .expect("gravity coefficients lock poisoned")
                .clone())
        }

        fn query_current_pose(&self) -> Result<Pose, ClientError> {
            Ok(self.pose.lock().expect("pose lock poisoned").clone())
        }

        async fn set_arm_state(&self, state: ArmState) -> Result<(), ClientError> {
            self.state_calls
                .lock()
                .expect("state calls lock poisoned")
                .push(state);
            Ok(())
        }

        fn submit_joint_target(&self, positions: [f64; ARM_DOF]) -> Result<JointTarget, ClientError> {
            self.joint_targets
                .lock()
                .expect("joint targets lock poisoned")
                .push(positions);
            Ok(JointTarget::new(positions))
        }

        fn submit_task_target(&self, _pose: &Pose) -> Result<JointTarget, ClientError> {
            Ok(JointTarget::new([0.0; ARM_DOF]))
        }

        fn set_eef_state(&self, _state: EefState) -> Result<(), ClientError> {
            Ok(())
        }

        async fn submit_e2_command(
            &self,
            _command: &SingleEefCommand,
        ) -> Result<(), ClientError> {
            Ok(())
        }

        async fn submit_g2_mit_command(
            &self,
            _command: &SingleEefCommand,
        ) -> Result<(), ClientError> {
            Ok(())
        }

        async fn submit_g2_pvt_command(
            &self,
            _command: &SingleEefCommand,
        ) -> Result<(), ClientError> {
            Ok(())
        }

        async fn shutdown_gracefully(&self) -> Result<(), ClientError> {
            self.shutdown_called.store(true, Ordering::SeqCst);
            Ok(())
        }
    }

    #[tokio::test(flavor = "current_thread")]
    async fn hello_and_feedback_stream_publish_events() -> Result<(), Box<dyn Error>> {
        let local = tokio::task::LocalSet::new();
        local
            .run_until(async {
                let _guard = test_guard();
                let service_root = unique_root("airbot_transport");
                let ports = create_test_ports(&service_root, 4096)?;
                let client = Arc::new(FakeClient::new());
                let (shutdown_tx, shutdown_rx) = oneshot::channel();

                let task = tokio::task::spawn_local(run_iceoryx2_transport_with_client_and_shutdown(
                    Iceoryx2TransportConfig {
                        service_root: service_root.clone(),
                        interface: "can0".to_owned(),
                        allow_control: true,
                        can_backend: CanWorkerBackend::AsyncFd,
                        model_backend: ModelBackendKind::PlayAnalytical,
                        poll_interval: Duration::from_millis(10),
                        max_message_size: 4096,
                    },
                    Arc::clone(&client),
                    async move {
                        shutdown_rx.await.map_err(|_| {
                            std::io::Error::new(std::io::ErrorKind::BrokenPipe, "shutdown dropped")
                        })
                    },
                ));

                sleep(Duration::from_millis(50)).await;
                send_request(
                    &ports.request_publisher,
                    &Iceoryx2Request::Hello {
                        access_mode: AccessMode::Control,
                    },
                )?;
                let connected = receive_event(&ports.event_subscriber).await?;
                match connected {
                    Iceoryx2Event::Connected {
                        connection_mode,
                        control_allowed,
                        ..
                    } => {
                        assert_eq!(connection_mode, AccessMode::Control);
                        assert!(control_allowed);
                    }
                    other => panic!("unexpected event: {other:?}"),
                }

                send_request(&ports.request_publisher, &Iceoryx2Request::SubscribeArmFeedback)?;
                let ack = receive_event(&ports.event_subscriber).await?;
                assert_eq!(
                    ack,
                    Iceoryx2Event::Ack {
                        message: "subscribed to arm feedback".to_owned()
                    }
                );

                client.emit_arm_feedback(ArmJointFeedback {
                    positions: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                    velocities: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                    torques: [0.6, 0.5, 0.4, 0.3, 0.2, 0.1],
                    valid: true,
                    timestamp_millis: 42,
                });

                let feedback_event = receive_event(&ports.event_subscriber).await?;
                match feedback_event {
                    Iceoryx2Event::ArmFeedback { feedback } => {
                        assert_eq!(feedback.positions[0], 1.0);
                        assert_eq!(feedback.velocities[5], 0.6);
                    }
                    other => panic!("unexpected event: {other:?}"),
                }

                shutdown_tx.send(()).expect("shutdown should succeed");
                task.await??;
                assert!(client.shutdown_called.load(Ordering::SeqCst));
                Ok::<(), Box<dyn Error>>(())
            })
            .await
    }

    #[tokio::test(flavor = "current_thread")]
    async fn control_requests_are_forwarded_to_client() -> Result<(), Box<dyn Error>> {
        let local = tokio::task::LocalSet::new();
        local
            .run_until(async {
                let _guard = test_guard();
                let service_root = unique_root("airbot_control");
                let ports = create_test_ports(&service_root, 4096)?;
                let client = Arc::new(FakeClient::new());
                let (shutdown_tx, shutdown_rx) = oneshot::channel();

                let task = tokio::task::spawn_local(run_iceoryx2_transport_with_client_and_shutdown(
                    Iceoryx2TransportConfig {
                        service_root: service_root.clone(),
                        interface: "can0".to_owned(),
                        allow_control: true,
                        can_backend: CanWorkerBackend::AsyncFd,
                        model_backend: ModelBackendKind::PlayAnalytical,
                        poll_interval: Duration::from_millis(10),
                        max_message_size: 4096,
                    },
                    Arc::clone(&client),
                    async move {
                        shutdown_rx.await.map_err(|_| {
                            std::io::Error::new(std::io::ErrorKind::BrokenPipe, "shutdown dropped")
                        })
                    },
                ));

                sleep(Duration::from_millis(50)).await;
                send_request(
                    &ports.request_publisher,
                    &Iceoryx2Request::Hello {
                        access_mode: AccessMode::Control,
                    },
                )?;
                let _connected = receive_event(&ports.event_subscriber).await?;

                send_request(
                    &ports.request_publisher,
                    &Iceoryx2Request::SetArmState {
                        state: ArmState::CommandFollowing,
                    },
                )?;
                let ack = receive_event(&ports.event_subscriber).await?;
                assert_eq!(
                    ack,
                    Iceoryx2Event::Ack {
                        message: "arm state set to CommandFollowing".to_owned()
                    }
                );

                send_request(
                    &ports.request_publisher,
                    &Iceoryx2Request::SubmitJointTarget {
                        positions: [1.0, 1.1, 1.2, 1.3, 1.4, 1.5],
                    },
                )?;
                let accepted = receive_event(&ports.event_subscriber).await?;
                match accepted {
                    Iceoryx2Event::JointTargetAccepted { target } => {
                        assert_eq!(target.positions, [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]);
                    }
                    other => panic!("unexpected event: {other:?}"),
                }

                let state_calls = client.state_calls.lock().expect("state calls lock poisoned");
                assert!(state_calls.contains(&ArmState::CommandFollowing));
                drop(state_calls);

                let targets = client
                    .joint_targets
                    .lock()
                    .expect("joint targets lock poisoned");
                assert_eq!(targets.last().copied(), Some([1.0, 1.1, 1.2, 1.3, 1.4, 1.5]));
                drop(targets);

                shutdown_tx.send(()).expect("shutdown should succeed");
                task.await??;
                Ok::<(), Box<dyn Error>>(())
            })
            .await
    }

    #[tokio::test(flavor = "current_thread")]
    async fn query_requests_publish_responses() -> Result<(), Box<dyn Error>> {
        let local = tokio::task::LocalSet::new();
        local
            .run_until(async {
                let _guard = test_guard();
                let service_root = unique_root("airbot_query");
                let ports = create_test_ports(&service_root, 4096)?;
                let client = Arc::new(FakeClient::new());
                let (shutdown_tx, shutdown_rx) = oneshot::channel();

                let task = tokio::task::spawn_local(run_iceoryx2_transport_with_client_and_shutdown(
                    Iceoryx2TransportConfig {
                        service_root: service_root.clone(),
                        interface: "can0".to_owned(),
                        allow_control: false,
                        can_backend: CanWorkerBackend::AsyncFd,
                        model_backend: ModelBackendKind::PlayAnalytical,
                        poll_interval: Duration::from_millis(10),
                        max_message_size: 4096,
                    },
                    Arc::clone(&client),
                    async move {
                        shutdown_rx.await.map_err(|_| {
                            std::io::Error::new(std::io::ErrorKind::BrokenPipe, "shutdown dropped")
                        })
                    },
                ));

                sleep(Duration::from_millis(50)).await;
                send_request(
                    &ports.request_publisher,
                    &Iceoryx2Request::Hello {
                        access_mode: AccessMode::Control,
                    },
                )?;
                let connected = receive_event(&ports.event_subscriber).await?;
                match connected {
                    Iceoryx2Event::Connected {
                        connection_mode,
                        control_allowed,
                        ..
                    } => {
                        assert_eq!(connection_mode, AccessMode::Readonly);
                        assert!(!control_allowed);
                    }
                    other => panic!("unexpected event: {other:?}"),
                }

                send_request(&ports.request_publisher, &Iceoryx2Request::SubscribeWarnings)?;
                let _ack = receive_event(&ports.event_subscriber).await?;
                client.emit_warning(
                    WarningEvent::new(WarningKind::MalformedFrame, "test warning")
                        .with_interface("can0"),
                );
                let warning = receive_event(&ports.event_subscriber).await?;
                match warning {
                    Iceoryx2Event::Warning { warning } => {
                        assert_eq!(warning.message, "test warning");
                    }
                    other => panic!("unexpected event: {other:?}"),
                }

                send_request(
                    &ports.request_publisher,
                    &Iceoryx2Request::QueryCurrentPose {
                        request_id: "pose-1".to_owned(),
                    },
                )?;
                let pose_event = receive_event(&ports.event_subscriber).await?;
                match pose_event {
                    Iceoryx2Event::CurrentPose { request_id, pose } => {
                        assert_eq!(request_id, "pose-1");
                        assert_eq!(pose.translation[0], 0.1);
                    }
                    other => panic!("unexpected event: {other:?}"),
                }

                send_request(
                    &ports.request_publisher,
                    &Iceoryx2Request::QueryGravityCoefficients {
                        request_id: "grav-1".to_owned(),
                    },
                )?;
                let gravity_event = receive_event(&ports.event_subscriber).await?;
                match gravity_event {
                    Iceoryx2Event::GravityCoefficients {
                        request_id,
                        coefficients,
                    } => {
                        assert_eq!(request_id, "grav-1");
                        assert!(coefficients.contains_key("G2"));
                    }
                    other => panic!("unexpected event: {other:?}"),
                }

                shutdown_tx.send(()).expect("shutdown should succeed");
                task.await??;
                Ok::<(), Box<dyn Error>>(())
            })
            .await
    }

    fn create_test_ports(
        service_root: &str,
        max_message_size: usize,
    ) -> Result<TestPorts, Box<dyn Error>> {
        let node = NodeBuilder::new()
            .signal_handling_mode(SignalHandlingMode::Disabled)
            .create::<ipc::Service>()?;

        let request_service_name: ServiceName = format!("{service_root}/requests").as_str().try_into()?;
        let request_service = node
            .service_builder(&request_service_name)
            .publish_subscribe::<[u8]>()
            .open_or_create()?;
        let request_publisher = request_service
            .publisher_builder()
            .initial_max_slice_len(max_message_size)
            .create()?;

        let event_service_name: ServiceName = format!("{service_root}/events").as_str().try_into()?;
        let event_service = node
            .service_builder(&event_service_name)
            .publish_subscribe::<[u8]>()
            .open_or_create()?;
        let event_subscriber = event_service.subscriber_builder().create()?;

        Ok(TestPorts {
            _node: node,
            request_publisher,
            event_subscriber,
        })
    }

    fn send_request(
        publisher: &RequestPublisher,
        request: &Iceoryx2Request,
    ) -> Result<(), Box<dyn Error>> {
        let payload = serde_json::to_vec(request)?;
        let sample = publisher.loan_slice_uninit(payload.len())?;
        let sample = sample.write_from_slice(&payload);
        sample.send()?;
        Ok(())
    }

    async fn receive_event(subscriber: &EventSubscriber) -> Result<Iceoryx2Event, Box<dyn Error>> {
        let event = timeout(Duration::from_secs(2), async {
            loop {
                if let Some(sample) = subscriber.receive().map_err(map_iceoryx_error)? {
                    let event = serde_json::from_slice::<Iceoryx2Event>(sample.payload())?;
                    break Ok::<Iceoryx2Event, Iceoryx2TransportError>(event);
                }
                sleep(Duration::from_millis(10)).await;
            }
        })
        .await??;
        Ok(event)
    }

    fn test_guard() -> std::sync::MutexGuard<'static, ()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(())).lock().unwrap()
    }

    fn unique_root(prefix: &str) -> String {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos();
        format!("{prefix}/{nanos}")
    }
}
