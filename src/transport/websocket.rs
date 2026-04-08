use crate::arm::{ArmJointFeedback, ArmState, JointTarget};
use crate::can::worker::CanWorkerBackend;
use crate::client::{AccessMode, AirbotPlayClient, ClientError, ConnectedRobotInfo, RequestTarget};
use crate::eef::{EefState, SingleEefCommand, SingleEefFeedback};
use crate::model::{MountedEefType, Pose};
use crate::request_service::RequestOutcome;
use crate::warnings::WarningEvent;
use futures_util::{Sink, SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::future::pending;
use std::sync::Arc;
use thiserror::Error;
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast;
use tokio::task::JoinSet;
use tokio_tungstenite::accept_async;
use tokio_tungstenite::tungstenite::{Error as WsError, Message};
use tracing::{info, warn};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WebSocketServerConfig {
    pub bind_addr: String,
    pub interface: String,
    pub allow_control: bool,
    pub can_backend: CanWorkerBackend,
}

impl Default for WebSocketServerConfig {
    fn default() -> Self {
        Self {
            bind_addr: "127.0.0.1:9002".to_owned(),
            interface: "can0".to_owned(),
            allow_control: true,
            can_backend: CanWorkerBackend::AsyncFd,
        }
    }
}

#[derive(Debug, Error)]
pub enum WebSocketServerError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("websocket error: {0}")]
    WebSocket(#[from] WsError),
    #[error("client error: {0}")]
    Client(#[from] ClientError),
    #[error("json error: {0}")]
    Json(#[from] serde_json::Error),
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ClientMessage {
    Hello { access_mode: AccessMode },
    SubscribeArmFeedback,
    SubscribeWarnings,
    SubscribeEefFeedback,
    QueryParam {
        request_id: String,
        target: RequestTarget,
        name: String,
    },
    QueryMountedEef { request_id: String },
    QueryGravityCoefficients { request_id: String },
    SetArmState { state: ArmState },
    SubmitJointTarget { positions: [f64; 6] },
    SubmitTaskTarget { pose: Pose },
    SetEefState { state: EefState },
    SubmitE2Command { command: SingleEefCommand },
    SubmitG2MitCommand { command: SingleEefCommand },
    SubmitG2PvtCommand { command: SingleEefCommand },
}

#[derive(Debug, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ServerMessage {
    Connected {
        info: ConnectedRobotInfo,
        connection_mode: AccessMode,
        control_allowed: bool,
    },
    Ack {
        message: String,
    },
    ArmFeedback {
        feedback: ArmJointFeedback,
    },
    EefFeedback {
        feedback: SingleEefFeedback,
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
    JointTargetAccepted {
        target: JointTarget,
    },
    Error {
        request_id: Option<String>,
        message: String,
    },
}

pub async fn run_websocket_server(config: WebSocketServerConfig) -> Result<(), WebSocketServerError> {
    let client = Arc::new(if config.allow_control {
        AirbotPlayClient::connect_control_with_backend(config.interface.clone(), config.can_backend).await?
    } else {
        AirbotPlayClient::connect_readonly_with_backend(config.interface.clone(), config.can_backend).await?
    });

    let listener = TcpListener::bind(&config.bind_addr).await?;
    info!(
        bind_addr = %config.bind_addr,
        interface = %config.interface,
        allow_control = config.allow_control,
        can_backend = ?config.can_backend,
        "AIRBOT Play websocket server listening"
    );

    let mut connections = JoinSet::new();
    let shutdown = shutdown_signal();
    tokio::pin!(shutdown);

    loop {
        tokio::select! {
            accept_result = listener.accept() => {
                let (stream, addr) = accept_result?;
                let client = Arc::clone(&client);
                let allow_control = config.allow_control;
                connections.spawn(async move {
                    if let Err(err) = handle_connection(stream, client, allow_control).await {
                        warn!(peer = %addr, error = %err, "websocket connection closed with error");
                    }
                });
            }
            shutdown_result = &mut shutdown => {
                shutdown_result?;
                info!("shutdown signal received, stopping websocket server");
                break;
            }
        }
    }

    connections.abort_all();
    while connections.join_next().await.is_some() {}
    client.shutdown_gracefully().await?;
    Ok(())
}

async fn handle_connection(
    stream: TcpStream,
    client: Arc<AirbotPlayClient>,
    allow_control: bool,
) -> Result<(), WebSocketServerError> {
    let mut ws = accept_async(stream).await?;
    let mut connection_mode = AccessMode::Readonly;
    let mut arm_rx: Option<broadcast::Receiver<ArmJointFeedback>> = None;
    let mut warning_rx: Option<broadcast::Receiver<WarningEvent>> = None;
    let mut eef_rx: Option<broadcast::Receiver<SingleEefFeedback>> = None;

    send_json(
        &mut ws,
        &ServerMessage::Connected {
            info: client.info().clone(),
            connection_mode,
            control_allowed: allow_control,
        },
    )
    .await?;

    loop {
        tokio::select! {
            message = ws.next() => {
                match message {
                    Some(Ok(Message::Text(text))) => {
                        let client_message: ClientMessage = serde_json::from_str(text.as_ref())?;
                        match client_message {
                            ClientMessage::Hello { access_mode } => {
                                connection_mode = if allow_control { access_mode } else { AccessMode::Readonly };
                                send_json(
                                    &mut ws,
                                    &ServerMessage::Ack {
                                        message: format!("connection mode set to {:?}", connection_mode),
                                    },
                                ).await?;
                            }
                            ClientMessage::SubscribeArmFeedback => {
                                arm_rx = Some(client.subscribe_arm_feedback());
                                send_json(&mut ws, &ServerMessage::Ack {
                                    message: "subscribed to arm feedback".to_owned(),
                                }).await?;
                            }
                            ClientMessage::SubscribeWarnings => {
                                warning_rx = Some(client.subscribe_warnings());
                                send_json(&mut ws, &ServerMessage::Ack {
                                    message: "subscribed to warnings".to_owned(),
                                }).await?;
                            }
                            ClientMessage::SubscribeEefFeedback => {
                                eef_rx = client
                                    .subscribe_e2_feedback()
                                    .or_else(|| client.subscribe_g2_feedback());
                                send_json(&mut ws, &ServerMessage::Ack {
                                    message: "subscribed to end-effector feedback".to_owned(),
                                }).await?;
                            }
                            ClientMessage::QueryParam { request_id, target, name } => {
                                match client.query_param(target, &name).await {
                                    Ok(outcome) => {
                                        send_json(&mut ws, &ServerMessage::RequestOutcome { request_id, outcome }).await?;
                                    }
                                    Err(err) => {
                                        send_json(&mut ws, &ServerMessage::Error {
                                            request_id: Some(request_id),
                                            message: err.to_string(),
                                        }).await?;
                                    }
                                }
                            }
                            ClientMessage::QueryMountedEef { request_id } => {
                                match client.query_mounted_eef().await {
                                    Ok(mounted_eef) => {
                                        send_json(&mut ws, &ServerMessage::MountedEef { request_id, mounted_eef }).await?;
                                    }
                                    Err(err) => {
                                        send_json(&mut ws, &ServerMessage::Error {
                                            request_id: Some(request_id),
                                            message: err.to_string(),
                                        }).await?;
                                    }
                                }
                            }
                            ClientMessage::QueryGravityCoefficients { request_id } => {
                                match client.query_gravity_coefficients().await {
                                    Ok(coefficients) => {
                                        send_json(&mut ws, &ServerMessage::GravityCoefficients {
                                            request_id,
                                            coefficients,
                                        }).await?;
                                    }
                                    Err(err) => {
                                        send_json(&mut ws, &ServerMessage::Error {
                                            request_id: Some(request_id),
                                            message: err.to_string(),
                                        }).await?;
                                    }
                                }
                            }
                            ClientMessage::SetArmState { state } => {
                                if let Err(err) = require_control(connection_mode) {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                    continue;
                                }
                                if let Err(err) = client.set_arm_state(state) {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                } else {
                                    send_json(&mut ws, &ServerMessage::Ack {
                                        message: format!("arm state set to {:?}", state),
                                    }).await?;
                                }
                            }
                            ClientMessage::SubmitJointTarget { positions } => {
                                if let Err(err) = require_control(connection_mode) {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                    continue;
                                }
                                match client.submit_joint_target(positions) {
                                    Ok(target) => {
                                        send_json(&mut ws, &ServerMessage::JointTargetAccepted { target }).await?;
                                    }
                                    Err(err) => {
                                        send_json(&mut ws, &ServerMessage::Error {
                                            request_id: None,
                                            message: err.to_string(),
                                        }).await?;
                                    }
                                }
                            }
                            ClientMessage::SubmitTaskTarget { pose } => {
                                if let Err(err) = require_control(connection_mode) {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                    continue;
                                }
                                match client.submit_task_target(&pose) {
                                    Ok(target) => {
                                        send_json(&mut ws, &ServerMessage::JointTargetAccepted { target }).await?;
                                    }
                                    Err(err) => {
                                        send_json(&mut ws, &ServerMessage::Error {
                                            request_id: None,
                                            message: err.to_string(),
                                        }).await?;
                                    }
                                }
                            }
                            ClientMessage::SetEefState { state } => {
                                if let Err(err) = require_control(connection_mode) {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                    continue;
                                }
                                if let Err(err) = client.set_eef_state(state) {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                } else {
                                    send_json(&mut ws, &ServerMessage::Ack {
                                        message: format!("end-effector state set to {:?}", state),
                                    }).await?;
                                }
                            }
                            ClientMessage::SubmitE2Command { command } => {
                                if let Err(err) = require_control(connection_mode) {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                    continue;
                                }
                                if let Err(err) = client.submit_e2_command(&command).await {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                } else {
                                    send_json(&mut ws, &ServerMessage::Ack {
                                        message: "submitted E2 MIT command".to_owned(),
                                    }).await?;
                                }
                            }
                            ClientMessage::SubmitG2MitCommand { command } => {
                                if let Err(err) = require_control(connection_mode) {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                    continue;
                                }
                                if let Err(err) = client.submit_g2_mit_command(&command).await {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                } else {
                                    send_json(&mut ws, &ServerMessage::Ack {
                                        message: "submitted G2 MIT command".to_owned(),
                                    }).await?;
                                }
                            }
                            ClientMessage::SubmitG2PvtCommand { command } => {
                                if let Err(err) = require_control(connection_mode) {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                    continue;
                                }
                                if let Err(err) = client.submit_g2_pvt_command(&command).await {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                } else {
                                    send_json(&mut ws, &ServerMessage::Ack {
                                        message: "submitted G2 PVT command".to_owned(),
                                    }).await?;
                                }
                            }
                        }
                    }
                    Some(Ok(Message::Binary(_))) => {
                        send_json(&mut ws, &ServerMessage::Error {
                            request_id: None,
                            message: "binary websocket messages are not supported".to_owned(),
                        }).await?;
                    }
                    Some(Ok(Message::Close(_))) | None => break,
                    Some(Ok(Message::Ping(payload))) => {
                        ws.send(Message::Pong(payload)).await?;
                    }
                    Some(Ok(_)) => {}
                    Some(Err(err)) => return Err(err.into()),
                }
            }
            arm_feedback = async { arm_rx.as_mut().unwrap().recv().await }, if arm_rx.is_some() => {
                match arm_feedback {
                    Ok(feedback) => send_json(&mut ws, &ServerMessage::ArmFeedback { feedback }).await?,
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(broadcast::error::RecvError::Closed) => arm_rx = None,
                }
            }
            warning = async { warning_rx.as_mut().unwrap().recv().await }, if warning_rx.is_some() => {
                match warning {
                    Ok(warning) => send_json(&mut ws, &ServerMessage::Warning { warning }).await?,
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(broadcast::error::RecvError::Closed) => warning_rx = None,
                }
            }
            eef_feedback = async { eef_rx.as_mut().unwrap().recv().await }, if eef_rx.is_some() => {
                match eef_feedback {
                    Ok(feedback) => send_json(&mut ws, &ServerMessage::EefFeedback { feedback }).await?,
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(broadcast::error::RecvError::Closed) => eef_rx = None,
                }
            }
            _ = pending::<()>(), if arm_rx.is_none() && warning_rx.is_none() && eef_rx.is_none() => {}
        }
    }

    Ok(())
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

async fn send_json<S>(sink: &mut S, message: &ServerMessage) -> Result<(), WebSocketServerError>
where
    S: Sink<Message, Error = WsError> + Unpin,
{
    sink.send(Message::Text(serde_json::to_string(message)?.into()))
        .await?;
    Ok(())
}

fn require_control(mode: AccessMode) -> Result<(), ClientError> {
    match mode {
        AccessMode::Readonly => Err(ClientError::PermissionDenied),
        AccessMode::Control => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::{ClientMessage, ServerMessage};
    use crate::client::{AccessMode, ConnectedRobotInfo};
    use crate::model::MountedEefType;

    #[test]
    fn hello_message_roundtrips() {
        let message = serde_json::from_str::<ClientMessage>(
            r#"{"type":"hello","access_mode":"control"}"#,
        )
        .expect("expected hello message");

        match message {
            ClientMessage::Hello { access_mode } => assert_eq!(access_mode, AccessMode::Control),
            other => panic!("unexpected message: {other:?}"),
        }
    }

    #[test]
    fn connected_message_serializes() {
        let message = ServerMessage::Connected {
            info: ConnectedRobotInfo {
                interface: "can0".to_owned(),
                access_mode: AccessMode::Control,
                mounted_eef: MountedEefType::E2B,
                gravity_coefficients: [0.6, 0.6, 0.6, 1.338, 1.236, 0.893],
            },
            connection_mode: AccessMode::Readonly,
            control_allowed: true,
        };

        let json = serde_json::to_string(&message).expect("expected connected message JSON");
        assert!(json.contains("\"type\":\"connected\""));
        assert!(json.contains("\"control_allowed\":true"));
    }

    #[test]
    fn readonly_mode_rejects_control_operations() {
        let error = super::require_control(AccessMode::Readonly).expect_err("readonly mode should reject control");
        assert!(error.to_string().contains("control permission"));
        assert!(super::require_control(AccessMode::Control).is_ok());
    }
}
