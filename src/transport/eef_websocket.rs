use crate::can::worker::CanWorkerBackend;
use crate::client::{AccessMode, ClientError, ConnectedRobotInfo};
use crate::eef::{EefClient, EefRuntimeProfile, EefState, SingleEefCommand, SingleEefFeedback};
use crate::model::ModelBackendKind;
use crate::warnings::WarningEvent;
use futures_util::{Sink, SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
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
pub struct EefWebSocketServerConfig {
    pub bind_addr: String,
    pub interface: String,
    pub allow_control: bool,
    pub can_backend: CanWorkerBackend,
    pub model_backend: ModelBackendKind,
    pub eef_profile: EefRuntimeProfile,
}

#[derive(Debug, Error)]
pub enum EefWebSocketServerError {
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
    SubscribeWarnings,
    SubscribeEefFeedback,
    SetEefState { state: EefState },
    SubmitE2Command { command: SingleEefCommand },
    SubmitG2MitCommand { command: SingleEefCommand },
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
    EefFeedback {
        feedback: SingleEefFeedback,
    },
    Warning {
        warning: WarningEvent,
    },
    Error {
        request_id: Option<String>,
        message: String,
    },
}

pub async fn run_eef_websocket_server(
    config: EefWebSocketServerConfig,
) -> Result<(), EefWebSocketServerError> {
    let client = Arc::new(if config.allow_control {
        EefClient::connect_control_with_backends(
            config.interface.clone(),
            config.can_backend,
            config.model_backend,
            config.eef_profile,
        )
        .await?
    } else {
        EefClient::connect_readonly_with_backends(
            config.interface.clone(),
            config.can_backend,
            config.model_backend,
            config.eef_profile,
        )
        .await?
    });

    let listener = TcpListener::bind(&config.bind_addr).await?;
    info!(
        bind_addr = %config.bind_addr,
        interface = %config.interface,
        allow_control = config.allow_control,
        can_backend = ?config.can_backend,
        model_backend = ?config.model_backend,
        eef_profile = config.eef_profile.label(),
        "AIRBOT Play EEF websocket server listening"
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
                        warn!(peer = %addr, error = %err, "EEF websocket connection closed with error");
                    }
                });
            }
            shutdown_result = &mut shutdown => {
                shutdown_result?;
                info!("shutdown signal received, stopping EEF websocket server");
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
    client: Arc<EefClient>,
    allow_control: bool,
) -> Result<(), EefWebSocketServerError> {
    let mut ws = accept_async(stream).await?;
    let mut connection_mode = AccessMode::Readonly;
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
                            ClientMessage::SubscribeWarnings => {
                                warning_rx = Some(client.subscribe_warnings());
                                send_json(&mut ws, &ServerMessage::Ack {
                                    message: "subscribed to warnings".to_owned(),
                                }).await?;
                            }
                            ClientMessage::SubscribeEefFeedback => {
                                eef_rx = client.subscribe_eef_feedback();
                                send_json(&mut ws, &ServerMessage::Ack {
                                    message: "subscribed to end-effector feedback".to_owned(),
                                }).await?;
                            }
                            ClientMessage::SetEefState { state } => {
                                if let Err(err) = require_control(connection_mode) {
                                    send_json(&mut ws, &ServerMessage::Error {
                                        request_id: None,
                                        message: err.to_string(),
                                    }).await?;
                                    continue;
                                }
                                if let Err(err) = client.set_eef_state(state).await {
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
            _ = pending::<()>(), if warning_rx.is_none() && eef_rx.is_none() => {}
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

async fn send_json<S>(sink: &mut S, message: &ServerMessage) -> Result<(), EefWebSocketServerError>
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
