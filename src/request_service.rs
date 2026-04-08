use crate::can::socketcan_io::{SocketCanIo, SocketCanIoError};
use crate::session::{RoutedFrame, SessionRouter};
use crate::types::{DecodedFrame, RawCanFrame};
use crate::warnings::{WarningEvent, WarningKind};
use serde::{Deserialize, Serialize};
use std::time::Duration;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum RequestError {
    #[error("socket CAN error: {0}")]
    Io(#[from] SocketCanIoError),
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RequestOutcome {
    pub raw_frames: Vec<RawCanFrame>,
    pub decoded_frames: Vec<DecodedFrame>,
    pub warnings: Vec<WarningEvent>,
}

#[derive(Debug, Clone, Copy)]
pub struct RequestService {
    timeout: Duration,
}

impl Default for RequestService {
    fn default() -> Self {
        Self {
            timeout: Duration::from_millis(300),
        }
    }
}

impl RequestService {
    pub fn new(timeout: Duration) -> Self {
        Self { timeout }
    }

    pub fn timeout(&self) -> Duration {
        self.timeout
    }

    pub async fn exchange<F>(
        &self,
        io: &SocketCanIo,
        outbound_frames: &[RawCanFrame],
        mut inspect: F,
    ) -> Result<RequestOutcome, RequestError>
    where
        F: FnMut(&RawCanFrame) -> Option<DecodedFrame>,
    {
        let _ = io.drain_pending(256);

        for frame in outbound_frames {
            io.send(frame).await?;
        }

        let mut outcome = RequestOutcome::default();

        let recv_result = tokio::time::timeout(self.timeout, async {
            loop {
                let frame = io.recv().await?;
                let decoded = inspect(&frame);
                outcome.raw_frames.push(frame);
                if let Some(decoded) = decoded {
                    outcome.decoded_frames.push(decoded);
                    return Ok::<(), RequestError>(());
                }
            }
        })
        .await;

        match recv_result {
            Ok(result) => {
                result?;
            }
            Err(_) => {
                outcome.warnings.push(
                    WarningEvent::new(
                        WarningKind::RequestedReplyTimeout,
                        "request timed out before a matching semantic reply was decoded",
                    )
                    .with_interface(io.interface().to_owned())
                    .with_detail("timeout_ms", self.timeout.as_millis().to_string()),
                );
            }
        }

        Ok(outcome)
    }

    pub async fn exchange_via_session<F>(
        &self,
        session: &SessionRouter,
        outbound_frames: &[RawCanFrame],
        mut inspect: F,
    ) -> Result<RequestOutcome, RequestError>
    where
        F: FnMut(&RoutedFrame) -> Option<DecodedFrame>,
    {
        let mut frames_rx = session.subscribe_frames();
        for frame in outbound_frames {
            session.io().send(frame).await?;
        }

        let mut outcome = RequestOutcome::default();
        let recv_result = tokio::time::timeout(self.timeout, async {
            loop {
                match frames_rx.recv().await {
                    Ok(routed) => {
                        if let Some(decoded) = inspect(&routed) {
                            outcome.raw_frames.push(routed.raw_frame.clone());
                            outcome.decoded_frames.push(decoded);
                            return Ok::<(), RequestError>(());
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => return Ok(()),
                }
            }
        })
        .await;

        match recv_result {
            Ok(result) => {
                result?;
            }
            Err(_) => {
                outcome.warnings.push(
                    WarningEvent::new(
                        WarningKind::RequestedReplyTimeout,
                        "request timed out before a matching routed reply was observed",
                    )
                    .with_interface(session.interface().to_owned())
                    .with_detail("timeout_ms", self.timeout.as_millis().to_string()),
                );
            }
        }

        Ok(outcome)
    }
}
