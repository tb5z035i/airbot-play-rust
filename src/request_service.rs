use crate::can::socketcan_io::{SocketCanIo, SocketCanIoError};
use crate::types::{DecodedFrame, RawCanFrame};
use crate::warnings::{WarningEvent, WarningKind};
use std::time::Duration;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum RequestError {
    #[error("socket CAN error: {0}")]
    Io(#[from] SocketCanIoError),
}

#[derive(Debug, Default)]
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
}
