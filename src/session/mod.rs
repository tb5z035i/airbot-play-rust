use crate::can::socketcan_io::{SocketCanIo, SocketCanIoError};
use crate::diag::decode::DiagnosticRouter;
use crate::types::{DecodedFrame, RawCanFrame};
use crate::warning_bus::WarningBus;
use crate::warnings::{WarningEvent, WarningKind};
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};
use thiserror::Error;
use tokio::sync::{broadcast, Mutex as AsyncMutex};
use tokio::task::JoinHandle;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RoutedFrame {
    pub interface: String,
    pub raw_frame: RawCanFrame,
    pub decoded_frames: Vec<DecodedFrame>,
}

#[derive(Debug, Error)]
pub enum SessionRouterError {
    #[error("socket CAN error: {0}")]
    Io(#[from] SocketCanIoError),
    #[error("session router already started")]
    AlreadyStarted,
}

#[derive(Debug, Default)]
pub struct FrameRouter {
    interface: String,
    inner: DiagnosticRouter,
}

impl FrameRouter {
    pub fn new(interface: impl Into<String>) -> Self {
        Self {
            interface: interface.into(),
            inner: DiagnosticRouter::default(),
        }
    }

    pub fn route(&mut self, frame: RawCanFrame) -> (RoutedFrame, Vec<WarningEvent>) {
        let decoded_frames = self.inner.inspect(&frame);
        let routed = RoutedFrame {
            interface: self.interface.clone(),
            raw_frame: frame,
            decoded_frames,
        };

        let mut warnings = Vec::new();
        if routed.decoded_frames.is_empty() {
            warnings.push(
                WarningEvent::new(
                    WarningKind::UnmatchedFrame,
                    format!("no protocol decoder matched raw frame {}", routed.raw_frame),
                )
                .with_interface(self.interface.clone())
                .with_detail("can_id", format!("0x{:03X}", routed.raw_frame.can_id))
                .with_detail("dlc", routed.raw_frame.can_dlc.to_string()),
            );
        }

        (routed, warnings)
    }
}

#[derive(Debug)]
pub struct SessionRouter {
    interface: String,
    io: Arc<SocketCanIo>,
    frame_router: Arc<AsyncMutex<FrameRouter>>,
    frames_tx: broadcast::Sender<RoutedFrame>,
    warning_bus: WarningBus,
    task: Mutex<Option<JoinHandle<()>>>,
}

impl SessionRouter {
    pub fn open(interface: impl Into<String>) -> Result<Self, SessionRouterError> {
        let interface = interface.into();
        let io = Arc::new(SocketCanIo::open(interface.clone())?);
        Ok(Self::from_io(interface, io))
    }

    pub fn from_io(interface: impl Into<String>, io: Arc<SocketCanIo>) -> Self {
        let interface = interface.into();
        let (frames_tx, _) = broadcast::channel(512);
        Self {
            interface: interface.clone(),
            io,
            frame_router: Arc::new(AsyncMutex::new(FrameRouter::new(interface))),
            frames_tx,
            warning_bus: WarningBus::default(),
            task: Mutex::new(None),
        }
    }

    pub fn interface(&self) -> &str {
        &self.interface
    }

    pub fn io(&self) -> &Arc<SocketCanIo> {
        &self.io
    }

    pub fn warning_bus(&self) -> &WarningBus {
        &self.warning_bus
    }

    pub fn subscribe_frames(&self) -> broadcast::Receiver<RoutedFrame> {
        self.frames_tx.subscribe()
    }

    pub fn subscribe_warnings(&self) -> broadcast::Receiver<WarningEvent> {
        self.warning_bus.subscribe()
    }

    pub fn publish_warning(&self, warning: WarningEvent) {
        self.warning_bus.publish(warning);
    }

    pub async fn route_frame(&self, frame: RawCanFrame) -> RoutedFrame {
        let (routed, warnings) = self.frame_router.lock().await.route(frame);
        let _ = self.frames_tx.send(routed.clone());
        for warning in warnings {
            self.warning_bus.publish(warning);
        }
        routed
    }

    pub async fn pump_once(&self) -> Result<RoutedFrame, SessionRouterError> {
        let frame = self.io.recv().await?;
        Ok(self.route_frame(frame).await)
    }

    pub fn start(&self) -> Result<(), SessionRouterError> {
        let mut task_guard = self.task.lock().expect("session task mutex poisoned");
        if task_guard.is_some() {
            return Err(SessionRouterError::AlreadyStarted);
        }

        let io = Arc::clone(&self.io);
        let router = Arc::clone(&self.frame_router);
        let frames_tx = self.frames_tx.clone();
        let warning_bus = self.warning_bus.clone();
        let interface = self.interface.clone();

        *task_guard = Some(tokio::spawn(async move {
            loop {
                match io.recv().await {
                    Ok(frame) => {
                        let (routed, warnings) = router.lock().await.route(frame);
                        let _ = frames_tx.send(routed);
                        for warning in warnings {
                            warning_bus.publish(warning);
                        }
                    }
                    Err(err) => {
                        warning_bus.publish(
                            WarningEvent::new(
                                WarningKind::MalformedFrame,
                                format!("session receive loop stopped: {err}"),
                            )
                            .with_interface(interface.clone()),
                        );
                        break;
                    }
                }
            }
        }));

        Ok(())
    }

    pub fn stop(&self) {
        if let Some(task) = self.task.lock().expect("session task mutex poisoned").take() {
            task.abort();
        }
    }
}

impl Drop for SessionRouter {
    fn drop(&mut self) {
        if let Some(task) = self.task.get_mut().expect("session task mutex poisoned").take() {
            task.abort();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::FrameRouter;
    use crate::types::{DecodedFrame, RawCanFrame};
    use crate::warnings::WarningKind;

    #[test]
    fn frame_router_decodes_known_protocol_frame() {
        let mut router = FrameRouter::new("can0");
        let (routed, warnings) = router.route(RawCanFrame::new(0x001, &[0x00]).unwrap());

        assert!(warnings.is_empty());
        assert_eq!(routed.interface, "can0");
        assert_eq!(routed.decoded_frames.len(), 1);
        match &routed.decoded_frames[0] {
            DecodedFrame::ParamRequest { name, .. } => assert_eq!(name, "board_id"),
            other => panic!("unexpected routed decode: {other:?}"),
        }
    }

    #[test]
    fn frame_router_warns_for_unmatched_frame() {
        let mut router = FrameRouter::new("can0");
        let (routed, warnings) = router.route(RawCanFrame::new(0x555, &[0xAA, 0xBB]).unwrap());

        assert!(routed.decoded_frames.is_empty());
        assert_eq!(warnings.len(), 1);
        assert_eq!(warnings[0].kind, WarningKind::UnmatchedFrame);
    }
}
