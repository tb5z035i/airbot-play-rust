use super::worker::CanWorker;
use crate::types::RawCanFrame;
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use thiserror::Error;
use tokio::sync::{broadcast, mpsc};
use tokio::task::JoinHandle;

const DEFAULT_ROUTE_CAPACITY: usize = 512;

#[derive(Debug)]
pub struct CanFrameRoutes {
    pub arm_rx: mpsc::Receiver<RawCanFrame>,
    pub motor_rxs: BTreeMap<u16, mpsc::Receiver<RawCanFrame>>,
    pub eef_rx: Option<mpsc::Receiver<RawCanFrame>>,
}

#[derive(Debug, Error)]
pub enum CanFrameRouterError {
    #[error("CAN frame router already started")]
    AlreadyStarted,
}

#[derive(Debug)]
pub struct CanFrameRouter {
    worker: Arc<CanWorker>,
    motor_txs: BTreeMap<u16, mpsc::Sender<RawCanFrame>>,
    arm_tx: mpsc::Sender<RawCanFrame>,
    eef_motor_id: Option<u16>,
    eef_tx: Option<mpsc::Sender<RawCanFrame>>,
    task: Mutex<Option<JoinHandle<()>>>,
}

impl CanFrameRouter {
    pub fn new(
        worker: Arc<CanWorker>,
        motor_ids: impl IntoIterator<Item = u16>,
        eef_motor_id: Option<u16>,
    ) -> (Arc<Self>, CanFrameRoutes) {
        let mut motor_txs = BTreeMap::new();
        let mut motor_rxs = BTreeMap::new();
        for motor_id in motor_ids {
            let (tx, rx) = mpsc::channel(DEFAULT_ROUTE_CAPACITY);
            motor_txs.insert(motor_id, tx);
            motor_rxs.insert(motor_id, rx);
        }

        let (arm_tx, arm_rx) = mpsc::channel(DEFAULT_ROUTE_CAPACITY);
        let (eef_tx, eef_rx) = if eef_motor_id.is_some() {
            let (tx, rx) = mpsc::channel(DEFAULT_ROUTE_CAPACITY);
            (Some(tx), Some(rx))
        } else {
            (None, None)
        };

        let router = Arc::new(Self {
            worker,
            motor_txs,
            arm_tx,
            eef_motor_id,
            eef_tx,
            task: Mutex::new(None),
        });

        (
            router,
            CanFrameRoutes {
                arm_rx,
                motor_rxs,
                eef_rx,
            },
        )
    }

    pub fn start(self: &Arc<Self>) -> Result<(), CanFrameRouterError> {
        let mut task_guard = self.task.lock().expect("CAN router task mutex poisoned");
        if task_guard.is_some() {
            return Err(CanFrameRouterError::AlreadyStarted);
        }

        let mut frames_rx = self.worker.subscribe_frames();
        let motor_txs = self.motor_txs.clone();
        let arm_tx = self.arm_tx.clone();
        let eef_motor_id = self.eef_motor_id;
        let eef_tx = self.eef_tx.clone();

        *task_guard = Some(tokio::spawn(async move {
            loop {
                match frames_rx.recv().await {
                    Ok(frame) => {
                        dispatch_arm_frame(&arm_tx, &frame).await;
                        dispatch_motor_frame(&motor_txs, eef_motor_id, eef_tx.as_ref(), frame).await;
                    }
                    Err(broadcast::error::RecvError::Closed) => break,
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                }
            }
        }));

        Ok(())
    }

    pub fn stop(&self) {
        if let Some(task) = self.task.lock().expect("CAN router task mutex poisoned").take() {
            task.abort();
        }
    }
}

impl Drop for CanFrameRouter {
    fn drop(&mut self) {
        if let Some(task) = self.task.get_mut().expect("CAN router task mutex poisoned").take() {
            task.abort();
        }
    }
}

async fn dispatch_arm_frame(arm_tx: &mpsc::Sender<RawCanFrame>, frame: &RawCanFrame) {
    if is_board_frame(frame.can_id) {
        let _ = arm_tx.send(frame.clone()).await;
    }
}

async fn dispatch_motor_frame(
    motor_txs: &BTreeMap<u16, mpsc::Sender<RawCanFrame>>,
    eef_motor_id: Option<u16>,
    eef_tx: Option<&mpsc::Sender<RawCanFrame>>,
    frame: RawCanFrame,
) {
    if let Some(motor_id) = routed_motor_id(frame.can_id) {
        if let Some(tx) = motor_txs.get(&motor_id) {
            let _ = tx.send(frame).await;
            return;
        }
        if Some(motor_id) == eef_motor_id {
            if let Some(tx) = eef_tx {
                let _ = tx.send(frame).await;
            }
        }
    }
}

fn is_board_frame(can_id: u32) -> bool {
    matches!(can_id, 0x100 | 0x108)
}

fn routed_motor_id(can_id: u32) -> Option<u16> {
    match can_id {
        0x001..=0x007 => Some(can_id as u16),
        0x101..=0x107 => Some((can_id - 0x100) as u16),
        0x701..=0x707 => Some((can_id - 0x700) as u16),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::{CanFrameRouter, dispatch_motor_frame, routed_motor_id};
    use crate::can::worker::CanWorker;
    use crate::types::RawCanFrame;
    use std::collections::BTreeMap;
    use tokio::sync::mpsc;

    #[test]
    fn routed_motor_id_matches_expected_reply_ranges() {
        assert_eq!(routed_motor_id(0x001), Some(1));
        assert_eq!(routed_motor_id(0x104), Some(4));
        assert_eq!(routed_motor_id(0x706), Some(6));
        assert_eq!(routed_motor_id(0x100), None);
        assert_eq!(routed_motor_id(0x108), None);
    }

    #[tokio::test]
    async fn router_builds_motor_and_board_routes() {
        let worker = CanWorker::dummy_for_tests();
        let (_router, routes) = CanFrameRouter::new(worker, [1, 2, 3], Some(7));
        assert_eq!(routes.motor_rxs.len(), 3);
        assert!(routes.eef_rx.is_some());
        let _ = routes.arm_rx;
    }

    #[tokio::test]
    async fn dispatch_motor_frame_targets_only_matching_motor() {
        let (tx4, mut rx4) = mpsc::channel(1);
        let (tx5, mut rx5) = mpsc::channel(1);
        let mut motor_txs = BTreeMap::new();
        motor_txs.insert(4, tx4);
        motor_txs.insert(5, tx5);

        let frame = RawCanFrame::new(0x704, &[0x04, 0x80, 0x08, 0x7F, 0xD7, 0xFF, 0x1E, 0x1C]).unwrap();
        dispatch_motor_frame(&motor_txs, None, None, frame.clone()).await;

        assert_eq!(rx4.recv().await.unwrap(), frame);
        assert!(rx5.try_recv().is_err());
    }
}
