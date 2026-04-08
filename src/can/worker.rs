use super::realtime::{configure_sched_fifo, lock_memory, set_current_thread_affinity};
use super::socketcan_io::{SocketCanIo, SocketCanIoError, frame_to_raw, raw_to_can_data_frame};
use crate::types::RawCanFrame;
use socketcan::{ShouldRetry, Socket};
use std::collections::VecDeque;
use std::io::ErrorKind;
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use thiserror::Error;
use tokio::sync::{broadcast, mpsc};
use tracing::warn;

const DEFAULT_FRAME_CHANNEL_CAPACITY: usize = 2048;
const DEFAULT_INTENT_CHANNEL_CAPACITY: usize = 256;
const DEFAULT_RT_READ_TIMEOUT: Duration = Duration::from_millis(1);
const DEFAULT_RT_WRITE_TIMEOUT: Duration = Duration::from_millis(1);

#[derive(Clone, Copy, Debug, Eq, PartialEq, clap::ValueEnum)]
pub enum CanWorkerBackend {
    AsyncFd,
    RealtimeThread,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum CanTxPriority {
    Diagnostic,
    Lifecycle,
    Control,
}

#[derive(Clone, Debug)]
pub struct CanTxIntent {
    pub priority: CanTxPriority,
    pub frames: Vec<RawCanFrame>,
}

impl CanTxIntent {
    pub fn new(priority: CanTxPriority, frames: Vec<RawCanFrame>) -> Self {
        Self { priority, frames }
    }
}

#[derive(Clone, Debug)]
pub struct RealtimeThreadConfig {
    pub fifo_priority: Option<i32>,
    pub lock_memory: bool,
    pub cpu_affinity: Option<usize>,
    pub read_timeout: Duration,
    pub write_timeout: Duration,
}

impl Default for RealtimeThreadConfig {
    fn default() -> Self {
        Self {
            fifo_priority: Some(80),
            lock_memory: false,
            cpu_affinity: None,
            read_timeout: DEFAULT_RT_READ_TIMEOUT,
            write_timeout: DEFAULT_RT_WRITE_TIMEOUT,
        }
    }
}

#[derive(Clone, Debug)]
pub struct CanWorkerConfig {
    pub interface: String,
    pub backend: CanWorkerBackend,
    pub frame_channel_capacity: usize,
    pub intent_channel_capacity: usize,
    pub realtime: RealtimeThreadConfig,
}

impl Default for CanWorkerConfig {
    fn default() -> Self {
        Self {
            interface: "can0".to_owned(),
            backend: CanWorkerBackend::AsyncFd,
            frame_channel_capacity: DEFAULT_FRAME_CHANNEL_CAPACITY,
            intent_channel_capacity: DEFAULT_INTENT_CHANNEL_CAPACITY,
            realtime: RealtimeThreadConfig::default(),
        }
    }
}

#[derive(Debug, Error)]
pub enum CanWorkerError {
    #[error("socket CAN error: {0}")]
    Io(#[from] SocketCanIoError),
    #[error("socket CAN worker channel closed")]
    Closed,
}

#[derive(Debug)]
enum WorkerCommand {
    Enqueue(CanTxIntent),
    Shutdown,
}

#[derive(Debug, Default)]
struct TxQueues {
    control: VecDeque<RawCanFrame>,
    lifecycle: VecDeque<RawCanFrame>,
    diagnostic: VecDeque<RawCanFrame>,
}

impl TxQueues {
    fn push_intent(&mut self, intent: CanTxIntent) {
        let queue = match intent.priority {
            CanTxPriority::Control => &mut self.control,
            CanTxPriority::Lifecycle => &mut self.lifecycle,
            CanTxPriority::Diagnostic => &mut self.diagnostic,
        };
        queue.extend(intent.frames);
    }

    fn pop_next(&mut self) -> Option<RawCanFrame> {
        self.control
            .pop_front()
            .or_else(|| self.lifecycle.pop_front())
            .or_else(|| self.diagnostic.pop_front())
    }

    fn push_front(&mut self, priority: CanTxPriority, frame: RawCanFrame) {
        let queue = match priority {
            CanTxPriority::Control => &mut self.control,
            CanTxPriority::Lifecycle => &mut self.lifecycle,
            CanTxPriority::Diagnostic => &mut self.diagnostic,
        };
        queue.push_front(frame);
    }

    fn is_empty(&self) -> bool {
        self.control.is_empty() && self.lifecycle.is_empty() && self.diagnostic.is_empty()
    }
}

#[derive(Debug)]
pub struct CanWorker {
    interface: String,
    backend: CanWorkerBackend,
    command_tx: mpsc::Sender<WorkerCommand>,
    frames_tx: broadcast::Sender<RawCanFrame>,
}

impl CanWorker {
    pub fn open(config: CanWorkerConfig) -> Result<Arc<Self>, CanWorkerError> {
        let (command_tx, command_rx) = mpsc::channel(config.intent_channel_capacity);
        let (frames_tx, _) = broadcast::channel(config.frame_channel_capacity);

        let worker = Arc::new(Self {
            interface: config.interface.clone(),
            backend: config.backend,
            command_tx,
            frames_tx: frames_tx.clone(),
        });

        match config.backend {
            CanWorkerBackend::AsyncFd => {
                let io = SocketCanIo::open(config.interface)?;
                tokio::spawn(async move {
                    if let Err(err) = run_asyncfd_worker(io, command_rx, frames_tx).await {
                        warn!(error = %err, "async CAN worker stopped");
                    }
                });
            }
            CanWorkerBackend::RealtimeThread => {
                let interface = config.interface.clone();
                let realtime = config.realtime.clone();
                thread::spawn(move || {
                    if let Err(err) =
                        run_realtime_worker(&interface, realtime, command_rx, frames_tx)
                    {
                        warn!(error = %err, interface = %interface, "realtime CAN worker stopped");
                    }
                });
            }
        }

        Ok(worker)
    }

    pub fn interface(&self) -> &str {
        &self.interface
    }

    pub fn backend(&self) -> CanWorkerBackend {
        self.backend
    }

    pub fn subscribe_frames(&self) -> broadcast::Receiver<RawCanFrame> {
        self.frames_tx.subscribe()
    }

    pub async fn send_intent(&self, intent: CanTxIntent) -> Result<(), CanWorkerError> {
        self.command_tx
            .send(WorkerCommand::Enqueue(intent))
            .await
            .map_err(|_| CanWorkerError::Closed)
    }

    pub fn blocking_send_intent(&self, intent: CanTxIntent) -> Result<(), CanWorkerError> {
        self.command_tx
            .blocking_send(WorkerCommand::Enqueue(intent))
            .map_err(|_| CanWorkerError::Closed)
    }

    pub async fn send_frames(
        &self,
        priority: CanTxPriority,
        frames: Vec<RawCanFrame>,
    ) -> Result<(), CanWorkerError> {
        self.send_intent(CanTxIntent::new(priority, frames)).await
    }

    pub fn blocking_send_frames(
        &self,
        priority: CanTxPriority,
        frames: Vec<RawCanFrame>,
    ) -> Result<(), CanWorkerError> {
        self.blocking_send_intent(CanTxIntent::new(priority, frames))
    }

    pub async fn shutdown(&self) {
        let _ = self.command_tx.send(WorkerCommand::Shutdown).await;
    }

    #[cfg(test)]
    pub(crate) fn dummy_for_tests() -> Arc<Self> {
        let (command_tx, mut command_rx) = mpsc::channel(DEFAULT_INTENT_CHANNEL_CAPACITY);
        let (frames_tx, _) = broadcast::channel(DEFAULT_FRAME_CHANNEL_CAPACITY);
        thread::spawn(move || while command_rx.blocking_recv().is_some() {});
        Arc::new(Self {
            interface: "test".to_owned(),
            backend: CanWorkerBackend::AsyncFd,
            command_tx,
            frames_tx,
        })
    }
}

async fn run_asyncfd_worker(
    io: SocketCanIo,
    mut command_rx: mpsc::Receiver<WorkerCommand>,
    frames_tx: broadcast::Sender<RawCanFrame>,
) -> Result<(), CanWorkerError> {
    let mut queues = TxQueues::default();
    let mut shutdown_requested = false;

    loop {
        while let Ok(command) = command_rx.try_recv() {
            apply_command(command, &mut queues, &mut shutdown_requested);
        }

        while let Some(frame) = queues.pop_next() {
            io.send(&frame).await?;
        }

        if shutdown_requested {
            return Ok(());
        }

        tokio::select! {
            maybe_command = command_rx.recv() => {
                let Some(command) = maybe_command else {
                    shutdown_requested = true;
                    continue;
                };
                apply_command(command, &mut queues, &mut shutdown_requested);
                if shutdown_requested && queues.is_empty() {
                    return Ok(());
                }
            }
            recv_result = io.recv() => {
                let frame = recv_result?;
                let _ = frames_tx.send(frame);
            }
        }
    }
}

fn run_realtime_worker(
    interface: &str,
    config: RealtimeThreadConfig,
    mut command_rx: mpsc::Receiver<WorkerCommand>,
    frames_tx: broadcast::Sender<RawCanFrame>,
) -> Result<(), CanWorkerError> {
    if let Some(priority) = config.fifo_priority
        && let Err(err) = configure_sched_fifo(priority)
    {
        warn!(error = %err, priority, "failed to configure SCHED_FIFO");
    }
    if config.lock_memory
        && let Err(err) = lock_memory()
    {
        warn!(error = %err, "failed to lock memory for realtime worker");
    }
    if let Some(core_id) = config.cpu_affinity
        && let Err(err) = set_current_thread_affinity(core_id)
    {
        warn!(error = %err, core_id, "failed to set CAN worker CPU affinity");
    }

    let socket = socketcan::CanSocket::open(interface)
        .map_err(|err| CanWorkerError::Io(SocketCanIoError::Io(err)))?;
    socket
        .set_write_timeout(Some(config.write_timeout))
        .map_err(|err| CanWorkerError::Io(SocketCanIoError::Io(err)))?;

    let mut queues = TxQueues::default();
    let mut shutdown_requested = false;
    loop {
        while let Ok(command) = command_rx.try_recv() {
            apply_command(command, &mut queues, &mut shutdown_requested);
        }

        if let Some((priority, frame)) = pop_next_with_priority(&mut queues) {
            let can_frame = raw_to_can_data_frame(&frame)?;
            match socket.write_frame(&can_frame) {
                Ok(()) => {}
                Err(err) if is_nonfatal_socket_retry(&err) => {
                    queues.push_front(priority, frame);
                }
                Err(err) => return Err(CanWorkerError::Io(SocketCanIoError::Io(err))),
            }
        }

        if shutdown_requested && queues.is_empty() {
            return Ok(());
        }

        match socket.read_frame_timeout(config.read_timeout) {
            Ok(frame) => {
                let raw = frame_to_raw(frame)?;
                let _ = frames_tx.send(raw);
            }
            Err(err) if is_nonfatal_socket_retry(&err) => {}
            Err(err) => return Err(CanWorkerError::Io(SocketCanIoError::Io(err))),
        }
    }
}

fn apply_command(command: WorkerCommand, queues: &mut TxQueues, shutdown_requested: &mut bool) {
    match command {
        WorkerCommand::Enqueue(intent) => {
            queues.push_intent(intent);
        }
        WorkerCommand::Shutdown => {
            *shutdown_requested = true;
        }
    }
}

fn pop_next_with_priority(queues: &mut TxQueues) -> Option<(CanTxPriority, RawCanFrame)> {
    if let Some(frame) = queues.control.pop_front() {
        return Some((CanTxPriority::Control, frame));
    }
    if let Some(frame) = queues.lifecycle.pop_front() {
        return Some((CanTxPriority::Lifecycle, frame));
    }
    queues
        .diagnostic
        .pop_front()
        .map(|frame| (CanTxPriority::Diagnostic, frame))
}

fn is_nonfatal_socket_retry(err: &std::io::Error) -> bool {
    err.should_retry() || err.kind() == ErrorKind::TimedOut
}

#[cfg(test)]
mod tests {
    use super::{
        CanTxIntent, CanTxPriority, TxQueues, WorkerCommand, apply_command,
        is_nonfatal_socket_retry,
    };
    use crate::types::RawCanFrame;
    use std::io::ErrorKind;

    #[test]
    fn tx_queues_prioritize_control_over_other_traffic() {
        let mut queues = TxQueues::default();
        queues.push_intent(CanTxIntent::new(
            CanTxPriority::Diagnostic,
            vec![RawCanFrame::new(0x300, &[0x03]).unwrap()],
        ));
        queues.push_intent(CanTxIntent::new(
            CanTxPriority::Lifecycle,
            vec![RawCanFrame::new(0x200, &[0x02]).unwrap()],
        ));
        queues.push_intent(CanTxIntent::new(
            CanTxPriority::Control,
            vec![RawCanFrame::new(0x100, &[0x01]).unwrap()],
        ));

        assert_eq!(queues.pop_next().unwrap().can_id, 0x100);
        assert_eq!(queues.pop_next().unwrap().can_id, 0x200);
        assert_eq!(queues.pop_next().unwrap().can_id, 0x300);
    }

    #[test]
    fn timed_out_io_is_treated_as_nonfatal_retry() {
        let err = std::io::Error::from(ErrorKind::TimedOut);
        assert!(is_nonfatal_socket_retry(&err));
    }

    #[test]
    fn shutdown_command_marks_worker_for_drain_instead_of_dropping_queue() {
        let mut queues = TxQueues::default();
        let mut shutdown_requested = false;
        let frame = RawCanFrame::new(0x100, &[0x01]).unwrap();
        apply_command(
            WorkerCommand::Enqueue(CanTxIntent::new(CanTxPriority::Lifecycle, vec![frame])),
            &mut queues,
            &mut shutdown_requested,
        );
        apply_command(
            WorkerCommand::Shutdown,
            &mut queues,
            &mut shutdown_requested,
        );

        assert!(shutdown_requested);
        assert!(!queues.is_empty());
    }
}
