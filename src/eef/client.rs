use super::{
    EefRuntime, EefRuntimeProfile, EefState, SingleEefCommand, SingleEefFeedback,
    spawn_eef_runtime_task,
};
use crate::arm::PlayArm;
use crate::can::router::CanFrameRouter;
use crate::can::worker::{CanTxPriority, CanWorker, CanWorkerBackend, CanWorkerConfig};
use crate::client::{AccessMode, ClientError, ConnectedRobotInfo};
use crate::model::{ModelBackendKind, ModelError, MountedEefType};
use crate::warning_bus::WarningBus;
use crate::warnings::WarningEvent;
use std::sync::{Arc, Mutex};
use tokio::sync::broadcast;
use tokio::task::JoinHandle;

pub struct EefClient {
    info: ConnectedRobotInfo,
    worker: Arc<CanWorker>,
    frame_router: Arc<CanFrameRouter>,
    eef: Arc<EefRuntime>,
    warning_bus: WarningBus,
    runtime_task: Mutex<Option<JoinHandle<()>>>,
}

impl EefClient {
    pub async fn connect_readonly_with_backends(
        interface: impl Into<String>,
        can_backend: CanWorkerBackend,
        model_backend: ModelBackendKind,
        eef_profile: EefRuntimeProfile,
    ) -> Result<Self, ClientError> {
        Self::connect_with_backends(
            interface,
            AccessMode::Readonly,
            can_backend,
            model_backend,
            eef_profile,
        )
        .await
    }

    pub async fn connect_control_with_backends(
        interface: impl Into<String>,
        can_backend: CanWorkerBackend,
        model_backend: ModelBackendKind,
        eef_profile: EefRuntimeProfile,
    ) -> Result<Self, ClientError> {
        Self::connect_with_backends(
            interface,
            AccessMode::Control,
            can_backend,
            model_backend,
            eef_profile,
        )
        .await
    }

    async fn connect_with_backends(
        interface: impl Into<String>,
        access_mode: AccessMode,
        can_backend: CanWorkerBackend,
        model_backend: ModelBackendKind,
        eef_profile: EefRuntimeProfile,
    ) -> Result<Self, ClientError> {
        let interface = interface.into();
        let worker = CanWorker::open(CanWorkerConfig {
            interface: interface.clone(),
            backend: can_backend,
            ..CanWorkerConfig::default()
        })?;
        let bootstrap = PlayArm::bootstrap(worker.as_ref()).await?;
        let mounted_eef = bootstrap.mounted_eef.clone();
        let eef = Arc::new(EefRuntime::new(mounted_eef.clone()));
        eef.validate_profile(eef_profile)?;

        let warning_bus = WarningBus::default();
        let (frame_router, routes) = CanFrameRouter::new(
            Arc::clone(&worker),
            std::iter::empty::<u16>(),
            matches!(&mounted_eef, MountedEefType::E2B | MountedEefType::G2).then_some(7),
        );
        frame_router
            .start()
            .map_err(|err| ClientError::Model(ModelError::Backend(err.to_string())))?;
        let runtime_task =
            spawn_eef_runtime_task(routes.eef_rx, Arc::clone(&eef), Arc::clone(&worker));

        Ok(Self {
            info: ConnectedRobotInfo {
                interface,
                access_mode,
                mounted_eef,
                model_backend,
                gravity_coefficients: bootstrap.gravity_coefficients,
            },
            worker,
            frame_router,
            eef,
            warning_bus,
            runtime_task: Mutex::new(Some(runtime_task)),
        })
    }

    pub fn info(&self) -> &ConnectedRobotInfo {
        &self.info
    }

    pub fn subscribe_warnings(&self) -> broadcast::Receiver<WarningEvent> {
        self.warning_bus.subscribe()
    }

    pub fn subscribe_eef_feedback(&self) -> Option<broadcast::Receiver<SingleEefFeedback>> {
        self.eef.subscribe_feedback()
    }

    pub async fn set_eef_state(&self, state: EefState) -> Result<(), ClientError> {
        self.require_control()?;
        let frames = self.eef.set_state(state)?;
        if !frames.is_empty() {
            self.worker
                .send_frames(CanTxPriority::Lifecycle, frames)
                .await?;
        }
        Ok(())
    }

    pub async fn submit_e2_command(&self, command: &SingleEefCommand) -> Result<(), ClientError> {
        self.require_control()?;
        let frames = self.eef.build_e2_command(command)?;
        if !frames.is_empty() {
            self.worker
                .send_frames(CanTxPriority::Control, frames)
                .await?;
        }
        Ok(())
    }

    pub async fn submit_g2_mit_command(
        &self,
        command: &SingleEefCommand,
    ) -> Result<(), ClientError> {
        self.require_control()?;
        self.eef.submit_g2_mit_target(command)?;
        Ok(())
    }

    pub async fn shutdown_gracefully(&self) -> Result<(), ClientError> {
        self.frame_router.stop();
        if let Some(task) = self
            .runtime_task
            .lock()
            .expect("EEF runtime task lock poisoned")
            .take()
        {
            task.abort();
        }

        let frames = self.eef.shutdown_frames()?;
        if !frames.is_empty() {
            self.worker
                .send_frames(CanTxPriority::Lifecycle, frames)
                .await?;
        }
        self.worker.shutdown().await;
        Ok(())
    }

    pub fn shutdown(&self) {
        self.frame_router.stop();
        if let Some(task) = self
            .runtime_task
            .lock()
            .expect("EEF runtime task lock poisoned")
            .take()
        {
            task.abort();
        }
    }

    fn require_control(&self) -> Result<(), ClientError> {
        match self.info.access_mode {
            AccessMode::Readonly => Err(ClientError::PermissionDenied),
            AccessMode::Control => Ok(()),
        }
    }
}

impl Drop for EefClient {
    fn drop(&mut self) {
        self.frame_router.stop();
        if let Some(task) = self
            .runtime_task
            .get_mut()
            .expect("EEF runtime task lock poisoned")
            .take()
        {
            task.abort();
        }
    }
}
