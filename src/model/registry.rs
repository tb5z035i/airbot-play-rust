use std::sync::Arc;

use clap::ValueEnum;
use serde::{Deserialize, Serialize};

use super::backend::{KinematicsDynamicsBackend, ModelError};
use super::pinocchio::PinocchioBackend;
use super::play_analytical::PlayAnalyticalBackend;
use super::urdf::{MountedEefType, bundled_urdf_xml, default_frame_name};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize, ValueEnum)]
#[serde(rename_all = "snake_case")]
pub enum ModelBackendKind {
    Pinocchio,
    #[default]
    PlayAnalytical,
}

#[derive(Debug, Default)]
pub struct ModelRegistry;

impl ModelRegistry {
    pub fn load(
        backend: ModelBackendKind,
        mounted_eef: MountedEefType,
    ) -> Result<Arc<dyn KinematicsDynamicsBackend>, ModelError> {
        match backend {
            ModelBackendKind::Pinocchio => {
                let urdf_xml = bundled_urdf_xml(&mounted_eef)?;
                let backend = PinocchioBackend::load_from_urdf_xml(
                    mounted_eef.clone(),
                    urdf_xml,
                    default_frame_name(&mounted_eef),
                )?;
                Ok(Arc::new(backend))
            }
            ModelBackendKind::PlayAnalytical => {
                let backend = PlayAnalyticalBackend::load(mounted_eef)?;
                Ok(Arc::new(backend))
            }
        }
    }
}
