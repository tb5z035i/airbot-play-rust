use std::sync::Arc;

use super::backend::{KinematicsDynamicsBackend, ModelError};
use super::pinocchio::PinocchioBackend;
use super::urdf::{MountedEefType, bundled_urdf_xml, default_frame_name};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ModelBackendKind {
    Pinocchio,
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
        }
    }
}
