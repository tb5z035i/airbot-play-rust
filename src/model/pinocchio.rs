use std::sync::Mutex;

use cxx::UniquePtr;

use super::backend::{KinematicsDynamicsBackend, ModelError, Pose};
use super::pinocchio_ffi::ffi;
use super::urdf::MountedEefType;

pub struct PinocchioBackend {
    call_lock: Mutex<()>,
    inner: UniquePtr<ffi::PinocchioModel>,
}

// SAFETY: the underlying Pinocchio model is only accessed behind `call_lock`, which
// serializes FFI calls and avoids concurrent mutable access to the native `Data`.
unsafe impl Send for PinocchioBackend {}
unsafe impl Sync for PinocchioBackend {}

impl PinocchioBackend {
    pub fn load_from_urdf_xml(
        _mounted_eef: MountedEefType,
        urdf_xml: impl AsRef<str>,
        frame_name: impl AsRef<str>,
    ) -> Result<Self, ModelError> {
        let inner = ffi::load_model(urdf_xml.as_ref(), frame_name.as_ref())
            .map_err(|err| ModelError::Backend(err.to_string()))?;
        Ok(Self {
            call_lock: Mutex::new(()),
            inner,
        })
    }

    fn inner_ref(&self) -> Result<&ffi::PinocchioModel, ModelError> {
        self.inner
            .as_ref()
            .ok_or_else(|| ModelError::Backend("Pinocchio model handle is null".to_owned()))
    }

    fn validate_len(&self, label: &'static str, values: &[f64]) -> Result<(), ModelError> {
        let expected = self.dof();
        if values.len() != expected {
            return Err(ModelError::VectorLengthMismatch {
                label,
                expected,
                got: values.len(),
            });
        }
        Ok(())
    }
}

impl KinematicsDynamicsBackend for PinocchioBackend {
    fn backend_name(&self) -> &'static str {
        "pinocchio"
    }

    fn dof(&self) -> usize {
        self.inner
            .as_ref()
            .map(|inner| inner.dof())
            .unwrap_or_default()
    }

    fn forward_kinematics(&self, joints: &[f64]) -> Result<Pose, ModelError> {
        self.validate_len("joint vector", joints)?;
        let _guard = self
            .call_lock
            .lock()
            .map_err(|_| ModelError::Backend("Pinocchio call lock poisoned".to_owned()))?;
        let pose = self
            .inner_ref()?
            .forward_kinematics(joints)
            .map_err(|err| ModelError::Backend(err.to_string()))?;
        Pose::from_slice(&pose)
    }

    fn inverse_kinematics(
        &self,
        target: &Pose,
        seed: Option<&[f64]>,
    ) -> Result<Vec<f64>, ModelError> {
        let seed = match seed {
            Some(values) => {
                self.validate_len("IK seed", values)?;
                values
            }
            None => &[],
        };
        let target_pose = target.as_vec();
        let _guard = self
            .call_lock
            .lock()
            .map_err(|_| ModelError::Backend("Pinocchio call lock poisoned".to_owned()))?;
        self.inner_ref()?
            .inverse_kinematics(&target_pose, seed)
            .map_err(|err| ModelError::Backend(err.to_string()))
    }

    fn forward_dynamics(
        &self,
        joints: &[f64],
        vel: &[f64],
        torque: &[f64],
    ) -> Result<Vec<f64>, ModelError> {
        self.validate_len("joint vector", joints)?;
        self.validate_len("velocity vector", vel)?;
        self.validate_len("torque vector", torque)?;
        let _guard = self
            .call_lock
            .lock()
            .map_err(|_| ModelError::Backend("Pinocchio call lock poisoned".to_owned()))?;
        self.inner_ref()?
            .forward_dynamics(joints, vel, torque)
            .map_err(|err| ModelError::Backend(err.to_string()))
    }

    fn inverse_dynamics(
        &self,
        joints: &[f64],
        vel: &[f64],
        acc: &[f64],
    ) -> Result<Vec<f64>, ModelError> {
        self.validate_len("joint vector", joints)?;
        self.validate_len("velocity vector", vel)?;
        self.validate_len("acceleration vector", acc)?;
        let _guard = self
            .call_lock
            .lock()
            .map_err(|_| ModelError::Backend("Pinocchio call lock poisoned".to_owned()))?;
        self.inner_ref()?
            .inverse_dynamics(joints, vel, acc)
            .map_err(|err| ModelError::Backend(err.to_string()))
    }
}
