use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub translation: [f64; 3],
    pub rotation_xyzw: [f64; 4],
}

impl Pose {
    pub fn from_components(translation: [f64; 3], rotation_xyzw: [f64; 4]) -> Result<Self, ModelError> {
        let norm = rotation_xyzw.iter().map(|value| value * value).sum::<f64>().sqrt();
        if norm <= f64::EPSILON {
            return Err(ModelError::InvalidQuaternion);
        }

        Ok(Self {
            translation,
            rotation_xyzw: [
                rotation_xyzw[0] / norm,
                rotation_xyzw[1] / norm,
                rotation_xyzw[2] / norm,
                rotation_xyzw[3] / norm,
            ],
        })
    }

    pub fn from_slice(values: &[f64]) -> Result<Self, ModelError> {
        if values.len() != 7 {
            return Err(ModelError::InvalidPoseVectorLength(values.len()));
        }

        Self::from_components(
            [values[0], values[1], values[2]],
            [values[3], values[4], values[5], values[6]],
        )
    }

    pub fn as_vec(&self) -> Vec<f64> {
        vec![
            self.translation[0],
            self.translation[1],
            self.translation[2],
            self.rotation_xyzw[0],
            self.rotation_xyzw[1],
            self.rotation_xyzw[2],
            self.rotation_xyzw[3],
        ]
    }
}

#[derive(Debug, Error)]
pub enum ModelError {
    #[error("invalid pose vector length {0}, expected 7")]
    InvalidPoseVectorLength(usize),
    #[error("pose quaternion must be non-zero")]
    InvalidQuaternion,
    #[error("vector length mismatch for {label}: expected {expected}, got {got}")]
    VectorLengthMismatch {
        label: &'static str,
        expected: usize,
        got: usize,
    },
    #[error("missing bundled URDF for `{0}`")]
    MissingBundledUrdf(String),
    #[error("unsupported model backend `{0}`")]
    UnsupportedBackend(String),
    #[error("model backend failure: {0}")]
    Backend(String),
}

pub trait KinematicsDynamicsBackend: Send + Sync {
    fn backend_name(&self) -> &'static str;
    fn dof(&self) -> usize;
    fn forward_kinematics(&self, joints: &[f64]) -> Result<Pose, ModelError>;
    fn inverse_kinematics(&self, target: &Pose, seed: Option<&[f64]>) -> Result<Vec<f64>, ModelError>;
    fn forward_dynamics(
        &self,
        joints: &[f64],
        vel: &[f64],
        torque: &[f64],
    ) -> Result<Vec<f64>, ModelError>;
    fn inverse_dynamics(
        &self,
        joints: &[f64],
        vel: &[f64],
        acc: &[f64],
    ) -> Result<Vec<f64>, ModelError>;
}
