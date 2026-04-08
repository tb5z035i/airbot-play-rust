use super::backend::ModelError;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum MountedEefType {
    None,
    E2B,
    G2,
    Other(String),
}

impl MountedEefType {
    pub fn from_label(label: &str) -> Self {
        match label.trim().to_ascii_lowercase().as_str() {
            "" | "none" | "na" => Self::None,
            "e2" | "e2b" => Self::E2B,
            "g2" => Self::G2,
            other => Self::Other(other.to_owned()),
        }
    }

    pub fn from_code(code: u32) -> Self {
        match code {
            0x00 => Self::None,
            0x02 => Self::E2B,
            0x03 => Self::G2,
            other => Self::Other(format!("unknown_{other:02X}")),
        }
    }

    pub fn as_label(&self) -> &str {
        match self {
            Self::None => "none",
            Self::E2B => "E2B",
            Self::G2 => "G2",
            Self::Other(label) => label.as_str(),
        }
    }
}

pub const DEFAULT_GRAVITY_COEFFICIENTS_NONE: [f64; 6] = [0.6, 0.6, 0.6, 1.6, 1.248, 1.5];
pub const DEFAULT_GRAVITY_COEFFICIENTS_E2B: [f64; 6] = [0.6, 0.6, 0.6, 1.338, 1.236, 0.893];
pub const DEFAULT_GRAVITY_COEFFICIENTS_G2: [f64; 6] = [0.6, 0.6, 0.6, 1.303, 1.181, 1.5];
pub const DEFAULT_GRAVITY_COEFFICIENTS_OTHER: [f64; 6] = [0.6, 0.6, 0.6, 1.5, 1.5, 1.5];

const PLAY_URDF: &str = include_str!("../../assets/urdf/play.urdf");
const PLAY_E2_URDF: &str = include_str!("../../assets/urdf/play_e2.urdf");
const PLAY_G2_URDF: &str = include_str!("../../assets/urdf/play_g2.urdf");

pub fn gravity_coefficients_for_eef(mounted_eef: &MountedEefType) -> [f64; 6] {
    match mounted_eef {
        MountedEefType::None => DEFAULT_GRAVITY_COEFFICIENTS_NONE,
        MountedEefType::E2B => DEFAULT_GRAVITY_COEFFICIENTS_E2B,
        MountedEefType::G2 => DEFAULT_GRAVITY_COEFFICIENTS_G2,
        MountedEefType::Other(_) => DEFAULT_GRAVITY_COEFFICIENTS_OTHER,
    }
}

pub fn bundled_urdf_xml(mounted_eef: &MountedEefType) -> Result<String, ModelError> {
    match mounted_eef {
        MountedEefType::None => Ok(PLAY_URDF.to_owned()),
        MountedEefType::E2B => Ok(PLAY_E2_URDF.to_owned()),
        MountedEefType::G2 => Ok(PLAY_G2_URDF.to_owned()),
        MountedEefType::Other(label) => Err(ModelError::MissingBundledUrdf(format!(
            "URDF for mounted end effector `{label}`"
        ))),
    }
}

pub fn default_frame_name(_mounted_eef: &MountedEefType) -> &'static str {
    "end_link"
}
