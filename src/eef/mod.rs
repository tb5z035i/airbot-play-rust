mod client;
pub mod e2;
pub mod g2;
mod runtime;

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum EefState {
    Disabled,
    Enabled,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SingleEefFeedback {
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
    pub valid: bool,
    pub timestamp_millis: u128,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SingleEefCommand {
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
    pub mit_kp: f64,
    pub mit_kd: f64,
    pub current_threshold: f64,
}

impl Default for SingleEefCommand {
    fn default() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            effort: 0.0,
            mit_kp: 0.0,
            mit_kd: 0.0,
            current_threshold: 0.0,
        }
    }
}

pub use client::EefClient;
pub use e2::{E2, E2Error};
pub use g2::{G2, G2Error};
pub use runtime::{EefRuntime, EefRuntimeError, EefRuntimeProfile, spawn_eef_runtime_task};
