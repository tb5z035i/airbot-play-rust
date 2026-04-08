use serde::{Deserialize, Serialize};
use std::sync::Mutex;
use std::time::{Duration, Instant};
use thiserror::Error;

pub const ARM_DOF: usize = 6;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointTarget {
    pub positions: [f64; ARM_DOF],
}

impl JointTarget {
    pub fn new(positions: [f64; ARM_DOF]) -> Self {
        Self { positions }
    }

    pub fn from_slice(values: &[f64]) -> Result<Self, CommandSlotError> {
        if values.len() != ARM_DOF {
            return Err(CommandSlotError::InvalidJointTargetLength(values.len()));
        }

        let mut positions = [0.0; ARM_DOF];
        positions.copy_from_slice(values);
        Ok(Self { positions })
    }
}

#[derive(Debug, Error)]
pub enum CommandSlotError {
    #[error("invalid joint target length {0}, expected 6")]
    InvalidJointTargetLength(usize),
}

#[derive(Clone, Debug)]
pub struct CommandSlotSnapshot {
    pub target: JointTarget,
    pub updated_at: Instant,
    pub age: Duration,
}

#[derive(Debug, Default)]
pub struct CommandSlot {
    inner: Mutex<Option<(JointTarget, Instant)>>,
}

impl CommandSlot {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set(&self, target: JointTarget) {
        *self.inner.lock().expect("command slot mutex poisoned") = Some((target, Instant::now()));
    }

    pub fn clear(&self) {
        self.inner.lock().expect("command slot mutex poisoned").take();
    }

    pub fn latest(&self) -> Option<CommandSlotSnapshot> {
        let guard = self.inner.lock().expect("command slot mutex poisoned");
        guard.as_ref().map(|(target, updated_at)| CommandSlotSnapshot {
            target: target.clone(),
            updated_at: *updated_at,
            age: updated_at.elapsed(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::{CommandSlot, JointTarget};

    #[test]
    fn command_slot_replaces_latest_target() {
        let slot = CommandSlot::new();
        slot.set(JointTarget::new([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
        slot.set(JointTarget::new([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]));

        let snapshot = slot.latest().expect("expected latest target");
        assert_eq!(snapshot.target.positions[0], 2.0);
    }
}
