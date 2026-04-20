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

/// Where a `JointTarget` originated. The control loop uses this to decide
/// whether to apply the per-tick joint-delta safety clamp: IK-derived
/// targets can branch-switch or otherwise produce large discontinuities
/// from the live feedback and so are clamped, while direct joint targets
/// (supplied by upstream callers as joint positions, encoded as MIT
/// commands downstream) are trusted as-is so intentional aggressive
/// motion is not throttled by the control layer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CommandSource {
    /// Joint positions supplied directly by an upstream caller, e.g.
    /// teleop leader-follower mapping or scripted joint moves. Sent
    /// downstream as MIT-mode CAN frames.
    DirectJoint,
    /// Joint positions produced by inverse kinematics from a cartesian
    /// pose target submitted via `submit_task_target`.
    InverseKinematics,
}

#[derive(Clone, Debug)]
pub struct CommandSlotSnapshot {
    pub target: JointTarget,
    pub source: CommandSource,
    pub updated_at: Instant,
    pub age: Duration,
}

#[derive(Debug, Default)]
pub struct CommandSlot {
    inner: Mutex<Option<(JointTarget, CommandSource, Instant)>>,
}

impl CommandSlot {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set(&self, target: JointTarget, source: CommandSource) {
        *self.inner.lock().expect("command slot mutex poisoned") =
            Some((target, source, Instant::now()));
    }

    pub fn clear(&self) {
        self.inner
            .lock()
            .expect("command slot mutex poisoned")
            .take();
    }

    pub fn latest(&self) -> Option<CommandSlotSnapshot> {
        let guard = self.inner.lock().expect("command slot mutex poisoned");
        guard
            .as_ref()
            .map(|(target, source, updated_at)| CommandSlotSnapshot {
                target: target.clone(),
                source: *source,
                updated_at: *updated_at,
                age: updated_at.elapsed(),
            })
    }
}

#[cfg(test)]
mod tests {
    use super::{CommandSlot, CommandSource, JointTarget};

    #[test]
    fn command_slot_replaces_latest_target() {
        let slot = CommandSlot::new();
        slot.set(
            JointTarget::new([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            CommandSource::DirectJoint,
        );
        slot.set(
            JointTarget::new([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            CommandSource::InverseKinematics,
        );

        let snapshot = slot.latest().expect("expected latest target");
        assert_eq!(snapshot.target.positions[0], 2.0);
        assert_eq!(snapshot.source, CommandSource::InverseKinematics);
    }
}
