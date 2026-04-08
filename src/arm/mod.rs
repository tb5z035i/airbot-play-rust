pub mod command_slot;
pub mod play;

pub use command_slot::{ARM_DOF, CommandSlot, CommandSlotError, JointTarget};
pub use play::{ArmJointFeedback, ArmState, PlayArm, PlayArmError};
