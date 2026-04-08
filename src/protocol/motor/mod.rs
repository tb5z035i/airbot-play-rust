pub mod dm;
pub mod od;

use crate::protocol::ProtocolError;
use crate::types::{DecodedFrame, ParamDefinition, ParamValue, ProtocolNode, RawCanFrame};

pub trait MotorProtocol {
    fn node(&self) -> ProtocolNode;
    fn definitions(&self) -> &[ParamDefinition];
    fn inspect(&mut self, frame: &RawCanFrame) -> Option<DecodedFrame>;

    fn generate_param_get(&self, name: &str) -> Result<Vec<RawCanFrame>, ProtocolError>;
    fn generate_param_set(
        &self,
        name: &str,
        value: &ParamValue,
    ) -> Result<Vec<RawCanFrame>, ProtocolError>;
}
