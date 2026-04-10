use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum WarningKind {
    RequestedReplyTimeout,
    RealtimeFeedbackTimeout,
    ControlRateLow,
    ControlTickOverrun,
    StaleCommandReplay,
    InvalidGravityCompensation,
    PartialFrameAssembly,
    UnmatchedFrame,
    MalformedFrame,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WarningEvent {
    pub interface: Option<String>,
    pub instance: Option<String>,
    pub kind: WarningKind,
    pub message: String,
    pub details: BTreeMap<String, String>,
}

impl WarningEvent {
    pub fn new(kind: WarningKind, message: impl Into<String>) -> Self {
        Self {
            interface: None,
            instance: None,
            kind,
            message: message.into(),
            details: BTreeMap::new(),
        }
    }

    pub fn with_interface(mut self, interface: impl Into<String>) -> Self {
        self.interface = Some(interface.into());
        self
    }

    pub fn with_instance(mut self, instance: impl Into<String>) -> Self {
        self.instance = Some(instance.into());
        self
    }

    pub fn with_detail(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.details.insert(key.into(), value.into());
        self
    }
}
