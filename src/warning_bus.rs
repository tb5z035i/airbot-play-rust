use crate::warnings::WarningEvent;
use tokio::sync::broadcast;

#[derive(Clone, Debug)]
pub struct WarningBus {
    sender: broadcast::Sender<WarningEvent>,
}

impl Default for WarningBus {
    fn default() -> Self {
        Self::new(256)
    }
}

impl WarningBus {
    pub fn new(capacity: usize) -> Self {
        let (sender, _) = broadcast::channel(capacity);
        Self { sender }
    }

    pub fn sender(&self) -> broadcast::Sender<WarningEvent> {
        self.sender.clone()
    }

    pub fn subscribe(&self) -> broadcast::Receiver<WarningEvent> {
        self.sender.subscribe()
    }

    pub fn publish(&self, event: WarningEvent) {
        let _ = self.sender.send(event);
    }
}
