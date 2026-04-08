use crate::protocol::board::BoardProtocol;
use crate::protocol::board::gpio::{PlayButtonProtocol, PlayLedProtocol};
use crate::protocol::board::play_base::PlayBaseBoardProtocol;
use crate::protocol::board::play_end::PlayEndBoardProtocol;
use crate::protocol::motor::MotorProtocol;
use crate::protocol::motor::dm::DmProtocol;
use crate::protocol::motor::od::OdProtocol;
use crate::types::{DecodedFrame, RawCanFrame};

#[derive(Debug)]
pub struct DiagnosticRouter {
    od: Vec<OdProtocol>,
    dm: Vec<DmProtocol>,
    play_base: PlayBaseBoardProtocol,
    play_end: PlayEndBoardProtocol,
    buttons: Vec<PlayButtonProtocol>,
    leds: Vec<PlayLedProtocol>,
}

impl Default for DiagnosticRouter {
    fn default() -> Self {
        let od = (1_u16..=7).map(OdProtocol::new).collect();
        let dm = (1_u16..=7).map(DmProtocol::new).collect();
        Self {
            od,
            dm,
            play_base: PlayBaseBoardProtocol::new(),
            play_end: PlayEndBoardProtocol::new(),
            buttons: vec![PlayButtonProtocol::new(0x00), PlayButtonProtocol::new(0x08)],
            leds: vec![PlayLedProtocol::new(0x00)],
        }
    }
}

impl DiagnosticRouter {
    pub fn inspect(&mut self, frame: &RawCanFrame) -> Vec<DecodedFrame> {
        let mut decoded = Vec::new();

        for protocol in &mut self.od {
            if let Some(event) = protocol.inspect(frame) {
                decoded.push(event);
            }
        }
        for protocol in &mut self.dm {
            if let Some(event) = protocol.inspect(frame) {
                decoded.push(event);
            }
        }
        if let Some(event) = self.play_base.inspect(frame) {
            decoded.push(event);
        }
        if let Some(event) = self.play_end.inspect(frame) {
            decoded.push(event);
        }
        for protocol in &self.buttons {
            if let Some(event) = protocol.inspect(frame) {
                decoded.push(event);
            }
        }
        for protocol in &self.leds {
            if let Some(event) = protocol.inspect(frame) {
                decoded.push(event);
            }
        }

        decoded
    }
}
