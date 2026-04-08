use crate::can::socketcan_io::{SocketCanIo, SocketCanIoError};
use crate::protocol::ProtocolError;
use crate::protocol::board::BoardProtocol;
use crate::protocol::board::play_base::PlayBaseBoardProtocol;
use crate::protocol::board::play_end::PlayEndBoardProtocol;
use crate::request_service::{RequestError, RequestOutcome, RequestService};
use crate::types::{DecodedFrame, DiscoveredInstance, ParamValue};
use crate::warnings::WarningEvent;
use std::collections::BTreeMap;
use std::time::Duration;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ProbeError {
    #[error("I/O error: {0}")]
    IoStd(#[from] std::io::Error),
    #[error("socket CAN error: {0}")]
    Io(#[from] SocketCanIoError),
    #[error("request error: {0}")]
    Request(#[from] RequestError),
    #[error("protocol error: {0}")]
    Protocol(#[from] ProtocolError),
}

#[derive(Debug, Default)]
pub struct ProbeResult {
    pub instances: Vec<DiscoveredInstance>,
    pub warnings: Vec<WarningEvent>,
}

pub async fn probe_all(timeout: Duration) -> Result<ProbeResult, ProbeError> {
    let interfaces = SocketCanIo::candidate_interfaces()?;
    let mut result = ProbeResult::default();

    for interface in interfaces {
        let io = SocketCanIo::open(interface.clone())?;
        let service = RequestService::new(timeout);
        let mut base = PlayBaseBoardProtocol::new();
        let mut end = PlayEndBoardProtocol::new();

        let identify = request_string(&service, &io, &mut base, "pcba_name").await?;
        let product_sn = request_string(&service, &io, &mut base, "product_sn").await?;
        let pcba_sn = request_string(&service, &io, &mut base, "pcba_sn").await?;
        let eef_type = request_u32(&service, &io, &mut end, "eef_type").await?;

        let mut warnings = Vec::new();
        warnings.extend(identify.1);
        warnings.extend(product_sn.1);
        warnings.extend(pcba_sn.1);
        warnings.extend(eef_type.1);
        result.warnings.extend(warnings.clone());

        let identified_as = identify.0;
        let product_sn = product_sn.0;
        let pcba_sn = pcba_sn.0;
        let mounted_eef = eef_type.0.map(map_eef_type);

        let looks_like_airbot = identified_as
            .as_deref()
            .map(|value| {
                let normalized = value.to_ascii_lowercase();
                normalized.contains("arm-") || normalized.contains("airbot")
            })
            .unwrap_or(false);

        if looks_like_airbot || product_sn.is_some() || mounted_eef.is_some() {
            let mut metadata = BTreeMap::new();
            if let Some(value) = identified_as.clone() {
                metadata.insert("identified_as".to_owned(), value);
            }
            if let Some(value) = mounted_eef.clone() {
                metadata.insert("mounted_eef".to_owned(), value);
            }

            result.instances.push(DiscoveredInstance {
                interface,
                identified_as,
                product_sn,
                pcba_sn,
                mounted_eef,
                metadata,
            });
        }
    }

    Ok(result)
}

async fn request_string<P: BoardProtocol>(
    service: &RequestService,
    io: &SocketCanIo,
    protocol: &mut P,
    name: &str,
) -> Result<(Option<String>, Vec<WarningEvent>), ProbeError> {
    let frames = protocol.generate_param_get(name)?;
    let outcome = service
        .exchange(io, &frames, |frame| protocol.inspect(frame))
        .await?;
    Ok((extract_string(&outcome), outcome.warnings))
}

async fn request_u32<P: BoardProtocol>(
    service: &RequestService,
    io: &SocketCanIo,
    protocol: &mut P,
    name: &str,
) -> Result<(Option<u32>, Vec<WarningEvent>), ProbeError> {
    let frames = protocol.generate_param_get(name)?;
    let outcome = service
        .exchange(io, &frames, |frame| protocol.inspect(frame))
        .await?;
    Ok((extract_u32(&outcome), outcome.warnings))
}

fn extract_string(outcome: &RequestOutcome) -> Option<String> {
    outcome
        .decoded_frames
        .iter()
        .find_map(|decoded| match decoded {
            DecodedFrame::ParamResponse { values, .. } => {
                values.values().find_map(|value| match value {
                    ParamValue::String(text) => Some(text.clone()),
                    _ => None,
                })
            }
            _ => None,
        })
}

fn extract_u32(outcome: &RequestOutcome) -> Option<u32> {
    outcome
        .decoded_frames
        .iter()
        .find_map(|decoded| match decoded {
            DecodedFrame::ParamResponse { values, .. } => {
                values.values().find_map(|value| match value {
                    ParamValue::U32(value) => Some(*value),
                    _ => None,
                })
            }
            _ => None,
        })
}

fn map_eef_type(value: u32) -> String {
    match value {
        0x00 => "none".to_owned(),
        0x02 => "E2B".to_owned(),
        0x03 => "G2".to_owned(),
        _ => format!("unknown_{value:02X}"),
    }
}
