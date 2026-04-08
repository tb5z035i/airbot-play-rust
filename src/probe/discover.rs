use crate::can::socketcan_io::{SocketCanIo, SocketCanIoError};
use crate::protocol::ProtocolError;
use crate::protocol::board::BoardProtocol;
use crate::protocol::board::play_base::PlayBaseBoardProtocol;
use crate::protocol::board::play_end::PlayEndBoardProtocol;
use crate::protocol::motor::MotorProtocol;
use crate::protocol::motor::dm::DmProtocol;
use crate::protocol::motor::od::OdProtocol;
use crate::request_service::{RequestError, RequestOutcome, RequestService};
use crate::types::{DecodedFrame, DiscoveredInstance, MotorSoftwareVersion, ParamValue};
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MotorProbeProtocol {
    Od,
    Dm,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MotorProbeTarget {
    joint_id: u16,
    protocol: MotorProbeProtocol,
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
            let base_board_software_version =
                request_string(&service, &io, &mut base, "firmware_version").await?;
            let end_board_software_version =
                request_string(&service, &io, &mut end, "firmware_version").await?;
            let motor_software_versions =
                probe_motor_software_versions(&service, &io, mounted_eef.as_deref()).await?;

            warnings.extend(base_board_software_version.1);
            warnings.extend(end_board_software_version.1);
            warnings.extend(motor_software_versions.1);

            let base_board_software_version = base_board_software_version.0;
            let end_board_software_version = end_board_software_version.0;
            let motor_software_versions = motor_software_versions.0;

            let mut metadata = BTreeMap::new();
            if let Some(value) = identified_as.clone() {
                metadata.insert("identified_as".to_owned(), value);
            }
            if let Some(value) = mounted_eef.clone() {
                metadata.insert("mounted_eef".to_owned(), value);
            }
            if let Some(value) = base_board_software_version.clone() {
                metadata.insert("base_board_software_version".to_owned(), value);
            }
            if let Some(value) = end_board_software_version.clone() {
                metadata.insert("end_board_software_version".to_owned(), value);
            }

            result.instances.push(DiscoveredInstance {
                interface,
                identified_as,
                product_sn,
                pcba_sn,
                mounted_eef,
                base_board_software_version,
                end_board_software_version,
                motor_software_versions,
                metadata,
            });
        }

        result.warnings.extend(warnings);
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
    Ok((extract_display_value(&outcome), outcome.warnings))
}

async fn request_motor_string<P: MotorProtocol>(
    service: &RequestService,
    io: &SocketCanIo,
    protocol: &mut P,
    name: &str,
) -> Result<(Option<String>, Vec<WarningEvent>), ProbeError> {
    let frames = protocol.generate_param_get(name)?;
    let outcome = service
        .exchange(io, &frames, |frame| protocol.inspect(frame))
        .await?;
    Ok((extract_display_value(&outcome), outcome.warnings))
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

async fn probe_motor_software_versions(
    service: &RequestService,
    io: &SocketCanIo,
    mounted_eef: Option<&str>,
) -> Result<(Vec<MotorSoftwareVersion>, Vec<WarningEvent>), ProbeError> {
    let mut versions = Vec::new();
    let mut warnings = Vec::new();

    for target in motor_probe_targets(mounted_eef) {
        let (software_version, new_warnings) = match target.protocol {
            MotorProbeProtocol::Od => {
                let mut protocol = OdProtocol::new(target.joint_id);
                request_motor_string(service, io, &mut protocol, "fw_version").await?
            }
            MotorProbeProtocol::Dm => {
                let mut protocol = DmProtocol::new(target.joint_id);
                request_motor_string(service, io, &mut protocol, "fw_version").await?
            }
        };
        warnings.extend(new_warnings);
        versions.push(MotorSoftwareVersion {
            joint_id: target.joint_id,
            software_version,
        });
    }

    Ok((versions, warnings))
}

fn motor_probe_targets(mounted_eef: Option<&str>) -> Vec<MotorProbeTarget> {
    let mut targets = (1_u16..=3_u16)
        .map(|joint_id| MotorProbeTarget {
            joint_id,
            protocol: MotorProbeProtocol::Od,
        })
        .chain((4_u16..=6_u16).map(|joint_id| MotorProbeTarget {
            joint_id,
            protocol: MotorProbeProtocol::Dm,
        }))
        .collect::<Vec<_>>();

    match mounted_eef {
        Some("E2B") => targets.push(MotorProbeTarget {
            joint_id: 7,
            protocol: MotorProbeProtocol::Od,
        }),
        Some("G2") => targets.push(MotorProbeTarget {
            joint_id: 7,
            protocol: MotorProbeProtocol::Dm,
        }),
        _ => {}
    }

    targets
}

fn extract_display_value(outcome: &RequestOutcome) -> Option<String> {
    outcome
        .decoded_frames
        .iter()
        .find_map(|decoded| match decoded {
            DecodedFrame::ParamResponse { values, .. } => {
                values.values().find_map(|value| match value {
                    ParamValue::Empty => None,
                    value => Some(value.to_string()),
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

#[cfg(test)]
mod tests {
    use super::{MotorProbeProtocol, MotorProbeTarget, extract_display_value, motor_probe_targets};
    use crate::request_service::RequestOutcome;
    use crate::types::{DecodedFrame, FrameKind, ParamValue, ProtocolNode, ProtocolNodeKind};
    use std::collections::BTreeMap;

    #[test]
    fn extract_display_value_handles_strings() {
        let mut values = BTreeMap::new();
        values.insert(
            "firmware_version".to_owned(),
            ParamValue::String("1.2.3".to_owned()),
        );
        let outcome = RequestOutcome {
            raw_frames: Vec::new(),
            decoded_frames: vec![DecodedFrame::ParamResponse {
                node: ProtocolNode {
                    kind: ProtocolNodeKind::PlayBaseBoard,
                    id: 0x100,
                },
                kind: FrameKind::GetParamResp,
                values,
            }],
            warnings: Vec::new(),
        };

        assert_eq!(extract_display_value(&outcome).as_deref(), Some("1.2.3"));
    }

    #[test]
    fn extract_display_value_formats_bytes4() {
        let mut values = BTreeMap::new();
        values.insert(
            "firmware_version".to_owned(),
            ParamValue::Bytes4([0x01, 0x02, 0x03, 0x04]),
        );
        let outcome = RequestOutcome {
            raw_frames: Vec::new(),
            decoded_frames: vec![DecodedFrame::ParamResponse {
                node: ProtocolNode {
                    kind: ProtocolNodeKind::PlayBaseBoard,
                    id: 0x100,
                },
                kind: FrameKind::GetParamResp,
                values,
            }],
            warnings: Vec::new(),
        };

        assert_eq!(
            extract_display_value(&outcome).as_deref(),
            Some("01 02 03 04")
        );
    }

    #[test]
    fn motor_probe_targets_include_arm_motors_and_eef_motor_when_present() {
        let targets = motor_probe_targets(Some("G2"));

        assert_eq!(targets.len(), 7);
        assert_eq!(
            targets[0],
            MotorProbeTarget {
                joint_id: 1,
                protocol: MotorProbeProtocol::Od,
            }
        );
        assert_eq!(
            targets[6],
            MotorProbeTarget {
                joint_id: 7,
                protocol: MotorProbeProtocol::Dm,
            }
        );
    }
}
