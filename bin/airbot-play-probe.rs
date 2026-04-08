use airbot_play_rust::probe::discover::probe_all;
use airbot_play_rust::types::{DiscoveredInstance, MotorSoftwareVersion};
use airbot_play_rust::warnings::WarningEvent;
use clap::Parser;
use serde::Serialize;
use std::time::Duration;
use tracing_subscriber::FmtSubscriber;

#[derive(Debug, Parser)]
struct Args {
    #[arg(long, default_value_t = 1000)]
    timeout_ms: u64,
    #[arg(long, conflicts_with = "json_report")]
    json: bool,
    #[arg(
        long = "json-report",
        visible_alias = "program-json",
        conflicts_with = "json"
    )]
    json_report: bool,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let subscriber = FmtSubscriber::builder().with_target(false).finish();
    let _ = tracing::subscriber::set_global_default(subscriber);

    let args = Args::parse();
    let result = probe_all(Duration::from_millis(args.timeout_ms)).await?;

    if args.json_report {
        let output = StructuredProbeOutput {
            schema_version: 1,
            tool: "airbot-play-probe",
            instance_count: result.instances.len(),
            warning_count: result.warnings.len(),
            instances: &result.instances,
            warnings: &result.warnings,
        };
        println!("{}", serde_json::to_string_pretty(&output)?);
    } else if args.json {
        println!("{}", serde_json::to_string_pretty(&result.instances)?);
    } else {
        if result.instances.is_empty() {
            println!("No AIRBOT instances found.");
        } else {
            println!("Found {} AIRBOT instance(s)\n", result.instances.len());

            for (index, instance) in result.instances.iter().enumerate() {
                print_instance(index + 1, instance);
            }
        }

        if !result.warnings.is_empty() {
            println!("Warnings:");
            for warning in result.warnings {
                if let Some(interface) = warning.interface {
                    println!(
                        "  - [{}] {:?}: {}",
                        interface, warning.kind, warning.message
                    );
                } else {
                    println!("  - {:?}: {}", warning.kind, warning.message);
                }
            }
        }
    }

    Ok(())
}

fn print_instance(index: usize, instance: &DiscoveredInstance) {
    println!("{}", format_instance(index, instance));
    println!();
}

fn format_instance(index: usize, instance: &DiscoveredInstance) -> String {
    [
        format!("[{}] {}", index, instance.interface),
        format!(
            "    Product SN          : {}",
            format_optional(instance.product_sn.as_deref())
        ),
        format!(
            "    PCBA SN             : {}",
            format_optional(instance.pcba_sn.as_deref())
        ),
        format!(
            "    Mounted EEF (board) : {}",
            format_optional(instance.mounted_eef.as_deref())
        ),
        format!(
            "    Base Board SW Ver   : {}",
            format_optional(instance.base_board_software_version.as_deref())
        ),
        format!(
            "    End Board SW Ver    : {}",
            format_optional(instance.end_board_software_version.as_deref())
        ),
        format!(
            "    Motor SW Versions   : {}",
            format_motor_software_versions(&instance.motor_software_versions)
        ),
    ]
    .join("\n")
}

fn format_optional(value: Option<&str>) -> &str {
    value.unwrap_or("unknown")
}

fn format_motor_software_versions(motor_versions: &[MotorSoftwareVersion]) -> String {
    if motor_versions.is_empty() {
        return "unknown".to_owned();
    }

    motor_versions
        .iter()
        .map(|motor| {
            format!(
                "J{}={}",
                motor.joint_id,
                format_optional(motor.software_version.as_deref())
            )
        })
        .collect::<Vec<_>>()
        .join(", ")
}

#[derive(Debug, Serialize)]
struct StructuredProbeOutput<'a> {
    schema_version: u32,
    tool: &'static str,
    instance_count: usize,
    warning_count: usize,
    instances: &'a [DiscoveredInstance],
    warnings: &'a [WarningEvent],
}

#[cfg(test)]
mod tests {
    use super::{format_instance, format_motor_software_versions};
    use airbot_play_rust::types::{DiscoveredInstance, MotorSoftwareVersion};
    use std::collections::BTreeMap;

    #[test]
    fn format_instance_omits_identity_and_includes_versions() {
        let instance = DiscoveredInstance {
            interface: "can0".to_owned(),
            identified_as: Some("AIRBOT-PLAY-BASE-001".to_owned()),
            product_sn: Some("P123".to_owned()),
            pcba_sn: Some("C456".to_owned()),
            mounted_eef: Some("G2".to_owned()),
            base_board_software_version: Some("01 02 03 04".to_owned()),
            end_board_software_version: Some("05 06 07 08".to_owned()),
            motor_software_versions: vec![
                MotorSoftwareVersion {
                    joint_id: 1,
                    software_version: Some("1.0.0".to_owned()),
                },
                MotorSoftwareVersion {
                    joint_id: 7,
                    software_version: None,
                },
            ],
            metadata: BTreeMap::new(),
        };

        let output = format_instance(1, &instance);

        assert!(!output.contains("Identity"));
        assert!(!output.contains("AIRBOT-PLAY-BASE-001"));
        assert!(output.contains("Base Board SW Ver"));
        assert!(output.contains("End Board SW Ver"));
        assert!(output.contains("Motor SW Versions"));
        assert!(output.contains("J1=1.0.0"));
        assert!(output.contains("J7=unknown"));
    }

    #[test]
    fn format_motor_software_versions_handles_empty_list() {
        assert_eq!(format_motor_software_versions(&[]), "unknown");
    }
}
