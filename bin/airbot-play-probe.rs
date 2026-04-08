use airbot_play_rust::probe::discover::probe_all;
use airbot_play_rust::types::DiscoveredInstance;
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
    println!("[{}] {}", index, instance.interface);
    println!(
        "    Identity            : {}",
        format_optional(instance.identified_as.as_deref())
    );
    println!(
        "    Product SN          : {}",
        format_optional(instance.product_sn.as_deref())
    );
    println!(
        "    PCBA SN             : {}",
        format_optional(instance.pcba_sn.as_deref())
    );
    println!(
        "    Mounted EEF (board) : {}",
        format_optional(instance.mounted_eef.as_deref())
    );
    println!();
}

fn format_optional(value: Option<&str>) -> &str {
    value.unwrap_or("unknown")
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
