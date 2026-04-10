use airbot_play_rust::can::worker::CanWorkerBackend;
use airbot_play_rust::model::ModelBackendKind;
use airbot_play_rust::transport::iceoryx2::{Iceoryx2TransportConfig, run_iceoryx2_transport};
use clap::Parser;
use std::time::Duration;

const DEFAULT_POLL_MS: u64 = 10;
const DEFAULT_MAX_MESSAGE_SIZE: usize = 16 * 1024;

#[derive(Debug, Parser)]
#[command(name = "airbot-play-iceoryx2")]
#[command(about = "AIRBOT Play iceoryx2 adapter")]
struct Args {
    #[arg(long, default_value = "can0")]
    interface: String,
    #[arg(long)]
    service_root: Option<String>,
    #[arg(long, default_value_t = true)]
    allow_control: bool,
    #[arg(long, value_enum, default_value_t = CanWorkerBackend::AsyncFd)]
    can_backend: CanWorkerBackend,
    #[arg(long, value_enum, default_value_t = ModelBackendKind::PlayAnalytical)]
    model_backend: ModelBackendKind,
    #[arg(long, default_value_t = DEFAULT_POLL_MS)]
    poll_ms: u64,
    #[arg(long, default_value_t = DEFAULT_MAX_MESSAGE_SIZE)]
    max_message_size: usize,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt().init();

    let args = Args::parse();
    run_iceoryx2_transport(build_config(args)).await?;
    Ok(())
}

fn build_config(args: Args) -> Iceoryx2TransportConfig {
    let service_root = args
        .service_root
        .unwrap_or_else(|| format!("airbot-play/{}", args.interface));
    Iceoryx2TransportConfig {
        service_root,
        interface: args.interface,
        allow_control: args.allow_control,
        can_backend: args.can_backend,
        model_backend: args.model_backend,
        poll_interval: Duration::from_millis(args.poll_ms),
        max_message_size: args.max_message_size,
    }
}

#[cfg(test)]
mod tests {
    use super::{Args, build_config};
    use airbot_play_rust::model::ModelBackendKind;
    use clap::Parser;

    #[test]
    fn model_backend_flag_parses() {
        let args =
            Args::try_parse_from(["airbot-play-iceoryx2", "--model-backend", "play-analytical"])
                .expect("CLI should accept the analytical backend");
        assert_eq!(args.model_backend, ModelBackendKind::PlayAnalytical);
    }

    #[test]
    fn service_root_defaults_from_interface() {
        let args = Args::try_parse_from(["airbot-play-iceoryx2", "--interface", "can7"])
            .expect("CLI should parse the interface");
        let config = build_config(args);
        assert_eq!(config.service_root, "airbot-play/can7");
    }
}
