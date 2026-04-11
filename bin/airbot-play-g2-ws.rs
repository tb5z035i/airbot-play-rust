use airbot_play_rust::can::worker::CanWorkerBackend;
use airbot_play_rust::eef::EefRuntimeProfile;
use airbot_play_rust::model::ModelBackendKind;
use airbot_play_rust::transport::eef_websocket::{
    EefWebSocketServerConfig, run_eef_websocket_server,
};
use clap::Parser;

#[derive(Debug, Parser)]
#[command(name = "airbot-play-g2-ws")]
#[command(about = "AIRBOT Play G2 websocket adapter")]
struct Args {
    #[arg(long, default_value = "127.0.0.1:9002")]
    bind: String,
    #[arg(long, default_value = "can0")]
    interface: String,
    #[arg(long, default_value_t = true)]
    allow_control: bool,
    #[arg(long, value_enum, default_value_t = CanWorkerBackend::AsyncFd)]
    can_backend: CanWorkerBackend,
    #[arg(long, value_enum, default_value_t = ModelBackendKind::PlayAnalytical)]
    model_backend: ModelBackendKind,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt().init();

    let args = Args::parse();
    run_eef_websocket_server(build_config(args)).await?;
    Ok(())
}

fn build_config(args: Args) -> EefWebSocketServerConfig {
    EefWebSocketServerConfig {
        bind_addr: args.bind,
        interface: args.interface,
        allow_control: args.allow_control,
        can_backend: args.can_backend,
        model_backend: args.model_backend,
        eef_profile: EefRuntimeProfile::G2,
    }
}

#[cfg(test)]
mod tests {
    use super::{Args, build_config};
    use airbot_play_rust::eef::EefRuntimeProfile;
    use airbot_play_rust::model::ModelBackendKind;
    use clap::Parser;

    #[test]
    fn model_backend_flag_parses() {
        let args =
            Args::try_parse_from(["airbot-play-g2-ws", "--model-backend", "play-analytical"])
                .expect("CLI should accept the analytical backend");
        assert_eq!(args.model_backend, ModelBackendKind::PlayAnalytical);
    }

    #[test]
    fn g2_bin_uses_g2_runtime_profile() {
        let args =
            Args::try_parse_from(["airbot-play-g2-ws"]).expect("CLI should use default arguments");
        let config = build_config(args);
        assert_eq!(config.eef_profile, EefRuntimeProfile::G2);
    }
}
