use airbot_play_rust::can::socketcan_io::SocketCanIo;
use airbot_play_rust::diag::decode::DiagnosticRouter;
use airbot_play_rust::diag::format::format_diag_line;
use airbot_play_rust::diag::input::{DiagInputLine, parse_candump_line};
use clap::Parser;
use std::fs::File;
use std::io::{self, BufRead, BufReader, IsTerminal};
use std::path::PathBuf;
use tracing_subscriber::FmtSubscriber;

#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, conflicts_with = "input")]
    interface: Option<String>,
    #[arg(short = 'f', long, conflicts_with = "interface")]
    input: Option<PathBuf>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let subscriber = FmtSubscriber::builder().with_target(false).finish();
    let _ = tracing::subscriber::set_global_default(subscriber);

    let args = Args::parse();
    if let Some(interface) = args.interface {
        run_live(&interface).await?;
    } else if let Some(path) = args.input {
        let file = File::open(path)?;
        run_offline(BufReader::new(file))?;
    } else if !io::stdin().is_terminal() {
        run_offline(BufReader::new(io::stdin().lock()))?;
    } else {
        return Err(
            "provide --interface for live mode, or --input / piped stdin for offline mode".into(),
        );
    }

    Ok(())
}

async fn run_live(interface: &str) -> Result<(), Box<dyn std::error::Error>> {
    let io = SocketCanIo::open(interface.to_owned())?;
    let mut router = DiagnosticRouter::default();

    loop {
        let frame = io.recv().await?;
        let input = DiagInputLine {
            timestamp: None,
            interface: interface.to_owned(),
            direction: Some("RX".to_owned()),
            frame,
        };
        let decoded = router.inspect(&input.frame);
        println!("{}", format_diag_line(&input, &decoded));
    }
}

fn run_offline<R: BufRead>(reader: R) -> Result<(), Box<dyn std::error::Error>> {
    let mut router = DiagnosticRouter::default();

    for line in reader.lines() {
        let line = line?;
        match parse_candump_line(&line) {
            Ok(Some(parsed)) => {
                let decoded = router.inspect(&parsed.frame);
                println!("{}", format_diag_line(&parsed, &decoded));
            }
            Ok(None) => {}
            Err(err) => println!("{line} | parse-error {err}"),
        }
    }

    Ok(())
}
