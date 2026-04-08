use crate::types::RawCanFrame;
use socketcan::{CanDataFrame, CanFrame, EmbeddedFrame, Frame, ShouldRetry, Socket};
use std::fs;
use std::io;
use std::path::Path;
use thiserror::Error;
use tokio::io::unix::AsyncFd;

#[derive(Debug, Error)]
pub enum SocketCanIoError {
    #[error("I/O error: {0}")]
    Io(#[from] io::Error),
    #[error("invalid CAN frame: {0}")]
    InvalidFrame(String),
}

#[derive(Debug)]
pub struct SocketCanIo {
    interface: String,
    socket: AsyncFd<socketcan::CanSocket>,
}

impl SocketCanIo {
    pub fn open(interface: impl Into<String>) -> Result<Self, SocketCanIoError> {
        let interface = interface.into();
        let socket = socketcan::CanSocket::open(&interface)?;
        socket.set_nonblocking(true)?;
        let socket = AsyncFd::new(socket)?;
        Ok(Self { interface, socket })
    }

    pub fn interface(&self) -> &str {
        &self.interface
    }

    pub async fn recv(&self) -> Result<RawCanFrame, SocketCanIoError> {
        loop {
            let mut readiness = self.socket.readable().await?;
            match self.socket.get_ref().read_frame() {
                Ok(frame) => return frame_to_raw(frame),
                Err(err) if err.should_retry() => {
                    readiness.clear_ready();
                }
                Err(err) => {
                    readiness.clear_ready();
                    return Err(SocketCanIoError::Io(err));
                }
            }
        }
    }

    pub async fn send(&self, frame: &RawCanFrame) -> Result<(), SocketCanIoError> {
        let can_frame =
            <CanDataFrame as socketcan::Frame>::from_raw_id(frame.can_id, frame.payload())
                .ok_or_else(|| {
                    SocketCanIoError::InvalidFrame(format!("failed to build frame {frame}"))
                })?;

        loop {
            let mut readiness = self.socket.writable().await?;
            match self.socket.get_ref().write_frame(&can_frame) {
                Ok(()) => {
                    readiness.clear_ready();
                    return Ok(());
                }
                Err(err) if err.should_retry() => {
                    readiness.clear_ready();
                }
                Err(err) => {
                    readiness.clear_ready();
                    return Err(SocketCanIoError::Io(err));
                }
            }
        }
    }

    pub fn drain_pending(&self, max_frames: usize) -> Result<Vec<RawCanFrame>, SocketCanIoError> {
        let mut drained = Vec::new();
        for _ in 0..max_frames {
            match self.socket.get_ref().read_frame() {
                Ok(frame) => drained.push(frame_to_raw(frame)?),
                Err(err) if err.should_retry() => break,
                Err(err) => return Err(SocketCanIoError::Io(err)),
            }
        }
        Ok(drained)
    }

    pub fn candidate_interfaces() -> io::Result<Vec<String>> {
        let mut interfaces = Vec::new();
        for entry in fs::read_dir("/sys/class/net")? {
            let entry = entry?;
            let path = entry.path();
            let type_path = path.join("type");
            let flags_path = path.join("flags");
            if !type_path.exists() || !flags_path.exists() {
                continue;
            }

            let interface_type = fs::read_to_string(&type_path)?;
            if interface_type.trim() != "280" {
                continue;
            }

            let flags = fs::read_to_string(flags_path)?;
            let flags = flags.trim().trim_start_matches("0x");
            let is_up = u64::from_str_radix(flags, 16).unwrap_or_default() & 0x1 == 0x1;
            if is_up {
                interfaces.push(
                    path.file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .to_string(),
                );
            }
        }
        interfaces.sort();
        Ok(interfaces)
    }

    pub fn interface_exists(interface: &str) -> bool {
        Path::new("/sys/class/net").join(interface).exists()
    }
}

pub fn raw_to_can_data_frame(frame: &RawCanFrame) -> Result<CanDataFrame, SocketCanIoError> {
    <CanDataFrame as socketcan::Frame>::from_raw_id(frame.can_id, frame.payload())
        .ok_or_else(|| SocketCanIoError::InvalidFrame(format!("failed to build frame {frame}")))
}

pub fn frame_to_raw(frame: CanFrame) -> Result<RawCanFrame, SocketCanIoError> {
    RawCanFrame::new(frame.raw_id(), frame.data()).map_err(SocketCanIoError::InvalidFrame)
}
