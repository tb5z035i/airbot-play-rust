use crate::diag::input::DiagInputLine;
use crate::types::{
    CLASSIC_CAN_MAX_DLEN, DecodedFrame, FrameKind, MotorCommand, MotorState, ParamValue,
    ProtocolNode, ProtocolNodeKind,
};

pub fn format_diag_line(input: &DiagInputLine, decoded: &[DecodedFrame]) -> String {
    let mut line = format_raw_prefix(input);
    if !decoded.is_empty() {
        let info = decoded
            .iter()
            .map(format_decoded_frame)
            .collect::<Vec<_>>()
            .join(" || ");
        line.push_str(" | ");
        line.push_str(&info);
    }
    line
}

pub fn format_raw_prefix(input: &DiagInputLine) -> String {
    let mut pieces = Vec::new();
    if let Some(timestamp) = &input.timestamp {
        pieces.push(format!("({timestamp})"));
    }
    pieces.push(input.interface.clone());
    if let Some(direction) = &input.direction {
        pieces.push(direction.clone());
    }
    pieces.push(format!("{:03X}", input.frame.can_id));
    pieces.push(format!("[{}]", input.frame.can_dlc));
    pieces.push(format_payload_region(input.frame.payload()));
    pieces.join(" ")
}

pub fn format_decoded_frame(decoded: &DecodedFrame) -> String {
    match decoded {
        DecodedFrame::MotionCommand {
            node,
            kind,
            command,
        } => format_motion_command(node, *kind, command),
        DecodedFrame::MotionFeedback { node, state } => format_motion_feedback(node, state),
        DecodedFrame::ParamRequest {
            node,
            kind,
            name,
            value,
        } => match value {
            Some(value) => format!(
                "{} {} {}={}",
                format_node(node),
                format_kind(*kind),
                name,
                format_param_value(value)
            ),
            None => format!("{} {} {}", format_node(node), format_kind(*kind), name),
        },
        DecodedFrame::ParamResponse { node, kind, values } => {
            if values.is_empty() {
                format!("{} {}", format_node(node), format_kind(*kind))
            } else {
                let body = values
                    .iter()
                    .map(|(name, value)| format!("{name}={}", format_param_value(value)))
                    .collect::<Vec<_>>()
                    .join(", ");
                format!("{} {} {}", format_node(node), format_kind(*kind), body)
            }
        }
        DecodedFrame::LifecycleCommand { node, kind } => {
            format!("{} {}", format_node(node), format_kind(*kind))
        }
        DecodedFrame::BoardEvent { node, name, value } => {
            format!(
                "{} event {}={}",
                format_node(node),
                name,
                format_param_value(value)
            )
        }
    }
}

fn format_motion_command(node: &ProtocolNode, kind: FrameKind, command: &MotorCommand) -> String {
    match kind {
        FrameKind::MitReq => format!(
            "{} mit pos={} vel={} eff={} kp={} kd={}",
            format_node(node),
            fmt_f64(command.pos),
            fmt_f64(command.vel),
            fmt_f64(command.eff),
            fmt_f64(command.mit_kp),
            fmt_f64(command.mit_kd)
        ),
        FrameKind::PvtReq => format!(
            "{} pvt pos={} vel={} current={}",
            format_node(node),
            fmt_f64(command.pos),
            fmt_f64(command.vel),
            fmt_f64(command.current_threshold)
        ),
        FrameKind::PvReq => format!(
            "{} pv pos={} vel={} current={}",
            format_node(node),
            fmt_f64(command.pos),
            fmt_f64(command.vel),
            fmt_f64(command.current_threshold)
        ),
        FrameKind::CsvReq => format!("{} csv vel={}", format_node(node), fmt_f64(command.vel)),
        other => format!("{} {}", format_node(node), format_kind(other)),
    }
}

fn format_motion_feedback(node: &ProtocolNode, state: &MotorState) -> String {
    format!(
        "{} state pos={} vel={} eff={} temp={} mos={} err={}",
        format_node(node),
        fmt_f64(state.pos),
        fmt_f64(state.vel),
        fmt_f64(state.eff),
        state.motor_temp,
        state.mos_temp,
        state.error_id
    )
}

fn format_kind(kind: FrameKind) -> &'static str {
    match kind {
        FrameKind::MitReq => "mit",
        FrameKind::CsvReq => "csv",
        FrameKind::PvtReq => "pvt",
        FrameKind::PvReq => "pv",
        FrameKind::GetParamReq => "get",
        FrameKind::SetParamReq => "set",
        FrameKind::GetParamResp => "reply",
        FrameKind::SetParamResp => "set-reply",
        FrameKind::PersistParamReq => "persist",
        FrameKind::PersistParamResp => "persist-reply",
        FrameKind::PingReq => "ping",
        FrameKind::EnableReq => "enable",
        FrameKind::DisableReq => "disable",
        FrameKind::SetZeroReq => "set-zero",
        FrameKind::ResetErrReq => "reset-err",
        FrameKind::MotionFeedback => "state",
        FrameKind::LedEffectReq => "led-effect",
        FrameKind::ButtonStateResp => "button",
        FrameKind::Void => "void",
    }
}

fn format_node(node: &ProtocolNode) -> String {
    let label = match node.kind {
        ProtocolNodeKind::OdMotor => "od",
        ProtocolNodeKind::DmMotor => "dm",
        ProtocolNodeKind::PlayBaseBoard => "play_base",
        ProtocolNodeKind::PlayEndBoard => "play_end",
        ProtocolNodeKind::PlayLed => "play_led",
        ProtocolNodeKind::PlayButton => "play_button",
    };
    format!("{label}#{}", node.id)
}

fn format_param_value(value: &ParamValue) -> String {
    match value {
        ParamValue::U8(value) => value.to_string(),
        ParamValue::I8(value) => value.to_string(),
        ParamValue::U16(value) => value.to_string(),
        ParamValue::I16(value) => value.to_string(),
        ParamValue::U32(value) => value.to_string(),
        ParamValue::I32(value) => value.to_string(),
        ParamValue::F32(value) => fmt_f64(*value as f64),
        ParamValue::Bytes4(bytes) | ParamValue::Raw4(bytes) => bytes
            .iter()
            .map(|byte| format!("{:02X}", byte))
            .collect::<Vec<_>>()
            .join(" "),
        ParamValue::String(value) => format!("\"{value}\""),
        ParamValue::FloatVec(values) => format!(
            "[{}]",
            values
                .iter()
                .map(|value| fmt_f64(*value as f64))
                .collect::<Vec<_>>()
                .join(", ")
        ),
        ParamValue::Empty => "<empty>".to_owned(),
    }
}

fn fmt_f64(value: f64) -> String {
    format!("{:4.6}", value)
}

fn format_payload_region(payload: &[u8]) -> String {
    (0..CLASSIC_CAN_MAX_DLEN)
        .map(|index| {
            payload
                .get(index)
                .map(|byte| format!("{:02X}", byte))
                .unwrap_or_else(|| "  ".to_owned())
        })
        .collect::<Vec<_>>()
        .join(" ")
}
