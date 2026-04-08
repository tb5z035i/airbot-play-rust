use airbot_play_rust::protocol::board::BoardProtocol;
use airbot_play_rust::protocol::board::play_base::PlayBaseBoardProtocol;
use airbot_play_rust::protocol::board::play_end::PlayEndBoardProtocol;
use airbot_play_rust::protocol::motor::MotorProtocol;
use airbot_play_rust::protocol::motor::dm::DmProtocol;
use airbot_play_rust::protocol::motor::od::OdProtocol;
use airbot_play_rust::types::{DecodedFrame, FrameKind, MotorCommand, ParamValue, RawCanFrame};
use pretty_assertions::assert_eq;

#[test]
fn od_mit_request_roundtrip() {
    let mut od = OdProtocol::new(1);
    let command = MotorCommand {
        pos: 1.2,
        vel: -0.7,
        eff: 2.5,
        mit_kp: 120.0,
        mit_kd: 4.0,
        current_threshold: 0.0,
    };

    let frames = od.generate_mit(&command).unwrap();
    assert_eq!(frames.len(), 1);
    let decoded = od.inspect(&frames[0]).unwrap();

    match decoded {
        DecodedFrame::MotionCommand { kind, .. } => assert_eq!(kind, FrameKind::MitReq),
        other => panic!("unexpected decoded frame: {other:?}"),
    }
}

#[test]
fn od_param_get_request_roundtrip() {
    let mut od = OdProtocol::new(3);
    let frames = od.generate_param_get("board_sn").unwrap();
    assert_eq!(frames.len(), 1);
    let decoded = od.inspect(&frames[0]).unwrap();

    match decoded {
        DecodedFrame::ParamRequest { name, kind, .. } => {
            assert_eq!(kind, FrameKind::GetParamReq);
            assert_eq!(name, "board_sn");
        }
        other => panic!("unexpected decoded frame: {other:?}"),
    }
}

#[test]
fn od_string_param_response_reassembles() {
    let mut od = OdProtocol::new(1);
    let chunks = [*b"ABCD", *b"EFGH", *b"IJKL", *b"MNOP"];
    let mut final_decode = None;

    for (index, chunk) in chunks.iter().enumerate() {
        let payload = [
            0x03,
            (index + 1) as u8,
            chunk[0],
            chunk[1],
            chunk[2],
            chunk[3],
        ];
        let frame = RawCanFrame::new(0x101, &payload).unwrap();
        final_decode = od.inspect(&frame);
    }

    match final_decode.expect("expected completed decode") {
        DecodedFrame::ParamResponse { values, .. } => {
            assert_eq!(
                values.get("board_sn"),
                Some(&ParamValue::String("ABCDEFGHIJKLMNOP".to_owned()))
            );
        }
        other => panic!("unexpected decoded frame: {other:?}"),
    }
}

#[test]
fn dm_param_get_request_roundtrip() {
    let mut dm = DmProtocol::new(4);
    let frames = dm.generate_param_get("control_mode").unwrap();
    assert_eq!(frames.len(), 1);
    let decoded = dm.inspect(&frames[0]).unwrap();

    match decoded {
        DecodedFrame::ParamRequest { name, kind, .. } => {
            assert_eq!(kind, FrameKind::GetParamReq);
            assert_eq!(name, "control_mode");
        }
        other => panic!("unexpected decoded frame: {other:?}"),
    }
}

#[test]
fn dm_mit_request_roundtrip() {
    let mut dm = DmProtocol::new(5);
    let command = MotorCommand {
        pos: 0.5,
        vel: 0.1,
        eff: 1.0,
        mit_kp: 30.0,
        mit_kd: 1.5,
        current_threshold: 0.0,
    };
    let frames = dm.generate_mit(&command).unwrap();
    let decoded = dm.inspect(&frames[0]).unwrap();

    match decoded {
        DecodedFrame::MotionCommand { kind, .. } => assert_eq!(kind, FrameKind::MitReq),
        other => panic!("unexpected decoded frame: {other:?}"),
    }
}

#[test]
fn dm_param_response_roundtrip() {
    let mut dm = DmProtocol::new(6);
    let payload = [0x06, 0x00, 0x33, 0x0A, 0x04, 0x00, 0x00, 0x00];
    let frame = RawCanFrame::new(0x706, &payload).unwrap();
    let decoded = dm.inspect(&frame).unwrap();

    match decoded {
        DecodedFrame::ParamResponse { values, kind, .. } => {
            assert_eq!(kind, FrameKind::GetParamResp);
            assert_eq!(values.get("control_mode"), Some(&ParamValue::U32(4)));
        }
        other => panic!("unexpected decoded frame: {other:?}"),
    }
}

#[test]
fn board_param_get_request_roundtrip() {
    let mut base = PlayBaseBoardProtocol::new();
    let frames = base.generate_param_get("pcba_name").unwrap();
    let decoded = base.inspect(&frames[0]).unwrap();

    match decoded {
        DecodedFrame::ParamRequest { name, kind, .. } => {
            assert_eq!(kind, FrameKind::GetParamReq);
            assert_eq!(name, "pcba_name");
        }
        other => panic!("unexpected decoded frame: {other:?}"),
    }
}

#[test]
fn board_string_response_reassembles() {
    let mut base = PlayBaseBoardProtocol::new();
    let chunks = [
        *b"AIRB",
        *b"OT-P",
        *b"LAY-",
        *b"BASE",
        *b"-001",
        *b"\0\0\0\0",
    ];
    let mut final_decode = None;
    for (index, chunk) in chunks.iter().enumerate() {
        let payload = [
            0x07,
            (index + 1) as u8,
            chunk[0],
            chunk[1],
            chunk[2],
            chunk[3],
        ];
        let frame = RawCanFrame::new(0x100, &payload).unwrap();
        final_decode = base.inspect(&frame);
    }

    match final_decode.expect("expected completed decode") {
        DecodedFrame::ParamResponse { values, .. } => {
            assert_eq!(
                values.get("pcba_name"),
                Some(&ParamValue::String("AIRBOT-PLAY-BASE-001".to_owned()))
            );
        }
        other => panic!("unexpected decoded frame: {other:?}"),
    }
}

#[test]
fn board_float_vector_reassembles() {
    let mut base = PlayBaseBoardProtocol::new();
    let values = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0];
    let mut final_decode = None;
    for (index, value) in values.iter().enumerate() {
        let mut payload = [0_u8; 6];
        payload[0] = 0x0C;
        payload[1] = (index + 1) as u8;
        payload[2..6].copy_from_slice(&value.to_le_bytes());
        let frame = RawCanFrame::new(0x100, &payload).unwrap();
        final_decode = base.inspect(&frame);
    }

    match final_decode.expect("expected completed decode") {
        DecodedFrame::ParamResponse { values, .. } => {
            assert_eq!(
                values.get("zero_position"),
                Some(&ParamValue::FloatVec(vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))
            );
        }
        other => panic!("unexpected decoded frame: {other:?}"),
    }
}

#[test]
fn play_end_eef_type_response_decodes() {
    let mut end = PlayEndBoardProtocol::new();
    let payload = [0x05, 0x01, 0x03, 0x00, 0x00, 0x00];
    let frame = RawCanFrame::new(0x108, &payload).unwrap();
    let decoded = end.inspect(&frame).unwrap();

    match decoded {
        DecodedFrame::ParamResponse { values, .. } => {
            assert_eq!(values.get("eef_type"), Some(&ParamValue::U32(3)));
        }
        other => panic!("unexpected decoded frame: {other:?}"),
    }
}
