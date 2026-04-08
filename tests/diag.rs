use airbot_play_rust::diag::decode::DiagnosticRouter;
use airbot_play_rust::diag::format::format_diag_line;
use airbot_play_rust::diag::input::{DiagInputLine, parse_candump_line};
use airbot_play_rust::types::RawCanFrame;
use pretty_assertions::assert_eq;

#[test]
fn parses_plain_candump_line() {
    let line = "  can1  001   [1]  00";
    let parsed = parse_candump_line(line).unwrap().unwrap();

    assert_eq!(parsed.timestamp, None);
    assert_eq!(parsed.interface, "can1");
    assert_eq!(parsed.direction, None);
    assert_eq!(parsed.frame, RawCanFrame::new(0x001, &[0x00]).unwrap());
}

#[test]
fn parses_candump_xta_line() {
    let line = "(1775638165.450824)  can1  TX - -  001   [1]  00";
    let parsed = parse_candump_line(line).unwrap().unwrap();

    assert_eq!(parsed.timestamp.as_deref(), Some("1775638165.450824"));
    assert_eq!(parsed.interface, "can1");
    assert_eq!(parsed.direction.as_deref(), Some("TX"));
    assert_eq!(parsed.frame, RawCanFrame::new(0x001, &[0x00]).unwrap());
}

#[test]
fn parses_candump_axta_line() {
    let line = "(1775638083.790747)  can1  TX - -  001   [1]  00                        '.'";
    let parsed = parse_candump_line(line).unwrap().unwrap();

    assert_eq!(parsed.timestamp.as_deref(), Some("1775638083.790747"));
    assert_eq!(parsed.interface, "can1");
    assert_eq!(parsed.direction.as_deref(), Some("TX"));
    assert_eq!(parsed.frame, RawCanFrame::new(0x001, &[0x00]).unwrap());
}

#[test]
fn formats_single_line_human_readable_output() {
    let mut router = DiagnosticRouter::default();
    let input = DiagInputLine {
        timestamp: Some("1775638165.450824".to_owned()),
        interface: "can1".to_owned(),
        direction: Some("TX".to_owned()),
        frame: RawCanFrame::new(0x001, &[0x00]).unwrap(),
    };

    let decoded = router.inspect(&input.frame);
    let line = format_diag_line(&input, &decoded);

    assert_eq!(
        line,
        "(1775638165.450824) can1 TX 001 [1] 00                      | od#1 get board_id"
    );
}

#[test]
fn multi_frame_info_only_appears_on_last_row() {
    let mut router = DiagnosticRouter::default();
    let chunks = [*b"ABCD", *b"EFGH", *b"IJKL", *b"MNOP"];
    let mut lines = Vec::new();

    for (index, chunk) in chunks.iter().enumerate() {
        let input = DiagInputLine {
            timestamp: None,
            interface: "can1".to_owned(),
            direction: Some("RX".to_owned()),
            frame: RawCanFrame::new(
                0x101,
                &[
                    0x03,
                    (index + 1) as u8,
                    chunk[0],
                    chunk[1],
                    chunk[2],
                    chunk[3],
                ],
            )
            .unwrap(),
        };
        let decoded = router.inspect(&input.frame);
        lines.push(format_diag_line(&input, &decoded));
    }

    assert_eq!(lines[0], "can1 RX 101 [6] 03 01 41 42 43 44      ");
    assert_eq!(lines[1], "can1 RX 101 [6] 03 02 45 46 47 48      ");
    assert_eq!(lines[2], "can1 RX 101 [6] 03 03 49 4A 4B 4C      ");
    assert_eq!(
        lines[3],
        "can1 RX 101 [6] 03 04 4D 4E 4F 50       | od#1 reply board_sn=\"ABCDEFGHIJKLMNOP\""
    );
}
