use crate::types::RawCanFrame;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DiagInputLine {
    pub timestamp: Option<String>,
    pub interface: String,
    pub direction: Option<String>,
    pub frame: RawCanFrame,
}

pub fn parse_candump_line(line: &str) -> Result<Option<DiagInputLine>, String> {
    let trimmed = line.trim();
    if trimmed.is_empty() {
        return Ok(None);
    }

    let mut rest = trimmed;
    let timestamp = if let Some(stripped) = rest.strip_prefix('(') {
        let end = stripped
            .find(')')
            .ok_or_else(|| format!("invalid candump timestamp line: {line}"))?;
        let ts = stripped[..end].trim().to_owned();
        rest = stripped[end + 1..].trim_start();
        Some(ts)
    } else {
        None
    };

    let tokens = rest.split_whitespace().collect::<Vec<_>>();
    if tokens.len() < 3 {
        return Err(format!("not enough candump fields: {line}"));
    }

    let interface = tokens[0].to_owned();
    let mut idx = 1;

    let direction = if tokens
        .get(idx)
        .is_some_and(|value| *value == "TX" || *value == "RX")
    {
        let value = tokens[idx].to_owned();
        idx += 1;
        Some(value)
    } else {
        None
    };

    while tokens.get(idx).is_some_and(|value| *value == "-") {
        idx += 1;
    }

    let can_id_token = tokens
        .get(idx)
        .ok_or_else(|| format!("missing CAN id in line: {line}"))?;
    idx += 1;
    let dlc_token = tokens
        .get(idx)
        .ok_or_else(|| format!("missing DLC in line: {line}"))?;
    idx += 1;

    let can_id = u32::from_str_radix(can_id_token, 16)
        .map_err(|_| format!("invalid CAN id `{can_id_token}` in line: {line}"))?;
    let dlc = parse_dlc(dlc_token)?;

    let mut payload = Vec::with_capacity(dlc);
    for _ in 0..dlc {
        let token = tokens
            .get(idx)
            .ok_or_else(|| format!("missing payload byte in line: {line}"))?;
        idx += 1;
        payload.push(
            u8::from_str_radix(token, 16)
                .map_err(|_| format!("invalid payload byte `{token}` in line: {line}"))?,
        );
    }

    let frame = RawCanFrame::new(can_id, &payload)
        .map_err(|err| format!("invalid CAN frame in line `{line}`: {err}"))?;

    Ok(Some(DiagInputLine {
        timestamp,
        interface,
        direction,
        frame,
    }))
}

fn parse_dlc(token: &str) -> Result<usize, String> {
    let stripped = token
        .strip_prefix('[')
        .and_then(|value| value.strip_suffix(']'))
        .ok_or_else(|| format!("invalid DLC token `{token}`"))?;

    stripped
        .parse::<usize>()
        .map_err(|_| format!("invalid DLC token `{token}`"))
}
