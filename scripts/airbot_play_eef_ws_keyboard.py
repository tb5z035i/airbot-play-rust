# /// script
# dependencies = [
#   "websockets>=15.0",
# ]
# ///

"""Keyboard UI for AIRBOT E2/G2 feedback over websocket.

Usage examples:

    uv run scripts/airbot_play_eef_ws_keyboard.py --url ws://127.0.0.1:9002

    uv run scripts/airbot_play_eef_ws_keyboard.py --spawn-server --server-kind g2 --interface can0

    uv run scripts/airbot_play_eef_ws_keyboard.py --spawn-server --server-kind e2 --interface can0

Controls:
    q            Quit
    e            Set end-effector state to enabled
    x            Set end-effector state to disabled
    r            Latch the current G2 opening into the target buffer
    o / c        Open / close G2 by one step
    O / C        Open / close G2 by five steps
    [ / ]        Halve / double the G2 step size

Notes:
    - This script talks to a websocket server; it does not access CAN directly.
    - E2 is feedback-oriented here: after enabling the EEF, the server pumps zero MIT commands
      so the script can show live values without sending explicit E2 motion commands.
    - G2 open/close uses `submit_g2_mit_command`, and the server keeps replaying the latest
      MIT target after you change it.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import os
import select
import sys
import termios
import tty
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import websockets


DEFAULT_URL = "ws://127.0.0.1:9002"
DEFAULT_BIND = "127.0.0.1:9002"
DEFAULT_INTERFACE = "can0"
DEFAULT_SERVER_KIND = "generic"
DEFAULT_G2_STEP = 0.002
DEFAULT_G2_MIN_POSITION = 0.0
DEFAULT_G2_MAX_POSITION = 0.072
DEFAULT_G2_MIT_KP = 30.0
DEFAULT_G2_MIT_KD = 1.5
DEFAULT_G2_EFFORT = 0.0


@dataclass
class UiState:
    connection_mode: str = "control"
    control_allowed: bool = True
    interface: str = DEFAULT_INTERFACE
    mounted_eef: str = "unknown"
    eef_state: str = "disabled"
    g2_step: float = DEFAULT_G2_STEP
    g2_target_position: float | None = None
    last_feedback: dict[str, Any] | None = None
    last_error: str = ""
    last_ack: str = ""
    warnings: deque[str] = field(default_factory=lambda: deque(maxlen=8))

    def measured_position(self) -> float | None:
        if self.last_feedback is None:
            return None
        try:
            return float(self.last_feedback.get("position"))
        except (TypeError, ValueError):
            return None

    def measured_velocity(self) -> float | None:
        if self.last_feedback is None:
            return None
        try:
            return float(self.last_feedback.get("velocity"))
        except (TypeError, ValueError):
            return None

    def measured_effort(self) -> float | None:
        if self.last_feedback is None:
            return None
        try:
            return float(self.last_feedback.get("effort"))
        except (TypeError, ValueError):
            return None

    def measured_valid(self) -> bool | None:
        if self.last_feedback is None:
            return None
        value = self.last_feedback.get("valid")
        if value is None:
            return None
        return bool(value)


class RawTerminal:
    def __enter__(self) -> "RawTerminal":
        if not sys.stdin.isatty():
            raise SystemExit("This script requires an interactive TTY.")
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        sys.stdout.write("\x1b[?25l")
        sys.stdout.flush()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        sys.stdout.write("\x1b[?25h\x1b[0m\n")
        sys.stdout.flush()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AIRBOT E2/G2 websocket keyboard UI")
    parser.add_argument("--url", default=DEFAULT_URL, help=f"Websocket URL (default: {DEFAULT_URL})")
    parser.add_argument(
        "--spawn-server",
        action="store_true",
        help="Spawn a websocket server automatically before connecting.",
    )
    parser.add_argument(
        "--server-kind",
        choices=["generic", "e2", "g2"],
        default=DEFAULT_SERVER_KIND,
        help="Server wrapper to spawn with --spawn-server (default: generic).",
    )
    parser.add_argument(
        "--interface",
        default=DEFAULT_INTERFACE,
        help=f"CAN interface to pass to the spawned server (default: {DEFAULT_INTERFACE})",
    )
    parser.add_argument(
        "--bind",
        default=DEFAULT_BIND,
        help=f"Bind address to pass to the spawned server (default: {DEFAULT_BIND})",
    )
    parser.add_argument(
        "--readonly",
        action="store_true",
        help="Connect in readonly mode instead of control mode.",
    )
    parser.add_argument(
        "--g2-step",
        type=float,
        default=DEFAULT_G2_STEP,
        help=f"G2 open/close step size in meters (default: {DEFAULT_G2_STEP})",
    )
    parser.add_argument(
        "--g2-min-position",
        type=float,
        default=DEFAULT_G2_MIN_POSITION,
        help=f"G2 minimum opening in meters (default: {DEFAULT_G2_MIN_POSITION})",
    )
    parser.add_argument(
        "--g2-max-position",
        type=float,
        default=DEFAULT_G2_MAX_POSITION,
        help=f"G2 maximum opening in meters (default: {DEFAULT_G2_MAX_POSITION})",
    )
    parser.add_argument(
        "--g2-mit-kp",
        type=float,
        default=DEFAULT_G2_MIT_KP,
        help=f"G2 MIT proportional gain (default: {DEFAULT_G2_MIT_KP})",
    )
    parser.add_argument(
        "--g2-mit-kd",
        type=float,
        default=DEFAULT_G2_MIT_KD,
        help=f"G2 MIT derivative gain (default: {DEFAULT_G2_MIT_KD})",
    )
    parser.add_argument(
        "--g2-effort",
        type=float,
        default=DEFAULT_G2_EFFORT,
        help=f"G2 MIT feedforward effort (default: {DEFAULT_G2_EFFORT})",
    )
    return parser.parse_args()


def server_binary(server_kind: str) -> str:
    if server_kind == "e2":
        return "airbot-play-e2-ws"
    if server_kind == "g2":
        return "airbot-play-g2-ws"
    return "airbot-play-ws"


async def spawn_server(args: argparse.Namespace) -> asyncio.subprocess.Process:
    cmd = [
        "cargo",
        "run",
        "--bin",
        server_binary(args.server_kind),
        "--",
        "--interface",
        args.interface,
        "--bind",
        args.bind,
    ]
    if not args.readonly:
        cmd.append("--allow-control")

    return await asyncio.create_subprocess_exec(
        *cmd,
        stdout=asyncio.subprocess.DEVNULL,
        stderr=asyncio.subprocess.DEVNULL,
    )


async def connect_with_retry(url: str, timeout_s: float = 30.0):
    deadline = asyncio.get_running_loop().time() + timeout_s
    while True:
        try:
            return await websockets.connect(url)
        except OSError:
            if asyncio.get_running_loop().time() >= deadline:
                raise
            await asyncio.sleep(0.5)


async def send_json(ws, payload: dict[str, Any]) -> None:
    await ws.send(json.dumps(payload))


def mounted_eef_label(value: Any) -> str:
    if isinstance(value, str):
        return value
    return str(value)


def maybe_update_connection_mode_from_ack(state: UiState, message: str) -> None:
    prefix = "connection mode set to "
    lowered = message.lower()
    if lowered.startswith(prefix):
        state.connection_mode = lowered[len(prefix) :].strip()


async def receiver(ws, state: UiState) -> None:
    async for message in ws:
        payload = json.loads(message)
        message_type = payload.get("type")

        if message_type == "connected":
            info = payload.get("info", {})
            state.control_allowed = bool(payload.get("control_allowed", state.control_allowed))
            state.interface = info.get("interface", state.interface)
            state.mounted_eef = mounted_eef_label(info.get("mounted_eef", state.mounted_eef))
            state.last_ack = "Connected"
        elif message_type == "ack":
            state.last_ack = payload.get("message", "")
            maybe_update_connection_mode_from_ack(state, state.last_ack)
        elif message_type == "error":
            state.last_error = payload.get("message", "")
        elif message_type == "warning":
            warning = payload.get("warning", {})
            text = warning.get("message", str(warning))
            state.warnings.appendleft(text)
        elif message_type == "eef_feedback":
            feedback = payload.get("feedback", {})
            state.last_feedback = feedback
            if state.mounted_eef == "G2" and state.g2_target_position is None:
                try:
                    state.g2_target_position = float(feedback.get("position"))
                except (TypeError, ValueError):
                    pass


def try_read_key() -> str | None:
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return None
    data = os.read(sys.stdin.fileno(), 8)
    if not data:
        return None
    return data.decode(errors="ignore")


def fmt_number(value: float | None) -> str:
    if value is None:
        return "   ---   "
    return f"{value:+8.4f}"


def render(state: UiState) -> None:
    measured_position = state.measured_position()
    measured_velocity = state.measured_velocity()
    measured_effort = state.measured_effort()
    measured_valid = state.measured_valid()

    lines: list[str] = []
    lines.append("AIRBOT EEF Websocket Keyboard")
    lines.append(
        f"Interface: {state.interface} | Mounted EEF: {state.mounted_eef} | "
        f"Mode: {state.connection_mode} | Control allowed: {state.control_allowed}"
    )
    lines.append(f"EEF state: {state.eef_state} | G2 step: {state.g2_step:.4f} m")
    lines.append("")
    lines.append("Measured")
    lines.append(f"  position : {fmt_number(measured_position)}")
    lines.append(f"  velocity : {fmt_number(measured_velocity)}")
    lines.append(f"  effort   : {fmt_number(measured_effort)}")
    lines.append(f"  valid    : {measured_valid if measured_valid is not None else '<unknown>'}")
    if state.mounted_eef == "G2":
        lines.append(f"  target   : {fmt_number(state.g2_target_position)}")
    lines.append("")
    lines.append("Last ack   : " + (state.last_ack or "<none>"))
    lines.append("Last error : " + (state.last_error or "<none>"))
    lines.append("Warnings:")
    if state.warnings:
        lines.extend(f"  - {warning}" for warning in list(state.warnings)[:6])
    else:
        lines.append("  - <none>")
    lines.append("")
    if state.mounted_eef == "G2":
        lines.append("Keys: e enable | x disable | r latch | o/c open/close | O/C x5 | [/] step | q quit")
    else:
        lines.append("Keys: e enable | x disable | q quit")

    sys.stdout.write("\x1b[H\x1b[2J")
    sys.stdout.write("\n".join(lines))
    sys.stdout.flush()


async def set_eef_state(ws, state: UiState, new_state: str) -> None:
    if state.connection_mode != "control" or not state.control_allowed:
        state.last_error = "Control mode is required for end-effector state changes."
        return
    await send_json(ws, {"type": "set_eef_state", "state": new_state})
    state.eef_state = new_state


def clamp_position(args: argparse.Namespace, position: float) -> float:
    return max(args.g2_min_position, min(args.g2_max_position, position))


async def send_g2_target(ws, state: UiState, args: argparse.Namespace, position: float) -> None:
    clamped = clamp_position(args, position)
    state.g2_target_position = clamped
    await send_json(
        ws,
        {
            "type": "submit_g2_mit_command",
            "command": {
                "position": clamped,
                "velocity": 0.0,
                "effort": args.g2_effort,
                "mit_kp": args.g2_mit_kp,
                "mit_kd": args.g2_mit_kd,
                "current_threshold": 0.0,
            },
        },
    )


async def handle_g2_delta(
    ws,
    state: UiState,
    args: argparse.Namespace,
    delta: float,
) -> None:
    if state.mounted_eef != "G2":
        state.last_error = "Open/close control is only available when the mounted EEF is G2."
        return
    if state.connection_mode != "control" or not state.control_allowed:
        state.last_error = "Control mode is required for G2 open/close commands."
        return

    base = state.g2_target_position
    if base is None:
        base = state.measured_position()
    if base is None:
        state.last_error = "No G2 feedback yet; wait for the first feedback sample."
        return

    await send_g2_target(ws, state, args, base + delta)


async def handle_key(ws, state: UiState, args: argparse.Namespace, key: str) -> bool:
    if not key:
        return True

    if key == "q":
        return False
    if key == "e":
        await set_eef_state(ws, state, "enabled")
        return True
    if key == "x":
        await set_eef_state(ws, state, "disabled")
        return True
    if key == "[":
        state.g2_step = max(0.0005, state.g2_step / 2.0)
        return True
    if key == "]":
        state.g2_step = min(0.02, state.g2_step * 2.0)
        return True
    if key == "r":
        measured = state.measured_position()
        if measured is None:
            state.last_error = "No end-effector feedback available to latch."
        else:
            state.g2_target_position = clamp_position(args, measured)
            state.last_ack = "Latched the current G2 position into the target buffer"
        return True
    if key == "o":
        await handle_g2_delta(ws, state, args, state.g2_step)
        return True
    if key == "c":
        await handle_g2_delta(ws, state, args, -state.g2_step)
        return True
    if key == "O":
        await handle_g2_delta(ws, state, args, 5.0 * state.g2_step)
        return True
    if key == "C":
        await handle_g2_delta(ws, state, args, -5.0 * state.g2_step)
        return True

    return True


async def ui_loop(ws, state: UiState, args: argparse.Namespace) -> None:
    while True:
        render(state)
        key = try_read_key()
        if key is not None:
            should_continue = await handle_key(ws, state, args, key)
            if not should_continue:
                break
        await asyncio.sleep(0.05)


async def main() -> None:
    args = parse_args()
    state = UiState(
        connection_mode="readonly" if args.readonly else "control",
        g2_step=args.g2_step,
    )
    server_process = None

    try:
        if args.spawn_server:
            server_process = await spawn_server(args)

        async with await connect_with_retry(args.url) as ws:
            await send_json(ws, {"type": "hello", "access_mode": state.connection_mode})
            await send_json(ws, {"type": "subscribe_warnings"})
            await send_json(ws, {"type": "subscribe_eef_feedback"})

            receiver_task = asyncio.create_task(receiver(ws, state))
            try:
                with RawTerminal():
                    await ui_loop(ws, state, args)
            finally:
                receiver_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await receiver_task
    finally:
        if server_process is not None:
            server_process.terminate()
            with contextlib.suppress(ProcessLookupError):
                await server_process.wait()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
