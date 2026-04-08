# /// script
# dependencies = [
#   "websockets>=15.0",
# ]
# ///

"""Simple AIRBOT Play keyboard teleop/status UI over websocket.

Usage examples:

    uv run scripts/airbot_play_keyboard.py --url ws://127.0.0.1:9002

    uv run scripts/airbot_play_keyboard.py --spawn-server --interface can1

Controls:
    q            Quit
    f            Set arm state to free_drive
    c            Set arm state to command_following
    x            Set arm state to disabled
    r            Copy measured joint positions into the target buffer
    s            Send the current joint target buffer
    1..6         Select joint
    h / l        Decrease / increase selected joint target by one step
    H / L        Decrease / increase selected joint target by five steps
    [ / ]        Halve / double the joint step size

Notes:
    - This script talks to `airbot-play-ws`; it does not access CAN directly.
    - For joint motion commands, use `command_following`.
    - For state-only monitoring without motion, `free_drive` is usually the most useful.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import select
import subprocess
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
DEFAULT_ARM_STATE = "free_drive"


@dataclass
class UiState:
    connection_mode: str = "control"
    control_allowed: bool = True
    interface: str = DEFAULT_INTERFACE
    mounted_eef: str = "unknown"
    gravity_coefficients: list[float] = field(default_factory=lambda: [0.0] * 6)
    arm_state: str = "disabled"
    selected_joint: int = 0
    step_size: float = 0.05
    current_positions: list[float | None] = field(default_factory=lambda: [None] * 6)
    current_velocities: list[float | None] = field(default_factory=lambda: [None] * 6)
    current_efforts: list[float | None] = field(default_factory=lambda: [None] * 6)
    target_positions: list[float] = field(default_factory=lambda: [0.0] * 6)
    target_initialized: bool = False
    last_error: str = ""
    last_ack: str = ""
    warnings: deque[str] = field(default_factory=lambda: deque(maxlen=8))
    last_eef_feedback: dict[str, Any] | None = None

    def measured_target_seed(self) -> None:
        if all(value is not None for value in self.current_positions):
            self.target_positions = [float(value) for value in self.current_positions]  # type: ignore[arg-type]
            self.target_initialized = True


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
    parser = argparse.ArgumentParser(description="AIRBOT Play keyboard teleop/status UI")
    parser.add_argument("--url", default=DEFAULT_URL, help=f"Websocket URL (default: {DEFAULT_URL})")
    parser.add_argument(
        "--spawn-server",
        action="store_true",
        help="Spawn `cargo run --bin airbot-play-ws` automatically before connecting.",
    )
    parser.add_argument(
        "--interface",
        default=DEFAULT_INTERFACE,
        help=f"CAN interface to pass to the spawned websocket server (default: {DEFAULT_INTERFACE})",
    )
    parser.add_argument(
        "--bind",
        default=DEFAULT_BIND,
        help=f"Bind address to pass to the spawned websocket server (default: {DEFAULT_BIND})",
    )
    parser.add_argument(
        "--start-state",
        choices=["disabled", "free_drive", "command_following"],
        default=DEFAULT_ARM_STATE,
        help=f"Arm state to request after connecting (default: {DEFAULT_ARM_STATE})",
    )
    parser.add_argument(
        "--readonly",
        action="store_true",
        help="Connect in readonly mode instead of control mode.",
    )
    return parser.parse_args()


async def spawn_server(args: argparse.Namespace) -> asyncio.subprocess.Process:
    cmd = [
        "cargo",
        "run",
        "--bin",
        "airbot-play-ws",
        "--",
        "--interface",
        args.interface,
        "--bind",
        args.bind,
    ]
    if not args.readonly:
        cmd.append("--allow-control")

    process = await asyncio.create_subprocess_exec(
        *cmd,
        stdout=asyncio.subprocess.DEVNULL,
        stderr=asyncio.subprocess.DEVNULL,
    )
    return process


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


async def receiver(ws, state: UiState) -> None:
    async for message in ws:
        payload = json.loads(message)
        message_type = payload.get("type")

        if message_type == "connected":
            info = payload.get("info", {})
            state.connection_mode = payload.get("connection_mode", state.connection_mode)
            state.control_allowed = bool(payload.get("control_allowed", state.control_allowed))
            state.interface = info.get("interface", state.interface)
            mounted_eef = info.get("mounted_eef")
            if isinstance(mounted_eef, str):
                state.mounted_eef = mounted_eef
            state.gravity_coefficients = list(info.get("gravity_coefficients", state.gravity_coefficients))
            state.last_ack = "Connected"
        elif message_type == "ack":
            state.last_ack = payload.get("message", "")
        elif message_type == "error":
            state.last_error = payload.get("message", "")
        elif message_type == "warning":
            warning = payload.get("warning", {})
            text = warning.get("message", str(warning))
            state.warnings.appendleft(text)
        elif message_type == "arm_feedback":
            feedback = payload.get("feedback", {})
            positions = feedback.get("positions", [])
            velocities = feedback.get("velocities", [])
            efforts = feedback.get("torques", [])
            if len(positions) == 6:
                state.current_positions = [float(value) for value in positions]
                state.current_velocities = [float(value) for value in velocities]
                state.current_efforts = [float(value) for value in efforts]
                if not state.target_initialized:
                    state.measured_target_seed()
        elif message_type == "eef_feedback":
            state.last_eef_feedback = payload.get("feedback")
        elif message_type == "joint_target_accepted":
            target = payload.get("target", {})
            positions = target.get("positions", [])
            if len(positions) == 6:
                state.target_positions = [float(value) for value in positions]
                state.target_initialized = True
            state.last_ack = "Joint target accepted"


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
    lines: list[str] = []
    lines.append("AIRBOT Play Keyboard Control")
    lines.append(
        f"Interface: {state.interface} | Mounted EEF: {state.mounted_eef} | "
        f"Mode: {state.connection_mode} | Control allowed: {state.control_allowed}"
    )
    lines.append(f"Arm state: {state.arm_state} | Selected joint: J{state.selected_joint + 1} | Step: {state.step_size:.4f} rad")
    lines.append("")
    lines.append("Joint | Measured pos | Measured vel | Measured eff | Target pos")
    lines.append("------+--------------+--------------+--------------+-----------")

    for index in range(6):
        marker = ">" if index == state.selected_joint else " "
        lines.append(
            f"{marker} J{index + 1} | {fmt_number(state.current_positions[index])} | "
            f"{fmt_number(state.current_velocities[index])} | {fmt_number(state.current_efforts[index])} | "
            f"{state.target_positions[index]:+8.4f}"
        )

    lines.append("")
    if state.last_eef_feedback is not None:
        lines.append(
            "EEF  | pos={} vel={} eff={}".format(
                fmt_number(float(state.last_eef_feedback.get("position", 0.0))),
                fmt_number(float(state.last_eef_feedback.get("velocity", 0.0))),
                fmt_number(float(state.last_eef_feedback.get("effort", 0.0))),
            )
        )
    lines.append("Gravity coefficients: " + ", ".join(f"{value:.3f}" for value in state.gravity_coefficients))
    lines.append("")
    lines.append("Last ack   : " + (state.last_ack or "<none>"))
    lines.append("Last error : " + (state.last_error or "<none>"))
    lines.append("Warnings:")
    if state.warnings:
        lines.extend(f"  - {warning}" for warning in list(state.warnings)[:6])
    else:
        lines.append("  - <none>")
    lines.append("")
    lines.append("Keys: 1..6 select joint | h/l move | H/L move x5 | [/] step | f free | c follow | x disable | r latch | s send | q quit")

    sys.stdout.write("\x1b[H\x1b[2J")
    sys.stdout.write("\n".join(lines))
    sys.stdout.flush()


async def set_arm_state(ws, state: UiState, new_state: str) -> None:
    await send_json(ws, {"type": "set_arm_state", "state": new_state})
    state.arm_state = new_state
    if new_state == "command_following" and not state.target_initialized:
        state.measured_target_seed()


async def send_joint_target(ws, state: UiState) -> None:
    if not state.target_initialized:
        state.measured_target_seed()
    await send_json(ws, {"type": "submit_joint_target", "positions": state.target_positions})


async def handle_key(ws, state: UiState, key: str) -> bool:
    if not key:
        return True

    if key == "q":
        return False
    if key in "123456":
        state.selected_joint = int(key) - 1
        return True
    if key == "[":
        state.step_size = max(0.001, state.step_size / 2.0)
        return True
    if key == "]":
        state.step_size = min(1.0, state.step_size * 2.0)
        return True
    if key == "r":
        state.measured_target_seed()
        state.last_ack = "Target buffer latched from measured positions"
        return True
    if key == "s":
        await send_joint_target(ws, state)
        return True
    if key == "f":
        await set_arm_state(ws, state, "free_drive")
        return True
    if key == "c":
        await set_arm_state(ws, state, "command_following")
        return True
    if key == "x":
        await set_arm_state(ws, state, "disabled")
        return True

    delta = None
    if key == "h":
        delta = -state.step_size
    elif key == "l":
        delta = state.step_size
    elif key == "H":
        delta = -5.0 * state.step_size
    elif key == "L":
        delta = 5.0 * state.step_size

    if delta is not None:
        if not state.target_initialized:
            state.measured_target_seed()
        state.target_positions[state.selected_joint] += delta
        if state.arm_state == "command_following":
            await send_joint_target(ws, state)
        else:
            state.last_ack = "Target updated locally; press c then s to send"
        return True

    return True


async def ui_loop(ws, state: UiState) -> None:
    while True:
        render(state)
        key = try_read_key()
        if key is not None:
            should_continue = await handle_key(ws, state, key)
            if not should_continue:
                break
        await asyncio.sleep(0.05)


async def main() -> None:
    args = parse_args()
    state = UiState(connection_mode="readonly" if args.readonly else "control")
    server_process = None

    try:
        if args.spawn_server:
            server_process = await spawn_server(args)

        async with await connect_with_retry(args.url) as ws:
            await send_json(ws, {"type": "hello", "access_mode": state.connection_mode})
            await send_json(ws, {"type": "subscribe_warnings"})
            await send_json(ws, {"type": "subscribe_arm_feedback"})
            await send_json(ws, {"type": "subscribe_eef_feedback"})

            if args.start_state != "disabled":
                await set_arm_state(ws, state, args.start_state)

            receiver_task = asyncio.create_task(receiver(ws, state))
            try:
                with RawTerminal():
                    await ui_loop(ws, state)
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
    import contextlib

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
