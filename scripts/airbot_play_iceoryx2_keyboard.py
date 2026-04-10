# /// script
# dependencies = [
#   "iceoryx2==0.8.1",
# ]
#
# [tool.uv.sources]
# iceoryx2 = { path = "../third_party/iceoryx2/iceoryx2-ffi/python", editable = true }
# ///

"""Simple AIRBOT Play keyboard teleop/status UI over iceoryx2.

Usage examples:

    uv run scripts/airbot_play_iceoryx2_keyboard.py --service-root airbot-play/can0

    uv run scripts/airbot_play_iceoryx2_keyboard.py --spawn-server --interface can1

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
    - This script talks to `airbot-play-iceoryx2`; it does not access CAN directly.
    - The default service root is `airbot-play/<interface>`.
    - For joint motion commands, use `command_following`.
    - For state-only monitoring without motion, `free_drive` is usually the most useful.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import ctypes
import json
import os
import select
import sys
import termios
import time
import tty
from collections import deque
from dataclasses import dataclass, field
from typing import Any


DEFAULT_INTERFACE = "can0"
DEFAULT_SERVICE_ROOT_TEMPLATE = "airbot-play/{interface}"
DEFAULT_ARM_STATE = "free_drive"
DEFAULT_CONNECT_TIMEOUT_S = 30.0
DEFAULT_POLL_MS = 10
DEFAULT_MAX_MESSAGE_SIZE = 16 * 1024


def _require_iceoryx2():
    try:
        import iceoryx2 as iox2
    except Exception as exc:  # pragma: no cover - depends on local Python environment
        raise SystemExit(
            "This script requires the Python `iceoryx2` bindings. "
            "Install them first, for example with `uv add iceoryx2`."
        ) from exc
    return iox2


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

    def current_position_snapshot(self) -> list[float] | None:
        if not all(value is not None for value in self.current_positions):
            return None
        return [float(value) for value in self.current_positions if value is not None]

    def measured_target_seed(self, *, force: bool = False) -> None:
        if self.target_initialized and not force:
            return
        snapshot = self.current_position_snapshot()
        if snapshot is None:
            return
        self.target_positions = snapshot
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


class Iceoryx2Session:
    def __init__(self, args: argparse.Namespace, state: UiState) -> None:
        self._iox2 = _require_iceoryx2()
        self._args = args
        self._state = state
        self._service_root = effective_service_root(args)
        self._server_process: asyncio.subprocess.Process | None = None
        self._receiver_task: asyncio.Task[None] | None = None
        self._connected = asyncio.Event()
        self._pending_ack: str | None = None
        self._pending_ack_event = asyncio.Event()

        self._node = self._iox2.NodeBuilder.new().create(self._iox2.ServiceType.Ipc)
        request_service = (
            self._node.service_builder(
                self._iox2.ServiceName.new(f"{self._service_root}/requests")
            )
            .publish_subscribe(self._iox2.Slice[ctypes.c_uint8])
            .open_or_create()
        )
        event_service = (
            self._node.service_builder(
                self._iox2.ServiceName.new(f"{self._service_root}/events")
            )
            .publish_subscribe(self._iox2.Slice[ctypes.c_uint8])
            .open_or_create()
        )
        self._request_publisher = (
            request_service.publisher_builder()
            .initial_max_slice_len(args.max_message_size)
            .create()
        )
        self._event_subscriber = event_service.subscriber_builder().create()

    async def __aenter__(self) -> "Iceoryx2Session":
        if self._args.spawn_server:
            self._server_process = await spawn_server(self._args, self._service_root)

        self._receiver_task = asyncio.create_task(self._receiver())
        await self._wait_for_connected()
        await self.send_json_waiting_for_ack(
            {"type": "subscribe_warnings"},
            expected_message="subscribed to warnings",
        )
        await self.send_json_waiting_for_ack(
            {"type": "subscribe_arm_feedback"},
            expected_message="subscribed to arm feedback",
        )
        await self.send_json_waiting_for_ack(
            {"type": "subscribe_eef_feedback"},
            expected_message="subscribed to end-effector feedback",
        )
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        if self._receiver_task is not None:
            self._receiver_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._receiver_task

        if self._server_process is not None:
            self._server_process.terminate()
            with contextlib.suppress(ProcessLookupError):
                await self._server_process.wait()

    async def _wait_for_connected(self) -> None:
        deadline = time.monotonic() + self._args.connect_timeout
        while not self._connected.is_set():
            await self.send_json({"type": "hello", "access_mode": self._state.connection_mode})
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(
                    f"timed out waiting for {self._service_root}/events after "
                    f"{self._args.connect_timeout:.1f}s"
                )
            try:
                await asyncio.wait_for(self._connected.wait(), timeout=min(0.5, remaining))
            except asyncio.TimeoutError:
                continue

    async def send_json(self, payload: dict[str, Any]) -> None:
        payload_bytes = json.dumps(payload).encode("utf-8")
        if len(payload_bytes) > self._args.max_message_size:
            raise ValueError(
                f"payload of {len(payload_bytes)} bytes exceeds max message size "
                f"{self._args.max_message_size}"
            )

        update_connections = getattr(self._request_publisher, "update_connections", None)
        if callable(update_connections):
            update_connections()

        sample_uninit = self._request_publisher.loan_slice_uninit(len(payload_bytes))
        payload_slice = sample_uninit.payload()
        for index, value in enumerate(payload_bytes):
            payload_slice[index] = ctypes.c_uint8(value)
        sample = sample_uninit.assume_init()
        sample.send()
        await asyncio.sleep(0)

    async def send_json_waiting_for_ack(
        self, payload: dict[str, Any], *, expected_message: str
    ) -> None:
        self._pending_ack = expected_message
        self._pending_ack_event.clear()
        try:
            await self.send_json(payload)
            await asyncio.wait_for(
                self._pending_ack_event.wait(), timeout=self._args.connect_timeout
            )
        except asyncio.TimeoutError as exc:
            raise TimeoutError(
                f"timed out waiting for ack `{expected_message}` on "
                f"{self._service_root}/events"
            ) from exc
        finally:
            self._pending_ack = None
            self._pending_ack_event.clear()

    async def _receiver(self) -> None:
        poll_delay = max(self._args.poll_ms / 1000.0, 0.01)
        try:
            while True:
                sample = self._event_subscriber.receive()
                if sample is None:
                    await asyncio.sleep(poll_delay)
                    continue

                try:
                    payload = json.loads(self._sample_bytes(sample).decode("utf-8"))
                finally:
                    with contextlib.suppress(Exception):
                        sample.delete()

                self._handle_payload(payload)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self._state.last_error = str(exc)
            self._state.warnings.appendleft(f"receiver error: {exc}")

    def _handle_payload(self, payload: dict[str, Any]) -> None:
        message_type = payload.get("type")

        if message_type == "connected":
            info = payload.get("info", {})
            self._state.connection_mode = str(
                payload.get("connection_mode", self._state.connection_mode)
            )
            self._state.control_allowed = bool(
                payload.get("control_allowed", self._state.control_allowed)
            )
            self._state.interface = str(info.get("interface", self._state.interface))
            self._state.mounted_eef = mounted_eef_label(info.get("mounted_eef", self._state.mounted_eef))
            gravity = info.get("gravity_coefficients", self._state.gravity_coefficients)
            if isinstance(gravity, list) and gravity:
                self._state.gravity_coefficients = [float(value) for value in gravity]
            self._state.last_ack = "Connected"
            self._connected.set()
        elif message_type == "ack":
            message = str(payload.get("message", ""))
            self._state.last_ack = message
            if message == self._pending_ack:
                self._pending_ack_event.set()
        elif message_type == "error":
            self._state.last_error = str(payload.get("message", ""))
        elif message_type == "warning":
            warning = payload.get("warning", {})
            self._state.warnings.appendleft(str(warning.get("message", warning)))
        elif message_type == "arm_feedback":
            feedback = payload.get("feedback", {})
            positions = feedback.get("positions", [])
            velocities = feedback.get("velocities", [])
            efforts = feedback.get("torques", [])
            if len(positions) == 6 and len(velocities) == 6 and len(efforts) == 6:
                self._state.current_positions = [float(value) for value in positions]
                self._state.current_velocities = [float(value) for value in velocities]
                self._state.current_efforts = [float(value) for value in efforts]
                if not self._state.target_initialized:
                    self._state.measured_target_seed()
        elif message_type == "eef_feedback":
            feedback = payload.get("feedback")
            if isinstance(feedback, dict):
                self._state.last_eef_feedback = feedback
        elif message_type == "joint_target_accepted":
            target = payload.get("target", {})
            positions = target.get("positions", [])
            if len(positions) == 6:
                self._state.target_positions = [float(value) for value in positions]
                self._state.target_initialized = True
            self._state.last_ack = "Joint target accepted"

    @staticmethod
    def _sample_bytes(sample) -> bytes:
        payload = sample.payload()
        return ctypes.string_at(payload.as_ptr(), payload.len())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="AIRBOT Play keyboard teleop/status UI over iceoryx2"
    )
    parser.add_argument(
        "--service-root",
        help=(
            "iceoryx2 service root. Defaults to "
            f"{DEFAULT_SERVICE_ROOT_TEMPLATE.format(interface=DEFAULT_INTERFACE)}"
        ),
    )
    parser.add_argument(
        "--spawn-server",
        action="store_true",
        help="Spawn `cargo run --bin airbot-play-iceoryx2` automatically before connecting.",
    )
    parser.add_argument(
        "--interface",
        default=DEFAULT_INTERFACE,
        help=f"CAN interface to pass to the spawned transport (default: {DEFAULT_INTERFACE})",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=DEFAULT_CONNECT_TIMEOUT_S,
        help=(
            "Maximum time in seconds to wait for a `connected` event after sending `hello` "
            f"(default: {DEFAULT_CONNECT_TIMEOUT_S})"
        ),
    )
    parser.add_argument(
        "--poll-ms",
        type=int,
        default=DEFAULT_POLL_MS,
        help=f"Polling interval in milliseconds for event reads (default: {DEFAULT_POLL_MS})",
    )
    parser.add_argument(
        "--max-message-size",
        type=int,
        default=DEFAULT_MAX_MESSAGE_SIZE,
        help=(
            "Maximum JSON payload size for request publishing "
            f"(default: {DEFAULT_MAX_MESSAGE_SIZE})"
        ),
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
        help="Send `hello` with readonly access_mode.",
    )
    return parser.parse_args()


def effective_service_root(args: argparse.Namespace) -> str:
    if args.service_root:
        return str(args.service_root)
    return DEFAULT_SERVICE_ROOT_TEMPLATE.format(interface=args.interface)


async def spawn_server(
    args: argparse.Namespace, service_root: str
) -> asyncio.subprocess.Process:
    cmd = [
        "cargo",
        "run",
        "--bin",
        "airbot-play-iceoryx2",
        "--",
        "--interface",
        args.interface,
        "--service-root",
        service_root,
        "--poll-ms",
        str(args.poll_ms),
        "--max-message-size",
        str(args.max_message_size),
    ]
    if args.readonly:
        cmd.append("--allow-control=false")

    return await asyncio.create_subprocess_exec(
        *cmd,
        stdout=asyncio.subprocess.DEVNULL,
        stderr=asyncio.subprocess.DEVNULL,
    )


def mounted_eef_label(value: Any) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, dict) and len(value) == 1:
        key, inner = next(iter(value.items()))
        return f"{key}({inner})"
    return str(value)


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
    lines.append("AIRBOT Play Keyboard Control (iceoryx2)")
    lines.append(
        f"Interface: {state.interface} | Mounted EEF: {state.mounted_eef} | "
        f"Mode: {state.connection_mode} | Control allowed: {state.control_allowed}"
    )
    lines.append(
        f"Arm state: {state.arm_state} | Selected joint: J{state.selected_joint + 1} | "
        f"Step: {state.step_size:.4f} rad"
    )
    lines.append("")
    lines.append("Joint | Measured pos | Measured vel | Measured eff | Target pos")
    lines.append("------+--------------+--------------+--------------+-----------")

    for index in range(6):
        marker = ">" if index == state.selected_joint else " "
        lines.append(
            f"{marker} J{index + 1} | {fmt_number(state.current_positions[index])} | "
            f"{fmt_number(state.current_velocities[index])} | "
            f"{fmt_number(state.current_efforts[index])} | "
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
    lines.append(
        "Gravity coefficients: "
        + ", ".join(f"{value:.3f}" for value in state.gravity_coefficients)
    )
    lines.append("")
    lines.append("Last ack   : " + (state.last_ack or "<none>"))
    lines.append("Last error : " + (state.last_error or "<none>"))
    lines.append("Warnings:")
    if state.warnings:
        lines.extend(f"  - {warning}" for warning in list(state.warnings)[:6])
    else:
        lines.append("  - <none>")
    lines.append("")
    lines.append(
        "Keys: 1..6 select joint | h/l move | H/L move x5 | [/] step | "
        "f free | c follow | x disable | r latch | s send | q quit"
    )

    sys.stdout.write("\x1b[H\x1b[2J")
    sys.stdout.write("\n".join(lines))
    sys.stdout.flush()


async def set_arm_state(
    session: Iceoryx2Session, state: UiState, new_state: str
) -> None:
    await session.send_json({"type": "set_arm_state", "state": new_state})
    state.arm_state = new_state
    if new_state == "command_following" and not state.target_initialized:
        state.measured_target_seed()


async def send_joint_target(session: Iceoryx2Session, state: UiState) -> None:
    if not state.target_initialized:
        state.measured_target_seed()
    await session.send_json(
        {"type": "submit_joint_target", "positions": state.target_positions}
    )


async def handle_key(
    session: Iceoryx2Session, state: UiState, key: str
) -> bool:
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
        state.measured_target_seed(force=True)
        state.last_ack = "Target buffer latched from measured positions"
        return True
    if key == "s":
        await send_joint_target(session, state)
        return True
    if key == "f":
        await set_arm_state(session, state, "free_drive")
        return True
    if key == "c":
        await set_arm_state(session, state, "command_following")
        return True
    if key == "x":
        await set_arm_state(session, state, "disabled")
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
        if state.arm_state == "command_following":
            state.measured_target_seed(force=True)
        elif not state.target_initialized:
            state.measured_target_seed()
        state.target_positions[state.selected_joint] += delta
        if state.arm_state == "command_following":
            await send_joint_target(session, state)
        else:
            state.last_ack = "Target updated locally; press c then s to send"
        return True

    return True


async def ui_loop(session: Iceoryx2Session, state: UiState) -> None:
    while True:
        render(state)
        key = try_read_key()
        if key is not None:
            should_continue = await handle_key(session, state, key)
            if not should_continue:
                break
        await asyncio.sleep(0.05)


async def main() -> None:
    args = parse_args()
    state = UiState(connection_mode="readonly" if args.readonly else "control")

    async with Iceoryx2Session(args, state) as session:
        if args.start_state != "disabled":
            await set_arm_state(session, state, args.start_state)

        with RawTerminal():
            await ui_loop(session, state)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
