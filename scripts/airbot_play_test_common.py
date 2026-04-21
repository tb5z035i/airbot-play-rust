from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import math
import sys
import time
import uuid
from collections import deque
from dataclasses import dataclass
from typing import Any

import websockets


DEFAULT_URL = "ws://127.0.0.1:9002"
DEFAULT_BIND = "127.0.0.1:9002"
DEFAULT_INTERFACE = "can0"


@dataclass(frozen=True)
class Pose:
    translation: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "Pose":
        translation = payload.get("translation", [])
        rotation_xyzw = payload.get("rotation_xyzw", [])
        if len(translation) != 3 or len(rotation_xyzw) != 4:
            raise ValueError(f"invalid pose payload: {payload!r}")
        return cls(
            translation=tuple(float(value) for value in translation),
            rotation_xyzw=tuple(float(value) for value in rotation_xyzw),
        )

    def to_payload(self) -> dict[str, list[float]]:
        return {
            "translation": list(self.translation),
            "rotation_xyzw": list(self.rotation_xyzw),
        }


@dataclass(frozen=True)
class ArmFeedback:
    positions: list[float]
    velocities: list[float]
    torques: list[float]
    valid: bool
    timestamp_micros: int

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "ArmFeedback":
        positions = payload.get("positions", [])
        velocities = payload.get("velocities", [])
        torques = payload.get("torques", [])
        if len(positions) != 6 or len(velocities) != 6 or len(torques) != 6:
            raise ValueError(f"invalid arm feedback payload: {payload!r}")
        return cls(
            positions=[float(value) for value in positions],
            velocities=[float(value) for value in velocities],
            torques=[float(value) for value in torques],
            valid=bool(payload.get("valid", False)),
            timestamp_micros=int(payload.get("timestamp_micros", 0)),
        )


def add_connection_args(parser: argparse.ArgumentParser) -> None:
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
        "--connect-timeout",
        type=float,
        default=30.0,
        help="Maximum time in seconds to wait for the websocket server to become reachable.",
    )
    parser.add_argument(
        "--feedback-timeout",
        type=float,
        default=5.0,
        help="Maximum time in seconds to wait for arm feedback updates.",
    )


async def connect_with_retry(url: str, timeout_s: float) -> Any:
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            return await websockets.connect(url)
        except OSError:
            if time.monotonic() >= deadline:
                raise
            await asyncio.sleep(0.5)


async def sleep_until(deadline: float) -> None:
    delay = deadline - time.perf_counter()
    if delay > 0.0:
        await asyncio.sleep(delay)


def lerp(start: float, end: float, alpha: float) -> float:
    return start + (end - start) * alpha


def lerp_vec3(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    alpha: float,
) -> tuple[float, float, float]:
    return tuple(lerp(a, b, alpha) for a, b in zip(start, end))


def format_vec(values: tuple[float, ...] | list[float]) -> str:
    return "[" + ", ".join(f"{value:+.4f}" for value in values) + "]"


class MotionSession:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.ws: Any | None = None
        self.server_process: asyncio.subprocess.Process | None = None
        self.receiver_task: asyncio.Task[None] | None = None
        self.connected_event = asyncio.Event()
        self.feedback_event = asyncio.Event()
        self.latest_feedback: ArmFeedback | None = None
        self.connected_info: dict[str, Any] = {}
        self.pending_requests: dict[str, asyncio.Future[Any]] = {}
        self.pending_ack: asyncio.Future[str] | None = None
        self.last_ack = ""
        self.background_errors: deque[str] = deque(maxlen=8)
        self.warnings: deque[str] = deque(maxlen=8)

    async def __aenter__(self) -> "MotionSession":
        if self.args.spawn_server:
            self.server_process = await asyncio.create_subprocess_exec(
                "cargo",
                "run",
                "--bin",
                "airbot-play-ws",
                "--",
                "--interface",
                self.args.interface,
                "--bind",
                self.args.bind,
                "--allow-control",
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
            )

        self.ws = await connect_with_retry(self.args.url, self.args.connect_timeout)
        self.receiver_task = asyncio.create_task(self._receiver())
        await self._send_json({"type": "hello", "access_mode": "control"})
        await self._send_json({"type": "subscribe_warnings"})
        await self._send_json({"type": "subscribe_arm_feedback"})
        await asyncio.wait_for(self.connected_event.wait(), timeout=self.args.connect_timeout)
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        if self.receiver_task is not None:
            self.receiver_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self.receiver_task

        if self.ws is not None:
            with contextlib.suppress(Exception):
                await self.ws.close()

        for future in self.pending_requests.values():
            if not future.done():
                future.cancel()
        self.pending_requests.clear()

        if self.server_process is not None:
            self.server_process.terminate()
            with contextlib.suppress(ProcessLookupError):
                await self.server_process.wait()

    async def _send_json(self, payload: dict[str, Any]) -> None:
        if self.ws is None:
            raise RuntimeError("websocket is not connected")
        await self.ws.send(json.dumps(payload))

    async def _receiver(self) -> None:
        try:
            assert self.ws is not None
            async for message in self.ws:
                payload = json.loads(message)
                message_type = payload.get("type")

                if message_type == "connected":
                    self.connected_info = dict(payload.get("info", {}))
                    self.connected_event.set()
                elif message_type == "arm_feedback":
                    self.latest_feedback = ArmFeedback.from_payload(payload.get("feedback", {}))
                    self.feedback_event.set()
                elif message_type == "ack":
                    message = str(payload.get("message", ""))
                    self.last_ack = message
                    if self.pending_ack is not None and not self.pending_ack.done():
                        self.pending_ack.set_result(message)
                        self.pending_ack = None
                elif message_type == "warning":
                    warning = payload.get("warning", {})
                    self.warnings.appendleft(str(warning.get("message", warning)))
                elif message_type == "current_pose":
                    request_id = payload.get("request_id")
                    future = self.pending_requests.pop(request_id, None)
                    if future is not None and not future.done():
                        future.set_result(Pose.from_payload(payload.get("pose", {})))
                elif message_type == "error":
                    self._handle_error(payload)
        finally:
            connection_error = RuntimeError("websocket connection closed")
            if self.pending_ack is not None and not self.pending_ack.done():
                self.pending_ack.set_exception(connection_error)
                self.pending_ack = None
            for future in self.pending_requests.values():
                if not future.done():
                    future.set_exception(connection_error)

    def _handle_error(self, payload: dict[str, Any]) -> None:
        message = str(payload.get("message", "unknown server error"))
        request_id = payload.get("request_id")

        if request_id is not None:
            future = self.pending_requests.pop(str(request_id), None)
            if future is not None and not future.done():
                future.set_exception(RuntimeError(message))
                return

        if self.pending_ack is not None and not self.pending_ack.done():
            self.pending_ack.set_exception(RuntimeError(message))
            self.pending_ack = None
            return

        self.background_errors.append(message)
        print(f"[server error] {message}", file=sys.stderr)

    def require_feedback(self) -> ArmFeedback:
        if self.latest_feedback is None:
            raise RuntimeError("arm feedback has not been received yet")
        return self.latest_feedback

    async def wait_for_feedback(self, timeout: float | None = None) -> ArmFeedback:
        if self.latest_feedback is not None:
            return self.latest_feedback
        timeout = self.args.feedback_timeout if timeout is None else timeout
        try:
            await asyncio.wait_for(self.feedback_event.wait(), timeout=timeout)
        except TimeoutError as exc:
            raise TimeoutError(
                f"timed out waiting for arm feedback after {timeout:.1f}s; "
                "the arm may still be disabled or robot feedback is not arriving"
            ) from exc
        return self.require_feedback()

    async def ensure_feedback_stream(
        self,
        initial_state: str = "free_drive",
        timeout: float | None = None,
    ) -> ArmFeedback:
        if self.latest_feedback is not None:
            return self.latest_feedback
        await self.set_arm_state(initial_state)
        return await self.wait_for_feedback(timeout=timeout)

    def raise_if_background_error(self) -> None:
        if self.background_errors:
            raise RuntimeError(self.background_errors.popleft())

    async def set_arm_state(self, state: str, timeout: float = 5.0) -> str:
        if self.pending_ack is not None and not self.pending_ack.done():
            raise RuntimeError("another state transition is already waiting for an acknowledgement")

        loop = asyncio.get_running_loop()
        pending_ack: asyncio.Future[str] = loop.create_future()
        self.pending_ack = pending_ack
        await self._send_json({"type": "set_arm_state", "state": state})
        try:
            message = await asyncio.wait_for(pending_ack, timeout=timeout)
        finally:
            if self.pending_ack is pending_ack:
                self.pending_ack = None
        self.last_ack = message
        return message

    async def submit_joint_target(self, positions: list[float]) -> None:
        if len(positions) != 6:
            raise ValueError(f"expected 6 joint targets, got {len(positions)}")
        await self._send_json({"type": "submit_joint_target", "positions": positions})
        await asyncio.sleep(0)
        self.raise_if_background_error()

    async def submit_task_target(self, pose: Pose) -> None:
        await self._send_json({"type": "submit_task_target", "pose": pose.to_payload()})
        await asyncio.sleep(0)
        self.raise_if_background_error()

    async def query_current_pose(self, timeout: float = 5.0) -> Pose:
        request_id = f"pose-{uuid.uuid4().hex}"
        loop = asyncio.get_running_loop()
        future: asyncio.Future[Pose] = loop.create_future()
        self.pending_requests[request_id] = future
        await self._send_json({"type": "query_current_pose", "request_id": request_id})
        try:
            return await asyncio.wait_for(future, timeout=timeout)
        finally:
            self.pending_requests.pop(request_id, None)

    async def set_exit_state(self, state: str) -> None:
        await self.set_arm_state(state)

    def print_warnings(self) -> None:
        if not self.warnings:
            return
        print("Warnings seen during the run:")
        for warning in self.warnings:
            print(f"  - {warning}")

    def translation_error_norm(self, measured: Pose, target: Pose) -> float:
        return math.sqrt(
            sum((measured.translation[index] - target.translation[index]) ** 2 for index in range(3))
        )
