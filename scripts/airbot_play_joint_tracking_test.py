# /// script
# dependencies = [
#   "websockets>=15.0",
# ]
# ///

"""Joint tracking test over the AIRBOT Play websocket interface.

This test:
1. Reads the current joint positions.
2. Moves the selected joint linearly from its current position to `--negative-target`.
3. Moves the selected joint linearly from `--negative-target` to `--positive-target`.
4. Keeps commanding `--positive-target` until the selected joint reaches the tolerance window.
5. Reports the average absolute tracking error and the reach time to the positive target.

Usage example:

    uv run scripts/airbot_play_joint_tracking_test.py --spawn-server --interface can0 --joint 3
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import time

from airbot_play_test_common import MotionSession, add_connection_args, format_vec, lerp, sleep_until


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Selected-joint command-following tracking test")
    add_connection_args(parser)
    parser.add_argument(
        "--joint",
        type=int,
        choices=[1, 2, 3, 4, 5, 6],
        default=1,
        help="Joint number to command and measure (default: 1)",
    )
    parser.add_argument(
        "--negative-target",
        type=float,
        default=-1.0,
        help="Intermediate target for the selected joint in radians (default: -1.0)",
    )
    parser.add_argument(
        "--positive-target",
        type=float,
        default=1.0,
        help="Final target for the selected joint in radians (default: 1.0)",
    )
    parser.add_argument(
        "--move-to-negative-duration",
        type=float,
        default=4.0,
        help="Time in seconds for the current->negative ramp (default: 4.0)",
    )
    parser.add_argument(
        "--move-to-positive-duration",
        type=float,
        default=1.0,
        help="Time in seconds for the negative->positive ramp (default: 1.0)",
    )
    parser.add_argument(
        "--command-rate",
        type=float,
        default=250.0,
        help="Joint target command rate in Hz (default: 250.0)",
    )
    parser.add_argument(
        "--reach-tolerance",
        type=float,
        default=0.002,
        help="Absolute position tolerance in radians for declaring the selected joint reached (default: 0.002)",
    )
    parser.add_argument(
        "--reach-timeout",
        type=float,
        default=8.0,
        help="Maximum time in seconds to wait for the selected joint to reach the final target (default: 8.0)",
    )
    parser.add_argument(
        "--exit-state",
        choices=["free_drive", "disabled"],
        default="free_drive",
        help="Arm state to request before exiting (default: free_drive)",
    )
    return parser.parse_args()


async def run_segment(
    session: MotionSession,
    joint_index: int,
    start_target: float,
    end_target: float,
    duration_s: float,
    command_rate_hz: float,
    error_samples: list[float],
) -> float:
    steps = max(1, int(round(max(duration_s, 0.0) * command_rate_hz)))
    period = 1.0 / command_rate_hz
    next_tick = time.perf_counter()
    issued_at = next_tick

    for step in range(1, steps + 1):
        alpha = step / steps
        target = lerp(start_target, end_target, alpha)
        feedback = session.require_feedback()
        positions = feedback.positions.copy()
        positions[joint_index] = target
        await session.submit_joint_target(positions)
        issued_at = time.perf_counter()
        error_samples.append(abs(target - feedback.positions[joint_index]))
        next_tick += period
        await sleep_until(next_tick)

    return issued_at


async def hold_final_target_until_reached(
    session: MotionSession,
    joint_index: int,
    target: float,
    command_rate_hz: float,
    tolerance: float,
    timeout_s: float,
    error_samples: list[float],
) -> tuple[float | None, float]:
    period = 1.0 / command_rate_hz
    next_tick = time.perf_counter()
    deadline = next_tick + timeout_s

    while time.perf_counter() <= deadline:
        feedback = session.require_feedback()
        positions = feedback.positions.copy()
        positions[joint_index] = target
        await session.submit_joint_target(positions)
        error = abs(target - feedback.positions[joint_index])
        error_samples.append(error)
        if error <= tolerance:
            return time.perf_counter(), feedback.positions[joint_index]
        next_tick += period
        await sleep_until(next_tick)

    return None, session.require_feedback().positions[joint_index]


async def main() -> None:
    args = parse_args()
    joint_index = args.joint - 1

    async with MotionSession(args) as session:
        await session.ensure_feedback_stream()
        print(f"Connected to {args.url}")
        print(f"Measured joints at start: {format_vec(session.require_feedback().positions)}")

        try:
            await session.set_arm_state("command_following")
            start_feedback = session.require_feedback()
            start_joint = start_feedback.positions[joint_index]
            print(f"Joint {args.joint} start position: {start_joint:+.4f} rad")
            print(
                f"Ramping joint {args.joint} to {args.negative_target:+.4f} rad, "
                f"then to {args.positive_target:+.4f} rad "
                f"at {args.command_rate:.1f} Hz"
            )

            error_samples: list[float] = []
            await run_segment(
                session,
                joint_index=joint_index,
                start_target=start_joint,
                end_target=args.negative_target,
                duration_s=args.move_to_negative_duration,
                command_rate_hz=args.command_rate,
                error_samples=error_samples,
            )
            final_target_issued_at = await run_segment(
                session,
                joint_index=joint_index,
                start_target=args.negative_target,
                end_target=args.positive_target,
                duration_s=args.move_to_positive_duration,
                command_rate_hz=args.command_rate,
                error_samples=error_samples,
            )

            reached_at, final_measured = await hold_final_target_until_reached(
                session,
                joint_index=joint_index,
                target=args.positive_target,
                command_rate_hz=args.command_rate,
                tolerance=args.reach_tolerance,
                timeout_s=args.reach_timeout,
                error_samples=error_samples,
            )

            average_error = sum(error_samples) / len(error_samples) if error_samples else 0.0
            print(f"Average abs(target - measured) for joint {args.joint}: {average_error:.5f} rad")
            if reached_at is None:
                print(
                    f"Reach time for joint {args.joint} to {args.positive_target:+.4f} rad "
                    f"within +/-{args.reach_tolerance:.4f} rad: "
                    "not reached before timeout"
                )
            else:
                print(
                    f"Reach time for joint {args.joint} to {args.positive_target:+.4f} rad "
                    f"within +/-{args.reach_tolerance:.4f} rad: "
                    f"{reached_at - final_target_issued_at:.3f} s"
                )
            print(f"Final measured joint {args.joint} position: {final_measured:+.5f} rad")
            session.print_warnings()
        finally:
            with contextlib.suppress(Exception):
                await session.set_exit_state(args.exit_state)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
