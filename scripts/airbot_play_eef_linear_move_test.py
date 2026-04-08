# /// script
# dependencies = [
#   "websockets>=15.0",
# ]
# ///

"""Cartesian end-effector linear move test over the AIRBOT Play websocket interface.

This test:
1. Queries the current end-effector pose.
2. Preserves the current end-effector orientation.
3. Moves linearly from the current pose to `(0.20, 0.0, 0.30)`.
4. Moves linearly from `(0.20, 0.0, 0.30)` to `(0.20, 0.0, 0.45)`.
5. Holds the final pose briefly, then prints the final translation error.

Usage example:

    uv run scripts/airbot_play_eef_linear_move_test.py --spawn-server --interface can0
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import time

from airbot_play_test_common import (
    MotionSession,
    Pose,
    add_connection_args,
    format_vec,
    lerp_vec3,
    sleep_until,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="End-effector linear move test")
    add_connection_args(parser)
    parser.add_argument("--start-x", type=float, default=0.40, help="Start pose X in meters (default: 0.20)")
    parser.add_argument("--start-y", type=float, default=0.0, help="Start pose Y in meters (default: 0.0)")
    parser.add_argument("--start-z", type=float, default=0.40, help="Start pose Z in meters (default: 0.30)")
    parser.add_argument("--end-z", type=float, default=0.60, help="Final pose Z in meters (default: 0.45)")
    parser.add_argument(
        "--move-to-start-duration",
        type=float,
        default=3.0,
        help="Time in seconds for the current->start Cartesian ramp (default: 6.0)",
    )
    parser.add_argument(
        "--move-to-end-duration",
        type=float,
        default=1.0,
        help="Time in seconds for the start->end Cartesian ramp (default: 6.0)",
    )
    parser.add_argument(
        "--final-hold-duration",
        type=float,
        default=0.5,
        help="Time in seconds to keep reissuing the final Cartesian target (default: 2.0)",
    )
    parser.add_argument(
        "--command-rate",
        type=float,
        default=250.0,
        help="Task target command rate in Hz (default: 250.0)",
    )
    parser.add_argument(
        "--exit-state",
        choices=["free_drive", "disabled"],
        default="free_drive",
        help="Arm state to request before exiting (default: free_drive)",
    )
    return parser.parse_args()


async def run_pose_segment(
    session: MotionSession,
    start_pose: Pose,
    end_pose: Pose,
    duration_s: float,
    command_rate_hz: float,
) -> None:
    steps = max(1, int(round(max(duration_s, 0.0) * command_rate_hz)))
    period = 1.0 / command_rate_hz
    next_tick = time.perf_counter()

    for step in range(1, steps + 1):
        alpha = step / steps
        pose = Pose(
            translation=lerp_vec3(start_pose.translation, end_pose.translation, alpha),
            rotation_xyzw=end_pose.rotation_xyzw,
        )
        await session.submit_task_target(pose)
        next_tick += period
        await sleep_until(next_tick)


async def hold_pose(
    session: MotionSession,
    pose: Pose,
    duration_s: float,
    command_rate_hz: float,
) -> None:
    steps = max(1, int(round(max(duration_s, 0.0) * command_rate_hz)))
    period = 1.0 / command_rate_hz
    next_tick = time.perf_counter()

    for _ in range(steps):
        await session.submit_task_target(pose)
        next_tick += period
        await sleep_until(next_tick)


async def main() -> None:
    args = parse_args()

    async with MotionSession(args) as session:
        await session.ensure_feedback_stream()

        try:
            await session.set_arm_state("command_following")
            current_pose = await session.query_current_pose()
            start_pose = Pose(
                translation=(args.start_x, args.start_y, args.start_z),
                rotation_xyzw=current_pose.rotation_xyzw,
            )
            end_pose = Pose(
                translation=(args.start_x, args.start_y, args.end_z),
                rotation_xyzw=current_pose.rotation_xyzw,
            )

            print(f"Connected to {args.url}")
            print(f"Current pose translation : {format_vec(list(current_pose.translation))}")
            print(f"Current pose orientation : {format_vec(list(current_pose.rotation_xyzw))}")
            print(f"Stage 1 target translation: {format_vec(list(start_pose.translation))}")
            await run_pose_segment(
                session,
                start_pose=current_pose,
                end_pose=start_pose,
                duration_s=args.move_to_start_duration,
                command_rate_hz=args.command_rate,
            )

            print(f"Stage 2 target translation: {format_vec(list(end_pose.translation))}")
            await run_pose_segment(
                session,
                start_pose=start_pose,
                end_pose=end_pose,
                duration_s=args.move_to_end_duration,
                command_rate_hz=args.command_rate,
            )

            print(f"Holding final pose for {args.final_hold_duration:.2f} s")
            await hold_pose(
                session,
                pose=end_pose,
                duration_s=args.final_hold_duration,
                command_rate_hz=args.command_rate,
            )

            final_pose = await session.query_current_pose()
            translation_error = [
                final_pose.translation[index] - end_pose.translation[index]
                for index in range(3)
            ]
            print(f"Final pose translation  : {format_vec(list(final_pose.translation))}")
            print(f"Target translation      : {format_vec(list(end_pose.translation))}")
            print(f"Translation error       : {format_vec(translation_error)}")
            print(
                "Translation error norm  : "
                f"{session.translation_error_norm(final_pose, end_pose):.5f} m"
            )
            session.print_warnings()
        finally:
            with contextlib.suppress(Exception):
                await session.set_exit_state(args.exit_state)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
