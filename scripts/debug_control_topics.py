#!/usr/bin/env python3
"""Utility to inspect publishers/subscribers for control-related topics.

Run this script from an environment where ROS 2 is sourced. It will invoke
`ros2 topic info --verbose` for each topic of interest and print the results in
a compact form so you can confirm which nodes are producing or consuming the
commands in the NMPC control chain.
"""

import shutil
import subprocess
from typing import List


# Topics that are directly involved in the NMPC → low-level control pipeline.
CONTROL_TOPICS: List[str] = [
    "/drone/control/waypoint_command",
    "/drone/control/attitude_command",
    "/drone/control/velocity_setpoint",
    "/drone/control/angular_velocity_setpoint",
    "/drone/control/waypoint_enable",
    "/drone/control/attitude_enable",
    "/drone/control/velocity_enable",
    "/X3/cmd_vel",
    "/X3/odometry",
    "/nmpc/enable",
]


def check_ros_cli() -> None:
    """Verify that the ros2 CLI is available before running any commands."""

    if shutil.which("ros2") is None:
        raise EnvironmentError(
            "The 'ros2' executable was not found in PATH. Make sure the ROS 2 "
            "environment is sourced before running this script."
        )


def topic_info(topic: str) -> str:
    """Return the verbose topic info for the given topic using ros2 CLI."""

    try:
        result = subprocess.run(
            ["ros2", "topic", "info", topic, "--verbose"],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise EnvironmentError(
            "Failed to execute 'ros2 topic info'. Ensure ROS 2 is sourced."
        ) from exc

    header = f"\n===== {topic} ====="
    if result.returncode != 0:
        body = result.stderr.strip() or "Unable to query topic."
    else:
        body = result.stdout.strip()

    return f"{header}\n{body}\n"


def main() -> None:
    check_ros_cli()

    print("Inspecting control-related ROS 2 topics...\n")
    for topic in CONTROL_TOPICS:
        print(topic_info(topic))


if __name__ == "__main__":
    main()
