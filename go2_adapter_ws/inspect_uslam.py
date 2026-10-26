#!/usr/bin/env python3
"""Read local SLAM configuration and ROS metadata; never start robot services."""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess


KEYS = re.compile(
    r"loop|closure|relocal|localization|mapping|frontend|backend|frame|"
    r"extrinsic|transform|publish.*(?:map|cloud|tf)|(?:lid|imu|odom|cloud).*topic|"
    r"resolution|voxel|key.?frame|uslam|unitree.*slam|config|param", re.I)
PRIVATE = re.compile(r"password|passwd|secret|token|credential|private.?key", re.I)
TOPICS = (
    "/uslam/client_command", "/uslam/server_log",
    "/api/slam_operate/request", "/api/slam_operate/response", "/qt_command",
    "/utlidar/robot_odom", "/utlidar/cloud_deskewed",
    "/uslam/frontend/odom", "/uslam/frontend/cloud_world_ds",
    "/uslam/cloud_map", "/uslam/localization/odom",
    "/uslam/localization/cloud_world", "/tf", "/tf_static",
)


def command(args, seconds=5, limit=100):
    print("\n$ " + " ".join(args), flush=True)
    try:
        result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, errors="replace", timeout=seconds)
        output = result.stdout
        status = result.returncode
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        status = "timeout (no sample or query did not finish)"
    except OSError as exc:
        print(str(exc))
        return ""
    lines = output.splitlines()
    print("\n".join(lines[:limit]))
    if len(lines) > limit:
        print("[display truncated]")
    print("[status: %s]" % status)
    return output


def config_inventory(root):
    print("\nModule candidate: " + str(root))
    if not root.is_dir():
        print("Not present on THIS computer; it may be on the robot's other computer.")
        return
    print("Candidate directory only; process arguments establish which config is active.")
    count = 0
    for parent, dirs, names in os.walk(str(root), followlinks=False):
        depth = len(Path(parent).relative_to(root).parts)
        dirs[:] = sorted(d for d in dirs if d not in ("data", "log", "logs", ".git"))
        if depth >= 5:
            dirs[:] = []
        for name in sorted(names):
            path = Path(parent) / name
            if path.suffix.lower() not in (".yaml", ".yml", ".json", ".conf", ".ini", ".sh"):
                continue
            if path.is_symlink():
                continue
            count += 1
            if count > 60:
                print("[file limit reached]")
                return
            print("\nFile: " + str(path))
            try:
                if path.stat().st_size > 262144:
                    print("[content skipped: over 256 KiB]")
                    continue
                contents = path.read_text(errors="replace")
                if path.suffix.lower() == ".json":
                    def selected(value, prefix=""):
                        if isinstance(value, dict):
                            for key, child in value.items():
                                label = prefix + "." + str(key) if prefix else str(key)
                                if PRIVATE.search(str(key)):
                                    continue
                                if isinstance(child, (dict, list)):
                                    selected(child, label)
                                elif KEYS.search(label):
                                    print(label + " = " + repr(child))
                        elif isinstance(value, list):
                            for i, child in enumerate(value[:100]):
                                selected(child, prefix + "[%d]" % i)
                    selected(json.loads(contents))
                else:
                    for number, line in enumerate(contents.splitlines(), 1):
                        if KEYS.search(line) and not PRIVATE.search(line):
                            print("%d: %s" % (number, line[:500]))
            except (OSError, ValueError) as exc:
                print("[cannot read/parse: %s]" % exc)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module-dir", action="append", type=Path,
                        help="Actual module directory; repeat to inspect more than one.")
    parser.add_argument("--skip-ros", action="store_true", help="Inspect local files only.")
    args = parser.parse_args()
    print("USLAM configuration inventory (read-only)")
    print("ROS_DISTRO=" + os.environ.get("ROS_DISTRO", "unset"))
    print("No startup, shutdown, publication, parameter change or service call is performed.")
    # Process names/arguments only; do not inspect environments or shell history.
    processes = subprocess.run(["ps", "-eo", "pid=,comm=,args="],
                               stdout=subprocess.PIPE, text=True, check=False).stdout
    print("\nMatching SLAM processes:")
    matching = [line for line in processes.splitlines()
                if re.search(r"uslam_server|uslam_client|slam_client|/unitree/[^ ]*slam|unitree_slam/bin", line)
                and not PRIVATE.search(line)
                and "inspect_uslam.py" not in line]
    print("\n".join(matching) or "No matching process found on THIS computer.")
    roots = args.module_dir or [Path("/unitree/module/unitree_lidar_slam"),
                               Path("/unitree/module/unitree_slam")]
    for root in roots:
        config_inventory(root.resolve())
    if args.skip_ros:
        return
    if not shutil.which("ros2"):
        print("\nROS 2 CLI unavailable; source the existing Go2 Foxy/Unitree environment to inspect DDS.")
        return
    topics = command(["ros2", "topic", "list", "-t"], seconds=8)
    for topic in TOPICS:
        if not re.search(r"^" + re.escape(topic) + r"\s", topics, re.M):
            continue
        command(["ros2", "topic", "info", topic, "--verbose"], seconds=4)
        if (topic not in ("/tf", "/tf_static", "/uslam/client_command", "/qt_command")
                and not topic.startswith("/api/")):
            # Bounded sample. No arrays avoids printing pointcloud payloads.
            command(["ros2", "topic", "echo", topic, "--no-arr"], seconds=2, limit=65)
    nodes = command(["ros2", "node", "list"], seconds=8)
    names = sorted(set(line.strip() for line in nodes.splitlines()
                       if line.strip().startswith("/") and
                       re.search(r"uslam|unitree.*slam", line, re.I)))
    for name in names[:5]:
        command(["ros2", "param", "dump", name], seconds=4)
    if not names:
        print("\nNo matching ROS node. Embedded DDS services need not expose ROS parameter services.")


if __name__ == "__main__":
    main()
