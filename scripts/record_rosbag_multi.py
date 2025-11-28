#!/usr/bin/env python3
"""
Record multiple ROS 2 topics into a single rosbag2 bag file.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from typing import List

def parse_args():
    parser = argparse.ArgumentParser(
        description="Record multiple ROS 2 topics into a single rosbag2 bag."
    )
    parser.add_argument(
        "--bag-name",
        "-o",
        type=str,
        default="multi_cams_bag",
        help="Name of the output bag directory",
    )
    parser.add_argument(
        "--topics",
        nargs="+",
        default=None,
        help="Explicit list of topics to record",
    )
    parser.add_argument(
        "--match-suffix",
        nargs="+",
        default=None,
        help=(
            "Optionally auto-add topics whose names end with any of these suffixes. "
        ),
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Recording duration in seconds",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Record ALL topics (-a). Overrides --topics/--match-suffix.",
    )
    return parser.parse_args()


def get_ros2_topics() -> List[str]:
    """
    Query `ros2 topic list` and return a list of topic names.
    """
    try:
        # Capture stdout of `ros2 topic list`
        result = subprocess.run(
            ["ros2", "topic", "list"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except FileNotFoundError:
        print("[ERROR] `ros2` CLI not found. Are you in a ROS 2 environment?")
        return []
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to run `ros2 topic list`: {e.stderr}")
        return []

    topics = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    return topics


def match_topics_by_suffix(all_topics: List[str], suffixes: List[str]) -> List[str]:
    """
    Filter topics whose name ends with any of the given suffixes.
    """
    matched = []
    for t in all_topics:
        for suf in suffixes:
            if t.endswith(suf):
                matched.append(t)
                break
    # Remove duplicates while preserving order
    unique = []
    seen = set()
    for t in matched:
        if t not in seen:
            seen.add(t)
            unique.append(t)
    return unique


def build_ros2_bag_command(args, topics: List[str]) -> List[str]:
    """
    Build the `ros2 bag record` command.
    """
    cmd = ["ros2", "bag", "record", "-o", args.bag_name]

    if args.all:
        cmd.append("-a")
    else:
        if not topics:
            print("[ERROR] No topics selected and --all not set.")
            return []
        cmd.extend(topics)

    return cmd


def run_ros2_bag(cmd: List[str], duration: float):
    if not cmd:
        return

    print("[INFO] Running command:")
    print("       " + " ".join(cmd))
    print()

    # Start the ros2 bag process
    try:
        proc = subprocess.Popen(cmd)
    except FileNotFoundError:
        print("[ERROR] Failed to start `ros2 bag`. Is ROS 2 sourced?")
        return

    try:
        if duration > 0:
            print(f"[INFO] Recording for {duration} seconds...")
            start = time.time()
            while True:
                elapsed = time.time() - start
                if elapsed >= duration:
                    print("[INFO] Duration reached. Stopping ros2 bag...")
                    proc.send_signal(signal.SIGINT)
                    break
                # Allow Ctrl+C
                time.sleep(0.5)
            proc.wait()
        else:
            print("[INFO] Recording... Press Ctrl+C to stop.")
            # Just wait; KeyboardInterrupt handled below
            proc.wait()
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C received. Stopping ros2 bag...")
        try:
            proc.send_signal(signal.SIGINT)
        except Exception:
            pass
        proc.wait()

    print("[INFO] ros2 bag record finished.")


def main():
    args = parse_args()

    if args.all:
        print("[INFO] --all set. All topics will be recorded into one bag.")
        topics = []
    else:
        topics = args.topics[:] if args.topics else []

        if args.match_suffix:
            all_topics = get_ros2_topics()
            auto_topics = match_topics_by_suffix(all_topics, args.match_suffix)
            print("[INFO] Auto-matched topics by suffix:")
            for t in auto_topics:
                print(f"  {t}")
            topics.extend(auto_topics)

        # De-duplicate explicit + auto topics
        unique = []
        seen = set()
        for t in topics:
            if t not in seen:
                seen.add(t)
                unique.append(t)
        topics = unique

        print("[INFO] Final topic list to record:")
        for t in topics:
            print(f"  {t}")
        print()

    # Ensure output directory does not already exist unless you want overwrite
    if os.path.exists(args.bag_name):
        print(f"[WARN] Output bag directory '{args.bag_name}' already exists.")
        print("       ros2 bag will append a suffix (e.g. _0) to avoid overwrite.")

    cmd = build_ros2_bag_command(args, topics)
    if not cmd:
        sys.exit(1)

    run_ros2_bag(cmd, args.duration)


if __name__ == "__main__":
    main()
