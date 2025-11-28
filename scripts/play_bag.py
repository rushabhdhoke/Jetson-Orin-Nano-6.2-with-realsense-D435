#!/usr/bin/env python3
"""
Playback RealSense .bag recordings created by record_bag.py.

- Opens a .bag file.
- Displays color + depth side-by-side using OpenCV.
"""

import argparse
import os
import sys
import cv2
import numpy as np
import pyrealsense2 as rs


def parse_args():
    parser = argparse.ArgumentParser(
        description="Play back a RealSense .bag recording with color and depth preview."
    )
    parser.add_argument(
        "--input",
        "-i",
        type=str,
        required=True,
        help="Path to the .bag file to play.",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Loop playback indefinitely until 'q' or ESC is pressed.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=0.0,
        help="Target playback FPS (0 = as fast as frames arrive).",
    )
    return parser.parse_args()

def play_once(bag_path: str, fps: float, window_name: str) -> bool:
    """
    Play the bag file one time.
    Returns:
        True  -> playback finished normally
        False -> user requested exit (q/ESC) or error
    """
    if not os.path.exists(bag_path):
        print(f"[ERROR] Bag file not found: {bag_path}")
        return False

    pipeline = rs.pipeline()
    config = rs.config()

    try:
        rs.config.enable_device_from_file(config, bag_path, repeat_playback=False)
    except Exception as e:
        print(f"[ERROR] Failed to open bag file: {e}")
        return False

    try:
        profile = pipeline.start(config)
    except Exception as e:
        print(f"[ERROR] Failed to start playback pipeline: {e}")
        return False

    device = profile.get_device()
    playback = device.as_playback()
    playback.set_real_time(False)  # Let us control playback timing
    colorizer = rs.colorizer()

    delay_ms = 1
    if fps > 0:
        delay_ms = int(1000.0 / fps)

    print(f"[INFO] Playing back: {bag_path}")
    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(5000)
            except Exception:
                # End of file or timeout
                print("[INFO] End of recording reached.")
                break

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                print("[WARN] Incomplete frame set, skipping.")
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_color_frame = colorizer.colorize(depth_frame)
            depth_color_image = np.asanyarray(depth_color_frame.get_data())

            if depth_color_image.shape[:2] != color_image.shape[:2]:
                depth_color_image = cv2.resize(
                    depth_color_image,
                    (color_image.shape[1], color_image.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )

            combined = np.hstack((color_image, depth_color_image))
            cv2.imshow(window_name, combined)
            key = cv2.waitKey(delay_ms) & 0xFF
            if key in (27, ord("q")):
                print("[INFO] Exit key pressed, quitting playback.")
                pipeline.stop()
                cv2.destroyAllWindows()
                return False

    finally:
        pipeline.stop()
    return True


def main():
    args = parse_args()
    bag_path = args.input
    window_name = "D435 Bag Playback (color | depth)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    if args.loop:
        print("[INFO] Loop mode enabled.")
        while True:
            finished = play_once(bag_path, args.fps, window_name)
            if not finished:
                break
    else:
        play_once(bag_path, args.fps, window_name)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
