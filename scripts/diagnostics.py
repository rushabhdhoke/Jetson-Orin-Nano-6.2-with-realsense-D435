#!/usr/bin/env python3
"""
Basic diagnostics for Intel RealSense D435/D435i on Jetson Orin Nano.

- Lists connected RealSense devices and their serial numbers.
- Checks USB type and product line.
- Optionally runs a short streaming test to verify FPS.
"""

import argparse
import sys
import time
import pyrealsense2 as rs

def parse_args():
    parser = argparse.ArgumentParser(
        description="Run basic diagnostics for RealSense D435/D435i."
    )
    parser.add_argument(
        "--stream-test",
        action="store_true",
        help="Run a short streaming test and report approximate FPS.",
    )
    parser.add_argument(
        "--test-duration",
        type=float,
        default=5.0,
        help="Streaming test duration in seconds (default: 5.0)",
    )
    parser.add_argument(
        "--width", type=int, default=640, help="Test stream width"
    )
    parser.add_argument(
        "--height", type=int, default=480, help="Test stream height "
    )
    parser.add_argument(
        "--fps", type=int, default=30, help="Test stream fps"
    )
    parser.add_argument(
        "--device-serial",
        type=str,
        default=None,
        help="Optional RealSense device serial number to target for stream test.",
    )
    return parser.parse_args()


def list_devices():
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("[ERROR] No RealSense devices found.")
        return []

    print(f"[INFO] Found {len(devices)} RealSense device(s):")
    for i, dev in enumerate(devices):
        name = dev.get_info(rs.camera_info.name)
        serial = dev.get_info(rs.camera_info.serial_number)
        product_line = dev.get_info(rs.camera_info.product_line)
        usb_type = dev.get_info(rs.camera_info.usb_type_descriptor)
        fw = dev.get_info(rs.camera_info.firmware_version)
        print(f"  [{i}] Name:         {name}")
        print(f"      Serial:       {serial}")
        print(f"      Product line: {product_line}")
        print(f"      USB type:     {usb_type}")
        print(f"      Firmware:     {fw}")
    return devices


def run_stream_test(args):
    print("\n[INFO] Running streaming test...")
    pipeline = rs.pipeline()
    config = rs.config()

    if args.device_serial:
        print(f"[INFO] Using device with serial: {args.device_serial}")
        config.enable_device(args.device_serial)
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )
    config.enable_stream(
        rs.stream.depth, args.width, args.height, rs.format.z16, args.fps
    )

    try:
        profile = pipeline.start(config)
    except Exception as e:
        print(f"[ERROR] Failed to start streaming pipeline: {e}")
        return

    print(
        f"[INFO] Streaming color+depth at {args.width}x{args.height}@{args.fps} for {args.test_duration} s"
    )
    start_time = time.time()
    frame_count = 0
    try:
        while True:
            if (time.time() - start_time) >= args.test_duration:
                break
            try:
                frames = pipeline.wait_for_frames(5000)
            except Exception as e:
                print(f"[WARN] Frame timeout or error: {e}")
                break

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                print("[WARN] Incomplete frame set, skipping.")
                continue
            frame_count += 1
    finally:
        pipeline.stop()
    elapsed = time.time() - start_time
    if elapsed > 0:
        fps_est = frame_count / elapsed
        print(
            f"[INFO] Stream test finished. Frames: {frame_count}, Time: {elapsed:.2f} s, Approx FPS: {fps_est:.2f}"
        )
    else:
        print("[INFO] Stream test finished but elapsed time was ~0 (unexpected).")


def main():
    args = parse_args()
    devices = list_devices()
    if len(devices) == 0:
        sys.exit(1)
    if args.stream_test:
        run_stream_test(args)

if __name__ == "__main__":
    main()
