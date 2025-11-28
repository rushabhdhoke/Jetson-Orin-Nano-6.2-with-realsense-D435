#!/usr/bin/env python3
"""
Record from multiple Intel RealSense cameras (e.g., D435/D435i) simultaneously.

- One pipeline per camera (identified by serial number).
- Each camera records to its own .bag file in the same output directory.
- Duration-based stop, or Ctrl+C.
"""

import argparse
import datetime
import os
import sys
import threading
import time
import pyrealsense2 as rs

def parse_args():
    parser = argparse.ArgumentParser(
        description="Record from multiple RealSense cameras simultaneously to .bag files."
    )
    parser.add_argument(
        "--serials",
        nargs="+",
        default=None,
        help=(
            "List of device serial numbers to record from. "
            "If omitted, all connected RealSense devices will be used."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="recordings_multi",
        help="Directory where .bag files will be saved",
    )
    parser.add_argument(
        "--basename",
        type=str,
        default="multi_d435",
        help="Base name for output files",
    )
    parser.add_argument(
        "--width", type=int, default=640, help="Stream width"
    )
    parser.add_argument(
        "--height", type=int, default=480, help="Stream height "
    )
    parser.add_argument(
        "--fps", type=int, default=30, help="Stream fps"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Recording duration in seconds for all cameras",
    )
    return parser.parse_args()


def discover_serials():
    """
    Discover all connected RealSense devices and return their serial numbers.
    """
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("[ERROR] No RealSense devices found.")
        return []

    serials = []
    print(f"[INFO] Found {len(devices)} RealSense device(s):")
    for i, dev in enumerate(devices):
        name = dev.get_info(rs.camera_info.name)
        serial = dev.get_info(rs.camera_info.serial_number)
        product_line = dev.get_info(rs.camera_info.product_line)
        print(f"  [{i}] Name: {name}, Serial: {serial}, Product line: {product_line}")
        serials.append(serial)

    return serials


def build_output_path(output_dir: str, basename: str, serial: str) -> str:
    os.makedirs(output_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = f"{basename}_{serial}_{ts}.bag"
    return os.path.join(output_dir, fname)


def record_worker(serial: str, args, stop_event: threading.Event, start_time: float):
    """
    Worker thread: start one pipeline for this serial and record to .bag.
    """
    out_path = build_output_path(args.output_dir, args.basename, serial)
    print(f"[{serial}] Output file: {out_path}")

    pipeline = rs.pipeline()
    config = rs.config()

    try:
        config.enable_device(serial)
    except Exception as e:
        print(f"[{serial}] [ERROR] Failed to bind to device: {e}")
        return

    # Tell device to record to file
    config.enable_record_to_file(out_path)

    # Enable color + depth
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )
    config.enable_stream(
        rs.stream.depth, args.width, args.height, rs.format.z16, args.fps
    )

    try:
        print(f"[{serial}] Starting pipeline...")
        pipeline.start(config)
    except Exception as e:
        print(f"[{serial}] [ERROR] Failed to start pipeline: {e}")
        return

    frame_count = 0
    print(f"[{serial}] Recording started.")

    try:
        while not stop_event.is_set():
            if args.duration > 0.0:
                elapsed = time.time() - start_time
                if elapsed >= args.duration:
                    # Signal all threads to stop
                    stop_event.set()
                    break

            try:
                # Blocking wait; recording continues in the device in the background
                frames = pipeline.wait_for_frames(5000)
            except Exception as e:
                print(f"[{serial}] [WARN] wait_for_frames failed or timeout: {e}")
                break

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                print(f"[{serial}] [WARN] Incomplete frame set, skipping.")
                continue
            frame_count += 1

    except Exception as e:
        print(f"[{serial}] [ERROR] Exception during record loop: {e}")
    finally:
        print(f"[{serial}] Stopping pipeline...")
        pipeline.stop()
        print(f"[{serial}] Finished. Total frames: {frame_count}")
        print(f"[{serial}] File saved to: {out_path}")


def main():
    args = parse_args()

    if args.serials is None or len(args.serials) == 0:
        # Auto-discover devices
        serials = discover_serials()
        if not serials:
            sys.exit(1)
    else:
        serials = args.serials
        print(f"[INFO] Using user-specified serials: {serials}")

    print(f"[INFO] Will record from {len(serials)} camera(s).")
    if args.duration > 0.0:
        print(f"[INFO] Target duration: {args.duration} seconds for all cameras.")
    else:
        print("[INFO] Duration: unlimited (Ctrl+C to stop).")

    stop_event = threading.Event()
    start_time = time.time()
    threads = []

    for serial in serials:
        t = threading.Thread(
            target=record_worker,
            args=(serial, args, stop_event, start_time),
            daemon=True,
        )
        t.start()
        threads.append(t)

    print("[INFO] Recording... Press Ctrl+C to stop.")

    try:
        # Wait for all threads to finish (duration or Ctrl+C)
        for t in threads:
            while t.is_alive():
                t.join(timeout=0.5)
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C received, stopping all cameras...")
        stop_event.set()
        for t in threads:
            t.join()
    print("[INFO] Multi-camera recording finished.")


if __name__ == "__main__":
    main()
