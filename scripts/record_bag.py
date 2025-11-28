#!/usr/bin/env python3
"""
Simple RealSense D435/D435i recorder for Jetson Orin Nano (JetPack 6.2).

- Records color + depth streams into a .bag file
- Optional live preview using OpenCV
- CLI options for resolution, fps, duration, and output path
"""

import argparse
import datetime
import os
import sys
import time
import cv2
import pyrealsense2 as rs

def parse_args():
    parser = argparse.ArgumentParser(
        description="Record RealSense D435/D435i streams to a .bag file"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="recordings",
        help="Directory where .bag files will be saved",
    )
    parser.add_argument(
        "--basename",
        type=str,
        default="d435",
        help="Base name for the output file with timestamp",
    )
    parser.add_argument(
        "--width", type=int, default=640, help="Stream width"
    )
    parser.add_argument(
        "--height", type=int, default=480, help="Stream height"
    )
    parser.add_argument(
        "--fps", type=int, default=30, help="Stream fps"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Recording duration in seconds",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Disable OpenCV live preview window",
    )
    return parser.parse_args()


def build_output_path(output_dir: str, basename: str) -> str:
    os.makedirs(output_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = f"{basename}_{ts}.bag"
    return os.path.join(output_dir, fname)

def main():
    args = parse_args()
    out_path = build_output_path(args.output_dir, args.basename)
    print(f"[INFO] Saving recording to: {out_path}")

    # Configure the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    if args.device-serial:
        print(f"[INFO] Using device with serial: {args.device_serial}")
        config.enable_device(args.device_serial)

    # Enable recording to file
    config.enable_record_to_file(out_path)

    # Enable color & depth streams
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )
    config.enable_stream(
        rs.stream.depth, args.width, args.height, rs.format.z16, args.fps
    )

    try:
        print("[INFO] Starting pipeline...")
        pipeline_profile = pipeline.start(config)
    except Exception as e:
        print(f"[ERROR] Failed to start RealSense pipeline: {e}")
        sys.exit(1)

    # Optional: create preview window
    show_preview = not args.no_preview
    if show_preview:
        cv2.namedWindow("D435 Preview (color)", cv2.WINDOW_NORMAL)

    start_time = time.time()
    frame_count = 0

    print("[INFO] Recording... Press Ctrl+C to stop.")
    try:
        while True:
            if args.duration > 0 and (time.time() - start_time) >= args.duration:
                print("[INFO] Reached requested duration, stopping.")
                break

            frames = pipeline.wait_for_frames(5000)  # 5s timeout
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                print("[WARN] Incomplete frame set, skipping.")
                continue
            frame_count += 1
            if show_preview:
                color_image = color_frame.get_data()
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
                cv2.imshow("D435 Preview (color)", color_image)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    print("[INFO] ESC pressed, stopping.")
                    break

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C received, stopping recording.")
    except Exception as e:
        print(f"[ERROR] Exception during recording: {e}")
    finally:
        print("[INFO] Stopping pipeline...")
        pipeline.stop()
        if show_preview:
            cv2.destroyAllWindows()
        print(f"[INFO] Finished. Total frames: {frame_count}")
        print(f"[INFO] Output file: {out_path}")


if __name__ == "__main__":
    main()
