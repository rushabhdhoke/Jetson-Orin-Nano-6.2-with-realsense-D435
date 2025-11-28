#!/usr/bin/env python3
"""
Live preview for Intel RealSense D435/D435i on Jetson Orin Nano.

- Displays color and depth streams side-by-side using OpenCV.
- Useful to quickly verify camera, exposure, and resolution settings.
"""

import argparse
import sys
import cv2
import numpy as np
import pyrealsense2 as rs

def parse_args():
    parser = argparse.ArgumentParser(
        description="Live preview of RealSense D435/D435i color and depth streams."
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
        "--device-serial",
        type=str,
        default=None,
        help="Optional RealSense device serial number to target",
    )
    parser.add_argument(
        "--no-depth",
        action="store_true",
        help="Disable depth visualization",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    pipeline = rs.pipeline()
    config = rs.config()

    if args.device_serial:
        print(f"[INFO] Using device with serial: {args.device_serial}")
        config.enable_device(args.device_serial)

    # Enable streams
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )
    if not args.no_depth:
        config.enable_stream(
            rs.stream.depth, args.width, args.height, rs.format.z16, args.fps
        )

    try:
        print("[INFO] Starting RealSense pipeline...")
        pipeline_profile = pipeline.start(config)
    except Exception as e:
        print(f"[ERROR] Failed to start RealSense pipeline: {e}")
        sys.exit(1)

    depth_sensor = None
    colorizer = rs.colorizer()  # For depth visualization

    if not args.no_depth:
        depth_sensor = pipeline_profile.get_device().first_depth_sensor()
    window_name = "D435 Preview (color | depth)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    print("[INFO] Press 'q' or ESC to quit.")
    try:
        while True:
            frames = pipeline.wait_for_frames(5000)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame() if not args.no_depth else None
            if not color_frame:
                print("[WARN] No color frame, skipping.")
                continue
            color_image = np.asanyarray(color_frame.get_data())

            if depth_frame is not None:
                # Colorize depth for visualization
                depth_color_frame = colorizer.colorize(depth_frame)
                depth_color_image = np.asanyarray(depth_color_frame.get_data())
                # Resize to match for side-by-side layout if necessary
                if depth_color_image.shape[:2] != color_image.shape[:2]:
                    depth_color_image = cv2.resize(
                        depth_color_image,
                        (color_image.shape[1], color_image.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )
                combined = np.hstack((color_image, depth_color_image))
            else:
                combined = color_image

            cv2.imshow(window_name, combined)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                print("[INFO] Exit key pressed, quitting.")
                break

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C received, quitting.")
    except Exception as e:
        print(f"[ERROR] Exception during preview: {e}")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
if __name__ == "__main__":
    main()
