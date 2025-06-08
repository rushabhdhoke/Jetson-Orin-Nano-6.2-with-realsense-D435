# Below Code turn on the camera and start's recording, and recording is stopped and saved when Ctrl+C is pressed. 
# Similar functionality to running rs-record but this is Python based local setup which can be modiefied, compared to Realsense official commands which are In C++ and cannot be edited.

#!/usr/bin/env python3
import pyrealsense2 as rs
import signal

# Global flag that tells the recording loop to keep going.
keep_recording = True

def stop_recording(signal_number, frame):
    """Signal handler: when Ctrl+C is pressed, set the flag to False."""
    global keep_recording
    keep_recording = False

def main():
    # Wire up Ctrl+C (SIGINT) to our stop_recording() function.
    signal.signal(signal.SIGINT, stop_recording)

    # 1) Set up a RealSense pipeline and its configuration.
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable color and depth streams both at 640×480 @ 30 FPS.
    config.enable_stream(rs.stream.color,  640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth,  640, 480, rs.format.z16,  30)

    # 2) Tell the pipeline to record into a file named “output.bag”.
    config.enable_record_to_file("output.bag")

    # 3) Start streaming. This also starts writing frames into output.bag.
    pipeline.start(config)
    print("Recording started. Press Ctrl+C to stop.")

    try:
        # 4) Loop until keep_recording becomes False (Ctrl+C).
        while keep_recording:
            # This blocks until the next set of (color+depth) frames arrives,
            # and the RealSense SDK automatically writes them to output.bag.
            frames = pipeline.wait_for_frames()
            # (If you wish to process or show frames, you could grab them here:
            #   color = frames.get_color_frame()
            #   depth = frames.get_depth_frame()
            #   …)
    except Exception as err:
        print("Streaming error:", err)
    finally:
        # 5) Once the loop exits (e.g. Ctrl+C), stop the pipeline cleanly.
        pipeline.stop()
        print("Recording stopped. Saved to 'output.bag'.")

if __name__ == "__main__":
    main()
