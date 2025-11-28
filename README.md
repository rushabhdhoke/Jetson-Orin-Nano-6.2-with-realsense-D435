# Jetson Orin Nano + RealSense D435/D435i Utilities

Python and ROS 2 tools for using Intel RealSense D435/D435i cameras on a Jetson Orin Nano (JetPack 6.2):

- Simple **non-ROS** recording and preview using `pyrealsense2`.
- **Multi-camera** recording (multiple devices at once).
- **ROS 2 (Jazzy)** workflow to record multiple camera topics into a single rosbag2.

> This repo assumes you already have Intel RealSense working on Jetson (librealsense, `pyrealsense2`, and/or the `realsense2_camera` ROS 2 driver).
> For Jetson-specific RealSense install guides, see the official librealsense + Jetson notes and community scripts (e.g., JetsonHacks).  

---

## 1. Repo Structure

```text
Jetson-Orin-Nano-6.2-with-realsense-D435/
├── scripts/
│   ├── record_bag.py           # Single-camera recording to .bag (SDK format)
│   ├── record_multi_bag.py     # Multi-camera (multiple devices) → one .bag per device
│   ├── preview.py              # Live color+depth preview
│   ├── play_bag.py             # Playback .bag files with OpenCV
│   └── diagnostics.py          # Device listing + basic stream test
│   └── record_rosbag_multi.py  # ROS 2: multi-camera topics → single rosbag2
├── jetson_setup/               # (Optional) Jetson/RealSense install helpers
│   └── install_realsense_jetpack6.sh
├── examples/
│   └── example_recording.md    # Example usage & screenshots (optional)
├── media/
│   ├── preview_rgb_depth.gif   # Preview screenshot/GIF (optional)
│   └── ...
├── README.md
└── LICENSE
```

## 2. Dependencies
System
- Jetson Orin Nano (JetPack 6.2 / Ubuntu 22.04).
- Intel RealSense D435 / D435i.
- Librealsense2 + pyrealsense2 installed and working.
- Optional: realsense2_camera ROS 2 driver for ROS workflows.


Python
Create a virtual environment (optional but recommended):
```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install opencv-python pyrealsense2
```

## 3. Non-ROS Mode (Direct pyrealsense2)
These tools use the RealSense SDK directly. They do not require ROS.

### 3.1 Single-Camera Recording – record_bag.py
Record color + depth from one D435/D435i into a .bag file (RealSense SDK format).
```bash
# 30-second recording with preview enabled
python scripts/record_bag.py --duration 30

# Tunable parameters
--width 848 --height 480 --fps 30 --duration 10 --device-serial <SERIAL> --duration 20
```

### 3.2 Live Preview – preview.py
Quickly verify that the camera, exposure, and depth stream are working
```bash
python scripts/preview.py

# Tunable parameters
--width 1280 --height 720 --fps 15
```

### 3.2 Playback .bag Files – play_bag.py
Visualize recorded .bag files:
```bash
python scripts/play_bag.py --input recordings/<date time.bag>

# Tunable paramters
--loop --fps 15
```

### 3.4 Diagnostics – diagnostics.py
This is useful to confirm that your USB bandwidth, drivers, and FPS are okay before recording longer sequences.

### 3.5 Multi-Camera Recording (Non-ROS) – record_multi_bag.py
Record from multiple RealSense devices simultaneously.
```bash
# Auto-detect all RealSense devices and record 30 seconds from each
python scripts/record_multi_bag.py --duration 30

# Custom resolution / fps / output directory
python scripts/record_multi_bag.py \
    --serials <SERIAL1> <SERIAL2> \
    --width 848 --height 480 --fps 30 \
    --output-dir recordings_multi \
    --basename myrun \
    --duration 20

```


## 4. ROS 2 Mode (Multi-Camera in One rosbag2)
If you are using ROS 2 Jazzy, you can run multiple camera drivers and record all their image topics into a single rosbag2 bag.
The script record_rosbag_multi.py is a thin wrapper around ros2 bag record:

```bash
# Example: launch multiple RealSense devices in ROS 2
ros2 launch realsense2_camera rs_multi_camera_launch.py
```

### 4.1 Record selected topics into one bag – record_rosbag_multi.py
Record specific topics into a single bag for 60 seconds
```bash
python scripts/record_rosbag_multi.py \
    --bag-name multi_cams_run1 \
    --topics /cam1/color/image_raw /cam2/color/image_raw \
    --duration 60
```

## 5. Notes / Limitations
RealSense SDK .bag vs ROS 2 rosbag2
- record_bag.py / record_multi_bag.py use the RealSense SDK recorder; each device writes its own .bag file.
- record_rosbag_multi.py uses ros2 bag record and saves all ROS 2 topics into a single rosbag2 dataset.
Make sure your storage (SSD) and USB bandwidth are sufficient for multi-camera high-resolution recording.
