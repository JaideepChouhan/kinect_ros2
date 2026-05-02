# kinect_ros2: Native ROS 2 Jazzy Driver for Microsoft Kinect v1 (Xbox 360)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-orange)](https://docs.ros.org/en/jazzy)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-red)](https://ubuntu.com/)
[![Python](https://img.shields.io/badge/Python-3.12-green)](https://python.org)
[![Version](https://img.shields.io/badge/Version-2.0.0-brightgreen)](https://github.com/kinect_ros2)

A complete, production-ready ROS 2 Jazzy driver ecosystem for the Microsoft Kinect v1 (Xbox 360) sensor, engineered on the foundation of the OpenKinect `libfreenect` library. **Version 2.0** transforms the package from a pure driver into a learning and prototyping platform with six ready-to-run example nodes, a clean context-manager API, launch files, and integrated test suite.

## What's New in Version 2.0

| Feature | v1 | v2 |
| :--- | :--- | :--- |
| RGB & Depth Topics | ✅ | ✅ |
| Point Cloud (XYZ) | ✅ | ✅ |
| Colourised Point Cloud (XYZRGB) | ❌ | ✅ **NEW** |
| Depth Histogram Visualizer | ❌ | ✅ **NEW** |
| Hand/Blob Proximity Detector | ❌ | ✅ **NEW** |
| Obstacle Avoidance Publisher | ❌ | ✅ **NEW** |
| Context-Manager Device API (`KinectDevice`) | ❌ | ✅ **NEW** |
| Six Example Nodes | ❌ | ✅ **NEW** |
| Automated Test Suite (pytest) | ❌ | ✅ **NEW** |
| Per-Example Launch Files | ❌ | ✅ **NEW** |
| All-Examples-in-One Launch | ❌ | ✅ **NEW** |

**Key Improvements:**
- **Clean API:** New `KinectDevice` context manager wraps libfreenect for easier direct hardware access
- **Example-Driven Learning:** Six fully-documented example nodes covering image annotation, distance measurement, histograms, hand detection, sensor fusion, and obstacle avoidance
- **Shared Parameters:** Centralized `examples_params.yaml` for consistent configuration across all examples
- **Launch Orchestration:** `kinect_all_examples.launch.py` starts all nodes with staggered timing to avoid CPU spikes
- **Testing:** Unit and integration tests validate core functionality

## Core Changes in Version 2

### New: KinectDevice Context Manager

**Location:** `kinect_ros2/kinect_device.py`

A Python context manager that wraps libfreenect synchronous calls for safer hardware access. Handles device initialization, LED control, tilt motor, and cleanup.

**Key Methods:**
- `__enter__() / __exit__()` – Sets LED color, safely closes device
- `get_video()` → `(RGB frame, timestamp)` – Raw RGB data
- `get_depth()` → `(raw disparity, timestamp)` – 11-bit disparity
- `get_depth_meters()` → `(float32 depth in meters, timestamp)` – Metric depth
- `get_accel()` → `(ax, ay, az) in g` – Accelerometer telemetry
- `set_tilt(degrees)` – Tilt motor control (clamped to [-31, 31])

**Usage Example:**
```python
from kinect_ros2.kinect_device import KinectDevice

with KinectDevice(device_index=0, led_color='green') as dev:
    rgb, _ = dev.get_video()
    depth_m, _ = dev.get_depth_meters()
    dev.set_tilt(10.0)
```

**Important:** In v2, only the driver node (`kinect_driver_node`) should use `KinectDevice`. All example nodes subscribe to driver topics instead of accessing hardware directly – this avoids `LIBUSB_ERROR_BUSY`.

### Enhanced: depth_utils.py

Expanded utility module for depth conversion and processing:

**Key Functions:**
- `raw_to_meters(raw, c1, c2)` – Converts uint16 disparity to metres
- `make_camera_info_dict(...)` – Builds CameraInfo message fields
- `nearest_in_roi(depth_m, roi_fraction, min_valid)` – Finds closest object in ROI

**Constants (Kinect v1):**
```python
DEFAULT_C1 = -0.0030711016
DEFAULT_C2 = 3.3309495161
DEPTH_MIN_M = 0.40
DEPTH_MAX_M = 5.00
```

### Updated: setup.py Entry Points

The corrected `setup.py` (v2.0.0) now includes all six example nodes:

```python
entry_points={
    'console_scripts': [
        'kinect_driver = kinect_ros2.kinect_driver_node:main',
        'depth_alert = kinect_ros2.depth_alert_node:main',
        'tilt_control = kinect_ros2.tilt_control_node:main',
        # v2 Examples
        'ex01_rgb_viewer = kinect_ros2.examples.ex01_rgb_viewer.rgb_viewer_node:main',
        'ex02_depth_ruler = kinect_ros2.examples.ex02_depth_ruler.depth_ruler_node:main',
        'ex03_depth_histogram = kinect_ros2.examples.ex03_depth_histogram.depth_histogram_node:main',
        'ex04_hand_detector = kinect_ros2.examples.ex04_hand_detector.hand_detector_node:main',
        'ex05_coloured_cloud = kinect_ros2.examples.ex05_coloured_cloud.coloured_cloud_node:main',
        'ex06_obstacle_avoidance = kinect_ros2.examples.ex06_obstacle_avoidance.obstacle_avoidance_node:main',
    ],
},
```

## Overview & Architecture

**Core Design Principles:**
- **Modular Node Design:** Each functional subsystem (optical acquisition, motor control, analysis) operates as an independent ROS 2 node with well-defined subscriptions, publications, and parameter contracts.
- **High-Throughput Data Flow:** Optimized for real-time processing with minimal latency through efficient buffer management and zero-copy message passing where applicable.
- **Hardware Abstraction:** Encapsulates low-level `libfreenect` library interactions, exposing a clean ROS 2 interface independent of underlying hardware implementation details.
- **Runtime Reconfigurability:** All operational parameters are dynamically adjustable via ROS 2 parameter service without restarting nodes (where applicable).
- **Type Safety:** Strong adherence to standard `sensor_msgs`, `std_msgs`, and `geometry_msgs` message types for maximum ecosystem compatibility.

**Key Capabilities:**
- **Dual-Stream Optical Acquisition:** Simultaneous RGB (640×480, 8-bit per channel, 30 Hz) and depth (640×480, raw 11-bit disparity and computed 32-bit metric depth in meters) publishers.
- **Hardware Actuator Interface:** Bidirectional control of the integrated stepper motor (tilt range: ±27 degrees) with feedback telemetry.
- **Inertial Measurement Integration:** Real-time access to the built-in 3-DOF accelerometer (±8g range, typical 100 Hz update rate).
- **Spatial Analysis Engine:** Region-of-interest (ROI) computation for depth-based proximity classification with hysteretic alert states.
- **Downstream Format Conversion:** Optional conversion pipeline to laser scan format for compatibility with legacy 2D navigation stacks (`depthimage_to_laserscan` integration).
- **Calibration Support:** Per-camera intrinsic parameters (focal length, principal point, distortion coefficients) published as standard `sensor_msgs/CameraInfo` messages.

## The Six Example Nodes (v2)

All example nodes **subscribe to topics** published by the camera driver (`kinect_driver_node`). They do not access the USB device directly – this design avoids `LIBUSB_ERROR_BUSY` conflicts and allows multiple nodes to run concurrently.

### Example 01 – RGB Viewer (subscriber version)

**File:** `examples/ex01_rgb_viewer/rgb_viewer_node.py`

Subscribes to `/camera/rgb/image_raw`, adds FPS counter and timestamp overlay, and publishes annotated frames.

**Publishes:**
- `/kinect/rgb/image_raw` – Passthrough
- `/kinect/rgb/image_annotated` – RGB with overlay
- `/kinect/rgb/fps` – Measured frame rate

**Parameters:**
- `publish_rate: 30.0`
- `frame_id: "kinect_rgb_frame"`
- `font_scale: 0.7`

**Key Learning:** cv_bridge, OpenCV drawing, exponential moving average for FPS.

---

### Example 02 – Depth Ruler

**File:** `examples/ex02_depth_ruler/depth_ruler_node.py`

Extracts the nearest object distance from a configurable central ROI using the N-th percentile.

**Publishes:**
- `/kinect/ruler/distance_m` (Float32) – Nearest distance
- `/kinect/ruler/depth_colormap` – Depth image with ROI rectangle and distance label

**Parameters:**
- `roi_cx, roi_cy`: Fraction of width/height (default 0.50)
- `roi_width, roi_height`: ROI size fraction (default 0.20)
- `percentile: 10.0` – Use 10th percentile (closest 10%)

**Use Case:** Measuring distance to nearest object in front of sensor.

---

### Example 03 – Depth Histogram

**File:** `examples/ex03_depth_histogram/depth_histogram_node.py`

Computes a histogram of depth values and visualizes as a bar chart.

**Publishes:**
- `/kinect/histogram/image` – ASCII/visual histogram chart
- `/kinect/histogram/mean_m` – Mean depth across image

**Parameters:**
- `n_bins: 47`
- `hist_min: 0.3, hist_max: 5.0` – Range in metres

**Key Learning:** Depth distribution visualization, NumPy histograms.

**QoS Note:** The driver uses `BEST_EFFORT` reliability. Subscribers must match this:
```python
qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
self.create_subscription(Image, '/camera/depth/image_meters', callback, qos)
```

---

### Example 04 – Hand Detector

**File:** `examples/ex04_hand_detector/hand_detector_node.py`

Binary threshold on depth (pixels closer than threshold become white), finds largest blob, computes centroid and mean distance.

**Publishes:**
- `/kinect/hand/detected` (Bool) – True if blob found above size threshold
- `/kinect/hand/centroid_x, /kinect/hand/centroid_y` (Float32) – Pixel coordinates
- `/kinect/hand/distance_m` (Float32) – Mean depth of blob
- `/kinect/hand/image_overlay` (Image) – Depth colormap with green mask, bounding box, crosshair

**Parameters:**
- `near_threshold_m: 0.80` – Objects closer than this are "hand"
- `min_blob_area_px: 500` – Minimum pixels to detect

**Use Case:** Simple hand proximity detection without machine learning.

---

### Example 05 – Coloured Point Cloud

**File:** `examples/ex05_coloured_cloud/coloured_cloud_node.py`

Fuses RGB and depth using `message_filters.ApproximateTimeSynchronizer`. Back-projects depth to 3D points and packs RGB into float32.

**Publishes:**
- `/kinect/depth/points_colored` (PointCloud2) – XYZRGB point cloud

**Parameters:**
- `slop: 0.05` – Sync tolerance (seconds)
- `skip: 2` – Process every N-th frame to reduce load

**Key Learning:** Sensor fusion, PointCloud2 structure, RGB packing.

**Viewing in RViz2:**
1. Set Fixed Frame to `kinect_depth_frame`
2. Add PointCloud2 display
3. Under display, choose Color Transformer → RGB8

---

### Example 06 – Obstacle Avoidance

**File:** `examples/ex06_obstacle_avoidance/obstacle_avoidance_node.py`

Uses middle vertical band of depth; divides into left, centre, right sectors. State machine determines navigation command.

**States:**
- `FORWARD` – All clear → publish forward velocity
- `TURN_LEFT` – Centre/right blocked → rotate left
- `TURN_RIGHT` – Centre/left blocked → rotate right
- `STOP` – All sectors blocked

**Publishes:**
- `/cmd_vel` (Twist) – Navigation command
- `/kinect/avoidance/state` (String) – Current state
- `/kinect/avoidance/sector_image` (Image) – Debug visualization

**Testing with turtlesim:**
```bash
ros2 run turtlesim turtlesim_node &
ros2 run kinect_ros2 ex06_obstacle_avoidance
# Move hand in front of Kinect; turtle reacts
```

**Parameters:**
- `danger_distance_m: 0.80`
- `linear_speed: 0.20, angular_speed: 0.40`
- `sector_overlap: 0.10`

## System Requirements & Hardware Specifications

| Category | Component | Specification |
| :--- | :--- | :--- |
| **Operating System** | Distribution | Ubuntu 24.04 LTS (Noble Numbat), Linux kernel 6.8+ |
| | Minimum RAM | 4 GB (8 GB recommended for concurrent ROS 2 nodes) |
| **ROS 2 Middleware** | Version | Jazzy Jalopy (ROS 2 official distribution) |
| | Communication | DDS (typically FastDDS via `rmw_fastrtps_cpp`) |
| **Runtime** | Python | 3.12 (3.10+ supported with compatibility testing) |
| | Package Manager | pip or conda with setuptools |
| **Hardware** | Sensor | Microsoft Kinect for Xbox 360 (Revision 1.0) |
| | USB Interface | USB 2.0+ (high-bandwidth, low-latency required) |
| | Power Supply | 12V DC, ≥1.5A dedicated supply (included with Kinect) |
| | Processing | Multi-core CPU recommended for concurrent node execution |
| **Dependencies** | libfreenect | ≥ 0.6.2, built from source with Python bindings |
| | OpenCV | cv_bridge requires opencv-python-headless or equivalent |
| | USB Driver | libusb-1.0.0-dev (kernel USB subsystem integration) |

### Hardware Sensor Specifications

**RGB Camera Module:**
- **Resolution:** 640 × 480 pixels
- **Color Format:** RGB, 24-bit (8 bits per channel)
- **Frame Rate:** 30 Hz nominal (at VGA resolution)
- **Sensor Type:** CMOS, rolling shutter
- **Field of View:** Approximately 58° horizontal, 43° vertical
- **Focal Length (intrinsic):** ~525 pixels (in image space)
- **Principal Point:** Approximately (319.5, 239.5) in pixel coordinates

**Depth Sensor Module:**
- **Resolution:** 640 × 480 pixels
- **Depth Range:** 400 mm to 4000 mm (practical range ~500 mm to 3500 mm)
- **Raw Disparity Format:** 11-bit unsigned integer, range 0–2047
  - 0: No valid depth (beyond range or invalid surface)
  - 2047: Typically represents invalid/saturated measurement
- **Depth Computation:** Linear disparity-to-depth via calibration constants
- **Frame Rate:** 30 Hz (synchronized with RGB)
- **Depth Format (computed):** 32-bit IEEE 754 single precision (meters)
- **Accuracy:** Typical ±5–10 mm at 1 meter distance (decreases with range)

**Inertial Measurement Unit (Onboard Accelerometer):**
- **Axes:** 3-DOF (X, Y, Z)
- **Range:** ±8g (acceleration due to gravity)
- **Output Units:** m/s² (via ROS 2 driver conversion)
- **Typical Update Rate:** ~100 Hz (may vary by libfreenect version)
- **Axes Alignment:** Relative to physical sensor mounting

**Motor Subsystem (Tilt Actuator):**
- **Type:** Stepper motor with integrated gearbox
- **Range:** ±27 degrees from neutral (0° = horizontal)
- **Resolution:** Approximately 0.5° per step (depends on gearing)
- **Settling Time:** ~500–1000 ms for full range traverse
- **Current Draw:** Minimal (<100 mA average)

## Installation & Deployment

### Prerequisites Verification

Before proceeding, verify your system meets the base requirements:

```bash
# Check Ubuntu version
lsb_release -a

# Verify ROS 2 installation
ros2 --version

# Check Python version
python3 --version

# Verify libusb presence (optional, for reference)
pkg-config --list-all | grep libusb
```

### Step 1: Install System-Level Dependencies

Acquire the complete set of build tools, USB support libraries, and multimedia dependencies required for `libfreenect` compilation and OpenCV integration:

```bash
sudo apt update && sudo apt upgrade -y

# Core build infrastructure
sudo apt install -y build-essential cmake git pkg-config

# USB communication and hardware access
sudo apt install -y libusb-1.0-0-dev libusb-dev

# Graphics libraries (required for optional example viewers)
sudo apt install -y libglfw3-dev freeglut3-dev

# Image processing and computer vision
sudo apt install -y libopencv-dev

# Python development headers
sudo apt install -y python3-dev python3-pip

# System utility for dynamic device rule reloading
sudo apt install -y libudev-dev

# Optional: Debug utilities
sudo apt install -y libusb-1.0-0-doc usbutils
```

**Verification:**
```bash
# Confirm libusb availability
ldconfig -p | grep libusb
# Output should include: libusbx.so (or libusb.so)
```

### Step 2: Clone and Build libfreenect from Source

The `freenect` Python module requires compilation with Python bindings enabled. The precompiled pip package may lack compatibility with your specific environment.

```bash
# Clone the OpenKinect repository
cd ~
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect

# Create isolated build directory
mkdir build
cd build

# Configure CMake with Python bindings enabled
cmake .. \
  -DBUILD_PYTHON=ON \
  -DBUILD_EXAMPLES=ON \
  -DPYTHON_EXECUTABLE=$(which python3) \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DCMAKE_BUILD_TYPE=Release

# Compile using all available processor cores
make -j$(nproc)

# Install to system directories
sudo make install

# Update dynamic linker cache (critical for library discovery)
sudo ldconfig

# Verify installation
pkg-config --modversion libfreenect
```

**Post-Build Verification:**
```bash
# Check library installation
ls -la /usr/local/lib/libfreenect*

# Verify pkg-config
pkg-config --cflags --libs libfreenect
```

### Step 3: Install Python Bindings

Install the compiled Python interface using pip:

```bash
pip3 install freenect
```

**Test the binding:**
```bash
python3 -c "import freenect; print(freenect.__version__)"
```

If the above command fails with `ModuleNotFoundError`, the Python bindings may not have been compiled correctly. Ensure CMake detected Python during the configuration phase:

```bash
# Re-run CMake with verbose output to check Python detection
cd ~/libfreenect/build
cmake .. -DBUILD_PYTHON=ON -DCMAKE_MESSAGE_LOG_LEVEL=VERBOSE

# Look for: "Found PythonInterp" and "Found PythonLibs"
```

### Step 4: Configure udev Rules for Hardware Access

By default, USB device access is restricted to the root user. Device rules allow non-root users in the `video` group to access the Kinect:

```bash
# Copy pre-configured udev rules
sudo cp ~/libfreenect/platform/linux/udev/51-kinect.rules /etc/udev/rules.d/

# Reload udev rule database
sudo udevadm control --reload-rules

# Trigger re-enumeration of USB devices
sudo udevadm trigger

# Add current user to video group
sudo usermod -aG video $USER

# Secondary verification: check group membership
groups $USER
```

**Critical:** The group membership change requires a new login session. Either:
- Log out completely and log back in, or
- Open a new terminal and execute `newgrp video`, or
- Execute `reboot` for system-wide effect.

**Verify functionality:**
```bash
# With Kinect powered and connected to USB
lsusb | grep -i kinect

# Expected output:
# Bus 001 Device 003: ID 045e:02ae Microsoft Corp. Xbox NUI Camera
# Bus 001 Device 004: ID 045e:02ad Microsoft Corp. Xbox NUI Motor
```

If devices do not appear, check USB port assignment and power supply voltage (12V DC).

### Step 5: Initialize ROS 2 Workspace

Establish the workspace directory structure and fetch dependencies:

```bash
# Create workspace hierarchy
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the kinect_ros2 package repository
git clone https://github.com/YOUR_USERNAME/kinect_ros2.git

# (Alternatively, if already in this directory, skip cloning)
```

### Step 6: Install ROS 2 Package Dependencies

Acquire all ROS 2 packages that `kinect_ros2` depends upon:

```bash
# Install binary packages for the Jazzy distribution
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-depthimage-to-laserscan \
  ros-jazzy-rviz2 \
  ros-jazzy-image-transport \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs

# Optional: Install testing and development tools
sudo apt install -y \
  ros-jazzy-rqt-image-view \
  ros-jazzy-rviz-visual-testing-framework
```

### Step 7: Build with colcon

Compile the package within the ROS 2 build system:

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Execute colcon build targeting the kinect_ros2 package
colcon build --packages-select kinect_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the setup script to register the package
source install/setup.bash

# Persistent sourcing: add to ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

**Build Verification:**
```bash
# List available executables
ros2 pkg executables kinect_ros2

# Expected output:
# kinect_ros2 depth_alert
# kinect_ros2 kinect_driver
# kinect_ros2 tilt_control
```

### Troubleshooting Installation

| Issue | Diagnosis | Resolution |
| :--- | :--- | :--- |
| CMake cannot find Python | `Could not find PythonLibs` | Verify `python3-dev` installed: `apt list --installed \| grep python3-dev` |
| libfreenect not found by pkg-config | `pkg-config: not found` | Ensure `/usr/local/lib/pkgconfig` in `PKG_CONFIG_PATH`: `export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH` |
| Permission denied on USB device | `libusb: error [udev_hotplug_start]` | Re-run udev rules setup and verify group membership: `id -G \| grep video` |
| Python freenect import fails | `ModuleNotFoundError: No module named 'freenect'` | Verify Python site-packages: `python3 -c "import site; print(site.getsitepackages())"` |
| ROS 2 package not found | `Package 'kinect_ros2' not found` | Source setup script: `source ~/ros2_ws/install/setup.bash` |

## Launch Files & Deployment Scenarios

The package provides a hierarchical set of launch files enabling flexible deployment from minimal sensor-only drivers to fully instrumented multi-node stacks with visualization and optional format converters.

### kinect.launch.py – Core Driver Minimal Deployment

Launches only the primary sensor driver node without ancillary processing pipelines.

**Usage:**
```bash
ros2 launch kinect_ros2 kinect.launch.py
```

**Command-Line Parameter Override Examples:**
```bash
# Adjust frame acquisition rate
ros2 launch kinect_ros2 kinect.launch.py target_fps:=15.0

# Select alternate sensor (for multi-Kinect setups)
ros2 launch kinect_ros2 kinect.launch.py device_index:=1

# Disable depth publishing (RGB-only mode)
ros2 launch kinect_ros2 kinect.launch.py publish_depth:=false

# Specify custom namespace for topic isolation
ros2 launch kinect_ros2 kinect.launch.py namespace:=kinect_unit_1
```

**Published Topics (Available Immediately):**
- `/kinect/camera/rgb/image_raw`
- `/kinect/camera/rgb/camera_info`
- `/kinect/camera/depth/image_raw`
- `/kinect/camera/depth/image_meters`
- `/kinect/camera/depth/camera_info`

**Use Case:** Headless sensor acquisition for embedded systems, mobile platforms, or environments where alert/tilt nodes are handled externally.

---

### kinect_full.launch.py – Multi-Node Complete Stack

Orchestrates the full ecosystem: core driver, depth-based proximity analysis, motor control, and optional laser scan converter.

**Usage:**
```bash
ros2 launch kinect_ros2 kinect_full.launch.py
```

**Key Launch Arguments:**
```bash
# Enable/disable depth-to-laserscan conversion node
ros2 launch kinect_ros2 kinect_full.launch.py enable_laserscan:=true

# Enable/disable tilt control node
ros2 launch kinect_ros2 kinect_full.launch.py enable_tilt:=true

# Enable/disable proximity alert node
ros2 launch kinect_ros2 kinect_full.launch.py enable_alert:=true

# Combine multiple parameter overrides
ros2 launch kinect_ros2 kinect_full.launch.py \
  enable_laserscan:=true \
  enable_tilt:=true \
  enable_alert:=true \
  device_index:=0 \
  namespace:=kinect
```

**Node Topology (with all optional nodes enabled):**
```
┌──────────────────────┐
│  kinect_driver_node  │ ──→ /camera/rgb/image_raw
│  (core sensor I/O)   │ ──→ /camera/rgb/camera_info
│                      │ ──→ /camera/depth/image_raw
│                      │ ──→ /camera/depth/image_meters
│                      │ ──→ /camera/depth/camera_info
└──────┬───────────────┘
       │
       ├──→ ┌──────────────────────────┐
       │    │  depth_alert_node        │ ──→ /kinect/proximity_alert
       │    │  (spatial analysis)      │
       │    └──────────────────────────┘
       │
       ├──→ ┌──────────────────────────┐
       │    │  tilt_control_node       │ ←─  /kinect/tilt_cmd
       │    │  (motor + IMU))          │ ──→ /kinect/tilt_angle
       │    │                          │ ──→ /kinect/accel
       │    └──────────────────────────┘
       │
       └──→ ┌──────────────────────────────────┐
            │  depthimage_to_laserscan_node    │ ──→ /scan
            │  (optional 2D converter)         │
            │  (external package)              │
            └──────────────────────────────────┘
```

**Typical Usage:** Autonomous mobile robots, obstacle avoidance systems, depth-based navigation.

---

### kinect_rviz.launch.py – Visualization & Debugging Stack

Integrates the full multi-node deployment alongside RViz 2 for real-time sensor data monitoring and system state visualization.

**Usage:**
```bash
ros2 launch kinect_ros2 kinect_rviz.launch.py
```

**Parameter Customization:**
```bash
ros2 launch kinect_ros2 kinect_rviz.launch.py \
  target_fps:=20.0 \
  enable_alert:=true \
  enable_laserscan:=false
```

**RViz Configuration Details:**
The supplied `config/kinect.rviz` configuration file includes:
- **Image Display:** RGB stream (rescaled for viewport, 8-bit RGB)
- **Depth Display:** Metric depth map with "rainbow" colormap (0–4 meter range)
- **PointCloud2:** Reconstructed 3D point cloud from depth (if generated upstream)
- **Laser Scan Overlay:** 2D laser scan visualization (if `enable_laserscan:=true`)
- **TF Tree:** Sensor frame hierarchy (RGB, depth, motor)
- **Status Panels:** Node health, frame rates, connection status

**Expected RViz Output:**
- Real-time RGB and depth stream rendering
- Visual feedback of tilt motor angle via TF frame visualization
- Proximity alert state indication
- Frame rate diagnostics (typically 25–30 Hz for RGB/depth)

---

## Launch Files for v2 Examples

### kinect_all_examples.launch.py (NEW in v2)

**Location:** `launch/kinect_all_examples.launch.py`

Orchestrates the complete v2 experience: launches the driver and all six example nodes with staggered delays to prevent CPU spikes.

**Usage:**
```bash
ros2 launch kinect_ros2 kinect_all_examples.launch.py
ros2 launch kinect_ros2 kinect_all_examples.launch.py publish_rate:=30.0 tilt_angle:=0.0
```

**Startup Sequence:**
1. Driver starts immediately (t=0s)
2. Examples start with 1.0s delays: ex01 at 1.0s, ex02 at 1.5s, ..., ex06 at 3.5s
3. Uses `TimerAction` for coordinated timing

---

### examples_params.yaml (NEW in v2)

**Location:** `config/examples_params.yaml`

Centralized parameter configuration for all six example nodes. Loaded automatically by launch files.

**Contents:**
```yaml
rgb_viewer_node:
  ros__parameters:
    publish_rate: 30.0
    frame_id: "kinect_rgb_frame"
    font_scale: 0.7

depth_ruler_node:
  ros__parameters:
    roi_cx: 0.50
    roi_cy: 0.50
    roi_width: 0.20
    roi_height: 0.20
    percentile: 10.0
    frame_id: "kinect_depth_frame"

depth_histogram_node:
  ros__parameters:
    n_bins: 47
    hist_min: 0.3
    hist_max: 5.0

hand_detector_node:
  ros__parameters:
    near_threshold_m: 0.80
    min_blob_area_px: 500
    frame_id: "kinect_depth_frame"

coloured_cloud_node:
  ros__parameters:
    frame_id: "kinect_depth_frame"
    slop: 0.05
    skip: 2

obstacle_avoidance_node:
  ros__parameters:
    danger_distance_m: 0.80
    linear_speed: 0.20
    angular_speed: 0.40
    sector_overlap: 0.10
    frame_id: "kinect_depth_frame"
```

## Nodes, Topics & Message Interface Specification

### kinect_driver_node – Primary Optical Data Acquisition

The core driver node encapsulates all direct hardware interaction via the `libfreenect` library. It manages USB communication, frame buffer allocation, sensor synchronization, and publishes stereoscopic image data with associated camera intrinsics.

**Node Characteristics:**
- **Executable:** `kinect_driver` (entry point defined in `setup.py`)
- **Default Namespace Prefix:** `/kinect` (configurable via launch parameter)
- **Concurrency Model:** Single-threaded event loop with blocking USB reads (no spinning)
- **Expected CPU Usage:** ~5–8% on modern multi-core systems (depends on frame rate)
- **Memory Footprint:** ~50–80 MB (primarily RGB/depth buffer allocation)
- **Lifespan:** Spawned at launch and terminates cleanly via `rclpy.shutdown()`

**Published Topics (QoS: Reliable, History: Keep Last 1):**

| Topic | Message Type | Description | Publication Rate |
| :--- | :--- | :--- | :--- |
| `/kinect/camera/rgb/image_raw` | `sensor_msgs/Image` | Uncompressed RGB video frame, 640×480 pixels, pixel format: `rgb8` | 30 Hz |
| `/kinect/camera/rgb/camera_info` | `sensor_msgs/CameraInfo` | RGB camera calibration: intrinsic matrix K, distortion coefficients D, frame ID `camera_rgb_optical_frame` | 30 Hz (synchronized with RGB) |
| `/kinect/camera/depth/image_raw` | `sensor_msgs/Image` | Raw disparity map, 640×480 pixels, pixel format: `16UC1` (unsigned 16-bit), intensity encoding raw hardware values (0–2047) | 30 Hz |
| `/kinect/camera/depth/image_meters` | `sensor_msgs/Image` | Metric depth in meters, 640×480 pixels, pixel format: `32FC1` (IEEE 754 single-precision float), range [0.0, 4.0] m | 30 Hz |
| `/kinect/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Depth camera calibration: intrinsic matrix K (distinct from RGB), frame ID `camera_depth_optical_frame`, distortion vector D | 30 Hz |

**Message Structure Details:**

`sensor_msgs/Image` for RGB:
```python
header:
  seq: [auto-incremented frame counter]
  stamp: [current ROS time, nanosecond precision]
  frame_id: "camera_rgb_optical_frame"
height: 480
width: 640
encoding: "rgb8"                 # 3 bytes per pixel: R, G, B
is_bigendian: false
step: 1920                       # width * 3 (bytes per row)
data: [uint8[...]               # 640 × 480 × 3 = 921,600 bytes
```

`sensor_msgs/CameraInfo` for RGB Intrinsics:
```python
header:
  stamp: [ROS time]
  frame_id: "camera_rgb_optical_frame"
height: 480
width: 640
distortion_model: "plumb_bob"
D: [D1, D2, D3, D4, D5]          # Distortion coefficients (typically near-zero)
K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]  # Intrinsic calibration matrix
    # fx ≈ 525.0 (focal length in pixels, x-axis)
    # fy ≈ 525.0 (focal length in pixels, y-axis)
    # cx ≈ 319.5 (principal point, x-coordinate)
    # cy ≈ 239.5 (principal point, y-coordinate)
R: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # Identity (single-camera baseline)
P: [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]  # Projection matrix for stereo (baseline = 0)
```

`sensor_msgs/Image` for Depth (Raw Disparity):
```python
header:
  stamp: [ROS time, synchronized with RGB timestamp]
  frame_id: "camera_depth_optical_frame"
height: 480
width: 640
encoding: "16UC1"               # Unsigned 16-bit integers
step: 1280                      # width * 2 (bytes per row)
data: [uint16[...]]             # 640 × 480 × 2 = 614,400 bytes
```

`sensor_msgs/Image` for Depth (Metric):
```python
header:
  stamp: [ROS time, synchronized with RGB and raw depth]
  frame_id: "camera_depth_optical_frame"
height: 480
width: 640
encoding: "32FC1"               # IEEE 754 single-precision floats
step: 2560                      # width * 4 (bytes per row)
data: [float32[...]]            # 640 × 480 × 4 = 1,228,800 bytes
# Values: 0.0 (no reading), or distance in meters [0.4, 4.0]
```

**Depth Conversion Mathematics:**

The conversion from raw disparity *r* (0–2047, encoded in image) to metric depth *d* (meters) uses the inverse linear relationship:

$$d = \frac{1}{(r \cdot C_1) + C_2}$$

where the hardware calibration constants are:
$$C_1 = -0.0030711016$$
$$C_2 = 3.3309495161$$

**Special Handling:**
- **r = 0**: No return (beyond sensor range or occluded) → mapped to 0.0 m
- **r = 2047**: Saturated/invalid reading → mapped to 0.0 m
- **0 < r < 2047**: Valid disparity → normal conversion via formula above

**Configurable Parameters:** (from `config/kinect_params.yaml`)

| Parameter | Type | Default | Range | Effect |
| :--- | :--- | :--- | :--- | :--- |
| `device_index` | `int` | `0` | 0 to N-1 | Selects Kinect device (0-indexed enumeration via libfreenect) |
| `target_fps` | `float` | `30.0` | 10.0–30.0 | Frame acquisition rate (Hz); USB bandwidth permitting |
| `rgb_frame_id` | `string` | `camera_rgb_optical_frame` | any | TF frame identifier for RGB stream |
| `depth_frame_id` | `string` | `camera_depth_optical_frame` | any | TF frame identifier for depth stream |
| `publish_rgb` | `bool` | `true` | true, false | Enable/disable RGB stream publication |
| `publish_depth` | `bool` | `true` | true, false | Enable/disable depth stream publication |
| `rgb_fx` | `float` | `525.0` | physical constraint | Focal length (RGB, pixels) |
| `rgb_fy` | `float` | `525.0` | physical constraint | Focal length (RGB, pixels) |
| `rgb_cx` | `float` | `319.5` | 0–640 | Principal point X, RGB |
| `rgb_cy` | `float` | `239.5` | 0–480 | Principal point Y, RGB |
| `depth_fx` | `float` | `580.0` | physical constraint | Focal length (depth, pixels) |
| `depth_fy` | `float` | `580.0` | physical constraint | Focal length (depth, pixels) |
| `depth_cx` | `float` | `319.5` | 0–640 | Principal point X, depth |
| `depth_cy` | `float` | `239.5` | 0–480 | Principal point Y, depth |

**Behavior on Hardware Errors:**
- USB communication loss → node terminates with error log
- Frame buffer underrun → skipped frame (no publication)
- Timeout on libfreenect call → logged warning, attempt recovery
- Device hot-unplugged → node shutdown triggered

---

### depth_alert_node – Proximity-Based Obstacle Classification

A lightweight spatial analysis node that subscribes to the metric depth map and performs Region-of-Interest (ROI) minimum value computation to classify the nearest obstacle into alert states.

**Node Characteristics:**
- **Executable:** `depth_alert`
- **Subscription Model:** Single depth input (metric, 32FC1)
- **Latency:** Minimal (<5 ms typical per frame)
- **CPU Overhead:** <2% (vectorized NumPy operations)

**Subscribed Topics:**

| Topic | Message Type | Expected Rate | Role |
| :--- | :--- | :--- | :--- |
| `/kinect/camera/depth/image_meters` | `sensor_msgs/Image` (32FC1) | 30 Hz | Metric depth map (see kinect_driver_node above) |

**Published Topics:**

| Topic | Message Type | Description | Publication Rate |
| :--- | :--- | :--- | :--- |
| `/kinect/proximity_alert` | `std_msgs/String` | Alert state classification (CLEAR, WARNING, or DANGER as string data) | 30 Hz (synchronized with input) |

**Message Content:**

`std_msgs/String` alert states:
```python
data: "CLEAR"      # Nearest obstacle > warning_distance_m (default: > 1.50 m)
data: "WARNING"    # warning_distance_m ≥ nearest ≥ danger_distance_m (default: 1.50 m to 0.80 m)
data: "DANGER"     # Nearest obstacle ≤ danger_distance_m (default: ≤ 0.80 m)
```

**Classification Algorithm:**
1. Extract central ROI from depth map (fraction specified by parameter)
2. Filter out invalid pixels (0.0 m, or values < min_valid_distance_m)
3. Compute minimum distance across valid ROI pixels
4. Map to alert state using hysteretic thresholds
5. Publish state string

**Configurable Parameters:**

| Parameter | Type | Default | Purpose |
| :--- | :--- | :--- | :--- |
| `danger_distance_m` | `float` | `0.80` | Threshold for DANGER state (≤ this value) |
| `warning_distance_m` | `float` | `1.50` | Threshold for WARNING state (> danger, ≤ this value) |
| `min_valid_distance_m` | `float` | `0.10` | Minimum distance to consider valid (ignore noise) |
| `roi_fraction` | `float` | `0.5` | Fraction of image center to analyze (0.0 = full image, 1.0 = single pixel) |
| `alert_topic` | `string` | `/kinect/proximity_alert` | Configurable output topic name |

**ROI Computation Example:**
If `roi_fraction = 0.5` and image is 640×480:
- ROI width: 640 × 0.5 = 320 pixels
- ROI height: 480 × 0.5 = 240 pixels
- ROI center: (320, 240) → top-left at (160, 120), bottom-right at (480, 360)

---

### tilt_control_node – Motor Actuation & IMU Telemetry

Manages the integrated stepper motor and accelerometer via the hardware API exposed by libfreenect. Provides bidirectional control: subscription to command angles and publication of feedback (verified angle and inertial telemetry).

**Node Characteristics:**
- **Executable:** `tilt_control`
- **Control Model:** Open-loop angle command with closed-loop feedback
- **API Adaptation:** Dynamically detects libfreenect Python variant (legacy vs. modern pointer semantics)
- **Update Rate:** Typically 10 Hz (tunable, though hardware is ~100 Hz internally)

**Subscribed Topics:**

| Topic | Message Type | Command Format | Command Range |
| :--- | :--- | :--- | :--- |
| `/kinect/tilt_cmd` | `std_msgs/Float32` | Desired tilt angle (degrees) | [-27.0, +27.0] |

**Command Usage:**
```bash
# Tilt up (+15 degrees)
ros2 topic pub /kinect/tilt_cmd std_msgs/Float32 "{data: 15.0}" --once

# Neutral position (0 degrees)
ros2 topic pub /kinect/tilt_cmd std_msgs/Float32 "{data: 0.0}" --once

# Tilt down (-20 degrees)
ros2 topic pub /kinect/tilt_cmd std_msgs/Float32 "{data: -20.0}" --once
```

**Published Topics:**

| Topic | Message Type | Description | Publication Rate |
| :--- | :--- | :--- | :--- |
| `/kinect/tilt_angle` | `std_msgs/Float32` | Current verified tilt angle (degrees) | Configurable (default 5 Hz) |
| `/kinect/accel` | `geometry_msgs/Vector3` | Accelerometer vector (m/s²) | Configurable (default 5 Hz) |

**Message Structures:**

`std_msgs/Float32` (tilt angle):
```python
data: 15.0    # Degrees from neutral (-27 to +27)
```

`geometry_msgs/Vector3` (accelerometer):
```python
x: -0.45      # m/s² (device pitch acceleration)
y: 0.12       # m/s² (device roll acceleration)
z: 9.81       # m/s² (gravity component)
```

**Accelerometer Axes:**
The three axes are aligned with the physical sensor orientation:
- **X-axis:** Forward-backward (pitch)
- **Y-axis:** Left-right (roll)
- **Z-axis:** Up-down (gravity-aligned)

The magnitude of the acceleration vector should be approximately 9.81 m/s² when static (gravity alone).

**Configurable Parameters:**

| Parameter | Type | Default | Purpose |
| :--- | :--- | :--- | :--- |
| `device_index` | `int` | `0` | Kinect device index (multi-sensor scenarios) |
| `publish_rate_hz` | `float` | `5.0` | Feedback telemetry publication frequency (Hz) |

**Operating Behavior:**
- **Command Receipt:** Angle command triggers immediate hardware actuation (non-blocking)
- **Settling Time:** Motor traversal requires ~500–1000 ms for full range
- **Feedback Loop:** Angle feedback is read from motor encoder; may lag command by settling time
- **No Guarantee:** If command is impossible (hardware failure), motor will not move but command is acknowledged
- **Timeout Behavior:** If no command is received for >10 seconds, motor maintains position (no idle drift)

## Package Architecture & Codebase Organization

The `kinect_ros2` package is structured to separate concerns across tightly-scoped modules, facilitating testing, maintenance, and extension.

```text
kinect_ros2/
│
├── kinect_ros2/                          # Main source package directory
│   ├── __init__.py                       # Package initialization marker
│   │
│   ├── kinect_driver_node.py             # Primary driver node
│   │   ├── Hardware initialization (device enumeration, configuration)
│   │   ├── RGB/depth frame acquisition loop
│   │   ├── Message serialization (Image, CameraInfo)
│   │   ├── Parameter management & validation
│   │   └── Error recovery logic
│   │
│   ├── kinect_device.py                  # NEW: Context manager API (v2)
│   │   ├── Device opening/closing protocol
│   │   ├── LED control & tilt motor interface
│   │   ├── Synchronous frame polling (video, depth, accel)
│   │   └── Safe cleanup on exit
│   │
│   ├── depth_alert_node.py               # Spatial analysis node
│   │   ├── Depth map subscription & buffering
│   │   ├── ROI computation & filtering
│   │   ├── Proximity classification logic
│   │   └── State transition management
│   │
│   ├── tilt_control_node.py              # Motor control node
│   │   ├── libfreenect API variant detection
│   │   ├── Motor command execution
│   │   ├── Angle feedback retrieval
│   │   ├── Accelerometer telemetry
│   │   └── Hardware state caching
│   │
│   ├── depth_utils.py                    # Shared utility module
│   │   ├── Depth conversion functions (disparity → meters)
│   │   ├── ROI extraction helpers
│   │   ├── Coordinate transformation utilities
│   │   ├── Validation functions (bounds checking)
│   │   └── NumPy-backed numerical operations
│   │
│   └── examples/                         # NEW: Six example nodes (v2)
│       ├── __init__.py
│       ├── ex01_rgb_viewer/
│       │   ├── __init__.py
│       │   ├── rgb_viewer_node.py        # RGB annotation & FPS overlay
│       │   └── launch_rgb_viewer.launch.py
│       ├── ex02_depth_ruler/
│       │   ├── __init__.py
│       │   ├── depth_ruler_node.py       # Distance measurement (percentile-based)
│       │   └── launch_depth_ruler.launch.py
│       ├── ex03_depth_histogram/
│       │   ├── __init__.py
│       │   ├── depth_histogram_node.py   # Depth distribution visualization
│       │   └── launch_depth_histogram.launch.py
│       ├── ex04_hand_detector/
│       │   ├── __init__.py
│       │   ├── hand_detector_node.py     # Blob detection & proximity alert
│       │   └── launch_hand_detector.launch.py
│       ├── ex05_coloured_cloud/
│       │   ├── __init__.py
│       │   ├── coloured_cloud_node.py    # Sensor fusion (RGB + depth→XYZRGB)
│       │   └── launch_coloured_cloud.launch.py
│       └── ex06_obstacle_avoidance/
│           ├── __init__.py
│           ├── obstacle_avoidance_node.py # State machine navigation
│           └── launch_obstacle_avoidance.launch.py
│
├── config/                               # Configuration files
│   ├── kinect_params.yaml                # Driver parameters
│   ├── examples_params.yaml              # NEW: Parameters for all examples (v2)
│   └── kinect.rviz                       # RViz 2 configuration
│
├── launch/                               # ROS 2 launch definitions
│   ├── kinect.launch.py                  # Minimal deployment (driver only)
│   ├── kinect_full.launch.py             # Multi-node deployment
│   ├── kinect_rviz.launch.py             # Full stack + visualization
│   └── kinect_all_examples.launch.py     # NEW: All examples + driver (v2)
│
├── resource/                             # Package metadata
│   └── kinect_ros2                       # Index resource (ament_index marker)
│
├── test/                                 # Test suite (v2 expanded)
│   ├── test_copyright.py                 # License header validation
│   ├── test_flake8.py                    # PEP 8 style checking
│   ├── test_pep257.py                    # Docstring format validation
│   ├── test_kinect_utils.py              # NEW: depth_utils unit tests
│   ├── test_camera_info.py               # NEW: Camera intrinsics validation
│   ├── test_device_api.py                # NEW: KinectDevice context tests
│   ├── test_depth_alert_params.py        # NEW: Alert node parameter tests
│   └── test_integration_topics.py        # NEW: ROS 2 topic integration tests
│
├── package.xml                           # ROS 2 package metadata (dependencies, description)
├── setup.py                              # Python setuptools configuration (entry points)
├── setup.cfg                             # Setuptools supplementary configuration
└── LICENSE                               # Apache 2.0 license text
```

**Module Dependencies (Core Nodes):**
```
kinect_driver_node ←─ depth_utils (calibration constants, conversion)
depth_alert_node ←─ depth_utils (ROI computation, filtering)
tilt_control_node ←─ (no internal dependencies)
```

**v2 Example Dependencies:**
```
ex01_rgb_viewer ←─ cv_bridge, opencv
ex02_depth_ruler ←─ depth_utils
ex03_depth_histogram ←─ opencv, numpy
ex04_hand_detector ←─ opencv, scipy (connected components)
ex05_coloured_cloud ←─ message_filters (time synchronization)
ex06_obstacle_avoidance ←─ numpy (sector analysis)
```

**External Dependencies Graph:**
```
rclpy ─────────────────→ ROS 2 node lifecycle & communication
sensor_msgs ───────────→ Image, CameraInfo, PointCloud2
std_msgs ──────────────→ Float32, String, Bool message types
geometry_msgs ─────────→ Vector3 (accelerometer), Twist (velocities)
cv_bridge ─────────────→ OpenCV ↔ ROS 2 Image bridge
message_filters ───────→ Time synchronization (ex05)
freenect ──────────────→ libfreenect Python bindings (hardware)
numpy ─────────────────→ Efficient tensor operations
opencv-python ────────→ Image processing (example nodes)
scipy ─────────────────→ Connected components labeling (ex04)
```

### Depth Calibration Constants

The depth conversion formula depends on two calibration constants specific to each Kinect unit. To refine these values:

1. **Capture Calibration Data:** Record depth measurements at known distances.
2. **Compute Regression:** Fit the inverse linear model to your data.
3. **Update Constants:** Modify `C1` and `C2` in the `depth_conversion()` function in `depth_utils.py`.

**Example Calibration Procedure:**
```python
# Measure depth at 0.5m, 1.0m, 1.5m, 2.0m, 3.0m
# For each distance, record the mean raw disparity value (r)
# Fit: d = 1 / (r * C1 + C2)

measured_data = [
    (0.50, 1450),  # (meters, raw_disparity)
    (1.00, 1000),
    (1.50, 700),
    (2.00, 550),
    (3.00, 380),
]

from scipy.optimize import curve_fit

def depth_model(r, c1, c2):
    return 1.0 / (r * c1 + c2)

r_vals = np.array([d[1] for d in measured_data])
d_vals = np.array([d[0] for d in measured_data])

popt, _ = curve_fit(depth_model, r_vals, d_vals)
print(f"C1={popt[0]}, C2={popt[1]}")
```

### RGB Camera Intrinsics Refinement

Use the ROS 2 `camera_calibration` package to compute or refine intrinsic parameters (fx, fy, cx, cy) via chessboard-based calibration.

---

## Running the v2 Examples

### Prerequisites

- Ubuntu 24.04 with ROS2 Jazzy
- libfreenect installed (`sudo apt install libfreenect-dev`)
- Python packages: `opencv-python`, `numpy`, `cv-bridge`, `rclpy`, `message-filters`
- Kinect Xbox 360 connected with permissions set
- Run `colcon build --packages-select kinect_ros2 --symlink-install` to build

### Build the Package

```bash
cd ~/freenect_ros2
colcon build --packages-select kinect_ros2 --symlink-install
source install/setup.bash
```

### Launch Everything at Once

```bash
ros2 launch kinect_ros2 kinect_all_examples.launch.py
```

After 3-4 seconds, all nodes will be active. You can then:

**View the annotated RGB stream:**
```bash
ros2 run rqt_image_view rqt_image_view
# Select /kinect/rgb/image_annotated
```

**See the depth ruler distance:**
```bash
ros2 topic echo /kinect/ruler/distance_m
```

**Watch the histogram chart:**
```bash
ros2 run rqt_image_view rqt_image_view
# Select /kinect/histogram/image
```

**Monitor hand detections:**
```bash
ros2 topic echo /kinect/hand/detected
```

**Visualize the coloured point cloud in RViz2:**
```bash
rviz2
# Set Fixed Frame to kinect_depth_frame
# Add PointCloud2 display for /kinect/depth/points_colored
# Under display, choose Color Transformer → RGB8
```

**Observe obstacle avoidance state:**
```bash
ros2 topic echo /kinect/avoidance/state
```

### Run Individual Examples

Each example can be launched alone (with the driver already running in another terminal):

```bash
# Terminal 1: Start the driver
ros2 run kinect_ros2 kinect_driver

# Terminal 2: Run a specific example
ros2 run kinect_ros2 ex01_rgb_viewer
# or ex02_depth_ruler, ex03_depth_histogram, ex04_hand_detector, etc.
```

**Using individual launch files:**
```bash
ros2 launch kinect_ros2 examples/ex01_rgb_viewer/launch_rgb_viewer.launch.py
ros2 launch kinect_ros2 examples/ex02_depth_ruler/launch_depth_ruler.launch.py
# etc.
```

---

## Troubleshooting & Diagnostics

This section provides systematic diagnosis and resolution for common issues encountered during deployment and operation.

### USB Hardware & Connectivity

| Symptom | Diagnosis | Resolution |
| :--- | :--- | :--- |
| `LIBUSB_ERROR_ACCESS` | User lacks USB device permissions or udev rules not loaded | Verify udev rules installed: `ls -la /etc/udev/rules.d/51-kinect.rules`; reload rules: `sudo udevadm control --reload-rules && sudo udevadm trigger`; verify group: `id -G \| grep video` (must include video group ID) |
| `LIBUSB_ERROR_NO_DEVICE` | Kinect enumeration failed; device not connected or powered | Verify USB connection with: `lsusb \| grep -i kinect` (should show 2 devices: camera and motor); check 12V power supply voltage |
| `LIBUSB_ERROR_BUSY` | Another process holds the device (e.g., `freenect-glview`, stalled node, or previous terminated instance) | Kill competing processes: `killall freenect-glview` or `pkill -f kinect`; power-cycle Kinect (unplug 12V and USB for 5 seconds) |
| No devices after `lsusb` | USB enumeration timeout; possible USB controller or cable issue | Test with different USB port (preferably dedicated, not hub); test with `dmesg \| tail -20` for kernel USB errors; consider powered USB hub if necessary |
| Intermittent disconnection | Weak power supply or noisy USB cable | Upgrade to dedicated 2A+ power supply; use shielded USB 2.0 cable (not 3.0 with additional insulation); minimize cable length (<3 meters) |

### Driver & Node Initialization

| Symptom | Diagnosis | Resolution |
| :--- | :--- | :--- |
| Node startup fails: `Could not open device` | Device not found or permission denied | Verify device with `lsusb`; ensure user in `video` group; power-cycle Kinect |
| `rcl_shutdown already called` | Overlapping node instances or improper cleanup | Ensure only one instance of each node; use `ros2 node list` to verify; kill stale nodes with `pkill -f kinect` |
| Node crashes after <1 second | libfreenect Python binding mismatch or missing calibration file | Verify freenect import: `python3 -c "import freenect; print(freenect.\_\_version\_\_)"`; reload binding if needed: `pip3 install --upgrade --force-reinstall freenect` |
| Topics not appearing | Workspace not sourced or node failed silently | Verify sourcing: `source ~/ros2_ws/install/setup.bash`; check node status: `ros2 node list`; inspect logs: `tail -50 ~/.ros/log/latest/*/stdout`; or launch with `--log-level debug` |

### Sensor Data & Quality

| Symptom | Diagnosis | Resolution |
| :--- | :--- | :--- |
| RGB image all black or noisy | Insufficient ambient light, excessive IR interference, or hardware failure | Increase scene brightness (turn on lights); move away from bright IR sources (direct sunlight, IR heaters); test with `rosservice call /kinect_driver/get_device_info` to inspect hardware state |
| Depth image all zeros or noisy | Surfaces too close (<0.5 m) or too far (>4 m); high ambient IR; reflective/transparent surfaces | Position objects 0.5–3 m away; reduce ambient IR; target diffuse reflective surfaces; move away from windows |
| Frame rate dropping below 30 Hz | USB bandwidth saturation or CPU bottleneck | Monitor with `ros2 topic hz /kinect/camera/rgb/image_raw`; verify no competing USB devices; check CPU with `top` (kinect_ros2 should use <10%); reduce resolution if supported (requires code modification) |
| RGB-Depth misalignment | Incorrect camera intrinsics or temporal skew | Recalibrate RGB camera with `camera_calibration` package; verify that RGB and depth timestamps are synchronized (check message headers in `rosbag` playback with `rostopic echo -n 1`) |

### Software & Dependencies

| Symptom | Diagnosis | Resolution |
| :--- | :--- | :--- |
| `ImportError: No module named 'freenect'` | Python binding not installed or installed in wrong Python version | Verify Python version: `python3 --version` (must be 3.10+); reinstall binding: `pip3 install freenect`; check site-packages path: `python3 -c "import site; print(site.getsitepackages())"` |
| `cv_bridge.CvBridgeError` when using depth images | OpenCV encoding mismatch or missing cv_bridge dependency | Verify cv_bridge installed: `python3 -c "import cv_bridge"`; ensure ROS 2 dependency: `apt list --installed \| grep ros-.*-cv-bridge` |
| Launch file not found | Installation incomplete or colcon build had errors | Rebuild package: `colcon build --packages-select kinect_ros2`; verify installation: `ros2 launch kinect_ros2 --help` |
| ROS 2 DDS discovery issues (nodes cannot communicate) | Middleware configuration or network isolation | Verify DDS works: `ros2 node list` (should show available nodes); check for network partitioning: `ROS_DOMAIN_ID=0 ros2 topic list`; examine DDS configuration in `~/.ros/fastdds_` |

### v2 Example Nodes (Troubleshooting)

| Symptom | Diagnosis | Resolution |
| :--- | :--- | :--- |
| `LIBUSB_ERROR_BUSY` when running examples | Two nodes accessing the Kinect USB device simultaneously | Ensure only `kinect_driver_node` uses the hardware. Verify all examples subscribe to topics, not direct device access. |
| `No module named 'kinect_ros2.examples...'` | Missing `__init__.py` in example subfolder or setup.py entry points not built | Run `touch kinect_ros2/examples/exXX/__init__.py`; verify console_scripts in setup.py; rebuild: `colcon build --packages-select kinect_ros2` |
| Example node starts but publishes nothing | Driver not running or driver not publishing expected topics | First, start the driver in a separate terminal: `ros2 run kinect_ros2 kinect_driver`; check that driver topics exist: `ros2 topic list \| grep camera` |
| QoS mismatch warnings from example nodes | Driver uses `BEST_EFFORT` QoS, but subscriber uses default `RELIABLE` | Examples should declare: `qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)` in subscription calls. |
| Hand detector never triggers | Threshold too low, object too far, or no depth data | Test with `ros2 topic echo /camera/depth/image_meters` to see if depth data arrives. Adjust parameter: `ros2 param set /hand_detector_node near_threshold_m 1.0` |
| Coloured point cloud empty in RViz2 | Time sync failure between depth and RGB, or wrong fixed frame | Increase `slop` parameter in coloured_cloud_node (default 0.05 seconds). Verify fixed frame is `kinect_depth_frame`. Check Color Transformer is set to RGB8. |
| Obstacle avoidance state never changes from STOP | Depth values all `inf` (invalid) or all zeros | Verify `/camera/depth/image_meters` is actively publishing: `ros2 topic hz /camera/depth/image_meters`. Adjust danger_distance_m if using turtlesim. |

### Hardware Limits & Behavior

| Symptom | Diagnosis | Resolution |
| :--- | :--- | :--- |
| Motor tilt not responding to commands | Motor stalled, gearbox jammed, or command validation failed | Manually test motor with: `ros2 topic pub /kinect/tilt_cmd std_msgs/Float32 "{data: 10.0}" --once`; inspect angle feedback: `ros2 topic echo /kinect/tilt_angle`; power-cycle Kinect if jammed |
| Accelerometer readings abnormal (non-sane gravity) | Sensor miscalibration or physical misalignment | Verify mounting: accelerometer Z-axis should read ~9.81 m/s² when static; recalibrate if needed by comparing to reference accelerometer |
| High power consumption (>15W) | Motor duty cycle, continuous operation without idle, or hardware fault | Monitor with: `watch -n 1 'usb-devices \| grep -A 20 Kinect'`; reduce motor movement frequency; check for mechanical resistance |

### Diagnostic Commands Reference

```bash
# 1. Verify USB enumeration
lsusb | grep -i kinect
lsusb -v -s 001:003  # Detailed info for specific device

# 2. Check udev rules
ls -la /etc/udev/rules.d/51-kinect.rules
sudo udevadm test /sys/bus/usb/devices/path-to-kinect-device

# 3. Test Python binding
python3 << 'EOF'
import freenect
dev = freenect.freenect_open_device(freenect.freenect_init(), 0)
if dev:
    print("Kinect connected and accessible")
    depth, ts = freenect.sync_get_depth(dev)
    print(f"Depth frame shape: {depth.shape}, dtype: {depth.dtype}")
else:
    print("Failed to open Kinect")
EOF

# 4. Monitor ROS 2 node lifecycle
ros2 node list
ros2 node info /kinect_driver_node
ros2 topic list -t | grep kinect

# 5. Inspect message contents
ros2 topic echo /kinect/camera/rgb/image_raw --once | head -20
ros2 topic echo /kinect/camera/rgb/camera_info --once

# 6. Benchmark frame rates
ros2 topic hz /kinect/camera/rgb/image_raw
ros2 topic hz /kinect/camera/depth/image_meters

# 7. Check CPU & memory usage
top -c | grep -E 'kinect|python'
ps aux | grep kinect_ros2

# 8. Inspect logs
tail -100 ~/.ros/log/latest/*/stdout
tail -100 ~/.ros/log/latest/*/stderr

# 9. ROS 2 system status
ros2 system prune
ros2 service list
```

## License

This project is licensed under the Apache License 2.0 - see the `LICENSE` file for details.

---

## API Reference & Module Documentation

### depth_utils Module

**Module Purpose:** Centralized utility functions for depth processing, coordinate transformations, and data validation.

**Functions:**

#### `convert_raw_to_meters(raw_disparity: np.ndarray) -> np.ndarray`

Converts raw 11-bit disparity values to metric depth in meters.

**Parameters:**
- `raw_disparity` (np.ndarray): Uint16 array of disparity values (0–2047)

**Returns:**
- `depth_meters` (np.ndarray): Float32 array of depth in meters

**Implementation:**
```python
def convert_raw_to_meters(raw_disparity):
    C1 = -0.0030711016
    C2 = 3.3309495161
    
    # Handle special cases (invalid pixels)
    valid_mask = (raw_disparity > 0) & (raw_disparity < 2047)
    depth = np.zeros_like(raw_disparity, dtype=np.float32)
    
    # Apply conversion formula
    depth[valid_mask] = 1.0 / (raw_disparity[valid_mask] * C1 + C2)
    
    return depth
```

---

#### `extract_roi(image: np.ndarray, roi_fraction: float) -> np.ndarray`

Extracts a centered region of interest from a depth image.

**Parameters:**
- `image` (np.ndarray): 2D image array (height × width)
- `roi_fraction` (float): Fraction of image center to extract (0.0 = full, 1.0 = center pixel)

**Returns:**
- `roi` (np.ndarray): Cropped region

---

#### `compute_min_distance(depth_image: np.ndarray, roi_fraction: float, min_valid: float) -> float`

Computes the minimum valid distance in a depth image's center ROI.

**Parameters:**
- `depth_image` (np.ndarray): Metric depth map (meters)
- `roi_fraction` (float): ROI size fraction
- `min_valid` (float): Ignore pixels below this distance (noise floor)

**Returns:**
- `min_distance` (float): Minimum distance in valid ROI, or `np.inf` if no valid pixels

---

### kinect_driver_node Module

**Main Node Class:** `KinectDriverNode(rclpy.node.Node)`

**Key Methods:**

#### `__init__()`

Initializes the node, configures ROS 2 parameters, opens the Kinect device, and starts frame acquisition.

**Raises:**
- `RuntimeError` if device cannot be opened

---

#### `update_parameters()`

Reads ROS 2 parameter values and applies them at runtime (if supported).

---

#### `frame_acquisition_loop()`

Blocking loop that continuously acquires frames from the Kinect and publishes them as ROS 2 messages. Runs in the node's executor.

---

### depth_alert_node Module

**Main Node Class:** `DepthAlertNode(rclpy.node.Node)`

**Event Handler Methods:**

#### `depth_callback(msg: sensor_msgs.Image)`

Processes incoming depth frames and publishes proximity alert state.

---

### tilt_control_node Module

**Main Node Class:** `TiltControlNode(rclpy.node.Node)`

**Key Methods:**

#### `detect_api_variant()`

Dynamically detects whether the libfreenect Python binding uses modern pointer semantics or legacy state objects. Sets `self.api_variant` to either `"pointer"` or `"state"`.

---

#### `tilt_cmd_callback(msg: std_msgs.Float32)`

Receives tilt angle commands and actuates the motor.

---

#### `publish_feedback()`

Periodically publishes current tilt angle and accelerometer telemetry.

---

## Development & Extension

### Adding Custom Nodes

To extend the ecosystem with additional analysis nodes:

1. **Create a new file** in `kinect_ros2/custom_node.py`
2. **Inherit from rclpy.node.Node** and implement the `main()` function
3. **Add entry point** to `setup.py`:
   ```python
   entry_points={
       'console_scripts': [
           'custom_node = kinect_ros2.custom_node:main',
       ],
   }
   ```
4. **Create corresponding launch file** in `launch/custom_full.launch.py`
5. **Test** with: `ros2 run kinect_ros2 custom_node`

---

### Custom Depth Processing Example

```python
# kinect_ros2/custom_segmentation.py

import rclpy
from sensor_msgs.msg import Image
from kinect_ros2.depth_utils import convert_raw_to_meters
import numpy as np

class SegmentationNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('segmentation')
        self.sub = self.create_subscription(
            Image, '/kinect/camera/depth/image_raw',
            self.process_depth, 10
        )
        self.pub = self.create_publisher(Image, '/segmented_depth', 10)
    
    def process_depth(self, msg):
        # Implement custom segmentation logic
        pass

def main():
    rclpy.init()
    node = SegmentationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Running Tests

Execute the integrated test suite to validate code quality:

```bash
# Run all tests (copyright, flake8, docstring validation)
colcon test --packages-select kinect_ros2

# Run specific test
colcon test --packages-select kinect_ros2 --ctest-args -R test_copyright

# View test results
colcon test-result --verbose
```

---

## Contributing Guidelines

We welcome contributions to improve kinect_ros2. Please follow these guidelines:

### Code Style

- **Python Version:** 3.10+
- **Linting:** PEP 8 (enforced by flake8)
- **Documentation:** PEP 257 docstring format
- **Type Hints:** Where applicable (ROS 2 nodes are dynamically typed)

### Contribution Workflow

1. **Fork** the repository on GitHub
2. **Create a feature branch:**
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. **Make changes** following PEP 8
4. **Add docstrings** to new functions/methods
5. **Test locally:**
   ```bash
   colcon build --packages-select kinect_ros2
   colcon test --packages-select kinect_ros2
   ```
6. **Commit with descriptive messages:**
   ```bash
   git commit -m "Add feature: description of changes"
   ```
7. **Push to your fork:**
   ```bash
   git push origin feature/your-feature-name
   ```
8. **Open a Pull Request** with a clear description of changes and motivation

### Reporting Issues

If you encounter bugs or have feature requests, use the GitHub issue tracker:

**Minimal Bug Report Template:**
```
### Title
Clear, concise description of the issue.

### Environment
- ROS 2 Version: `ros2 --version`
- Ubuntu Version: `lsb_release -a`
- Kinect Device: Xbox 360 v1 (confirm model/revision)

### Steps to Reproduce
1. Launch: `ros2 launch kinect_ros2 ...`
2. Observe: Describe the problem behavior

### Expected Behavior
What should happen

### Actual Behavior
What actually happens

### Logs
Paste relevant error messages from `~/.ros/log/latest/`

### Screenshots
If applicable, include images or RViz snapshots
```

---

## Acknowledgements & References

### Libraries & Frameworks

- **OpenKinect (`libfreenect`):** Low-level hardware driver for Kinect sensors
- **ROS 2 Jazzy:** Middleware and robotics framework
- **cv_bridge:** OpenCV ↔ ROS image bridging
- **numpy:** Efficient numerical operations

### Community

- The OpenKinect community for reverse-engineering and open-source Kinect support
- The ROS 2 community for the comprehensive middleware stack
- All contributors and testers who have improved this package

### References

1. [OpenKinect libfreenect Documentation](https://github.com/OpenKinect/libfreenect)
2. [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
3. [ROS 2 Message Types](https://docs.ros2.org/latest/api/sensor_msgs/index-msg.html)
4. [PEP 8 Style Guide](https://www.python.org/dev/peps/pep-0008/)

---

## Maintainance & Support

**Maintainer:** jaideepchouhan  
**Contact:** jaideepchouhan123@gmail.com  
**Repository:** https://github.com/JaideepChouhan/kinect_ros2  
**Issues:** https://github.com/JaideepChouhan/kinect_ros2/issues

---

## Release Notes

### Version 1.0.0 (Initial Release)

**Features:**
- Full RGB-D dual-stream acquisition
- Tilt motor control with feedback
- Accelerometer telemetry
- Proximity-based obstacle detection
- Launch files for multiple deployment scenarios
- RViz 2 visualization configuration
- Comprehensive documentation and examples

**Known Limitations:**
- Depth range limited to ~0.5–4.0 m practical for most surfaces
- No built-in point cloud generation (rely on downstream converters)
- Single-device operation (multi-Kinect support via launch parameter multiplexing)

**Future Roadmap:**
- Point cloud publisher integration
- Depth filtering and smoothing options
- Dynamic parameter reconfiguration service
- Multi-Kinect orchestration framework

### Version 2.0.0 (Learning Platform Release)

**Major Features:**
- ✅ **Six Example Nodes:** Complete reference implementations for common tasks
  - RGB annotation & FPS overlay (Ex01)
  - Distance measurement via percentile (Ex02)
  - Depth distribution histogram (Ex03)
  - Hand/blob detection (Ex04)
  - Sensor fusion (RGB + depth → XYZRGB point cloud) (Ex05)
  - Obstacle avoidance state machine (Ex06)
- ✅ **KinectDevice Context Manager:** Clean Python API for direct hardware access
- ✅ **Centralized Parameters:** `examples_params.yaml` for consistent configuration
- ✅ **Enhanced Launch Orchestration:** Staggered node startup to prevent CPU spikes
- ✅ **Automated Test Suite:** Unit and integration tests for core functionality
- ✅ **Improved Documentation:** Per-node descriptions, launch file details, troubleshooting

**New Dependencies:**
- `cv_bridge` (for RGB annotation)
- `message_filters` (for time synchronization in sensor fusion)
- `scipy` (for connected components in hand detection)
- `opencv-python` (for example node image processing)

**Breaking Changes:**
- None. v2 is fully backward-compatible with v1. Existing v1 nodes continue to work.

**Deprecations:**
- None.

**Known Issues:**
- Coloured point cloud sync may drop frames if depth and RGB arrive with >50ms skew
- Hand detector sensitive to lighting conditions; may require threshold tuning

**Contributors:**
- Community contributions for example nodes and documentation enhancements

---

## Conclusion & Next Steps

**kinect_ros2 v2.0** establishes a comprehensive learning and prototyping platform for Kinect-based robotics projects. The six example nodes provide working code patterns for:

1. **Real-time visualization** (RGB annotation, histograms)
2. **Distance measurement** (percentile-based proximity)
3. **Blob detection** (hand proximity without ML)
4. **Sensor fusion** (RGB-D point cloud registration)
5. **Reactive control** (state machine navigation)

### For Users

- **Start Small:** Run individual examples with `ros2 run` to understand core patterns
- **Customize:** Modify example nodes to suit your use case
- **Integrate:** Subscribe to driver topics from your own nodes
- **Report Issues:** Use GitHub issues for bugs and feature requests

### For Contributors

We welcome contributions:
- **Example Nodes:** Submit new examples showcasing Kinect + robotics tasks
- **Documentation:** Enhance guides, add tutorials, clarify APIs
- **Performance:** Optimize computation paths, reduce latency
- **Testing:** Expand test coverage, validate edge cases
- **Ports:** Support additional ROS 2 distributions or hardware variants

**Contribution Process:** Fork → feature branch → local test → pull request (see Contributing Guidelines above)

### Vision

kinect_ros2 aims to be the reference implementation for Kinect v1 in ROS 2, bridging legacy hardware with modern robotics software. The example-driven approach lowers barriers to entry for students, researchers, and hobbyists exploring computer vision and mobile robotics.

---

**Thank you for using kinect_ros2!** 🎉
