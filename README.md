# carla_camera_pipeline

CARLA RGB camera → ROS 2 topics → PNG dataset and/or MCAP rosbag.

## Topics
| Topic | Type |
|---|---|
| `/carla/camera/rgb` | `sensor_msgs/Image` (bgr8, sim-time stamped) |
| `/carla/camera/camera_info` | `sensor_msgs/CameraInfo` (intrinsics from FOV) |
| `/carla/ego/state` | `std_msgs/Float64MultiArray`: sim_time, x, y, z, roll, pitch, yaw, speed, steer, throttle, brake (CARLA frame) |
| `/clock` | `rosgraph_msgs/Clock` (sim time) |

## Setup (on the DCV host, alongside CARLA)
```bash
# CARLA Python API — version MUST match the server
pip install carla==<your server version>      # or add PythonAPI/carla/dist/*.whl / .egg

sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-rosbag2-storage-mcap

mkdir -p ~/ros2_ws/src && cp -r carla_camera_pipeline ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

## Run
```bash
# Terminal 1: CARLA (headless unless you want to watch in DCV)
./CarlaUE4.sh -RenderOffScreen

# Terminal 2: pipeline
ros2 launch carla_camera_pipeline pipeline.launch.py                       # save PNG dataset
ros2 launch carla_camera_pipeline pipeline.launch.py record_bag:=true      # also record MCAP bag
ros2 launch carla_camera_pipeline pipeline.launch.py save_images:=false record_bag:=true
```
Edit `config/pipeline.yaml` (resolution, FOV, mount, rate, output dir), or pass
`params_file:=/path/to/your.yaml`.

## Output
```
<output_dir>/run_YYYYmmdd_HHMMSS/
  images/000000.png ...
  frames.csv          index, image, sim_time, x, y, z, roll, pitch, yaw, speed, steer, throttle, brake
  camera_info.json    K, D, P, width, height
```

## Notes
- CARLA runs in **synchronous mode**; the publisher calls `world.tick()` every step, so frames are deterministic and never skipped. Ctrl+C restores async mode and destroys the actors.
- Raw 1280×720 at 20 Hz ≈ 55 MB/s. Use `image_format: jpg` or `save_every_n` to cut disk usage.
- Other nodes that want sim time: run with `use_sim_time:=true`.
- Pull data off the DCV host with `rsync -avP host:~/carla_dataset ./`.

## Object detection + LiDAR depth node

`object_detection` preserves the KITTI demo algorithm: YOLO on RGB, LiDAR
projection, then median rectified camera Z and depth IQR inside each box.
Depth is visible-surface camera Z in metres, **not** object-center position or
Euclidean range. Boxes may contain background/ground; overlapping boxes can
reuse points. No KITTI file paths or CARLA Python API are needed by this node.

### Agreed input contract

| Input | ROS type | Contract |
|---|---|---|
| `/carla/camera/rgb` | `sensor_msgs/msg/Image` | RGB-convertible, undistorted image, nonzero simulation timestamp; frame `camera_rgb` |
| `/carla/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Publish for every image with matching stamp, frame and resolution; valid R/P; zero distortion |
| `/carla/lidar/points` | `sensor_msgs/msg/PointCloud2` | float XYZ in metres, frame `lidar`; timestamp from the same CARLA simulation clock |
| `/tf_static` (rigid mounting) or `/tf` (moving mounting) | `tf2_msgs/msg/TFMessage` | Actual transform chain connecting `lidar` and `camera_rgb`, available at the image timestamp |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation clock, required with the default `use_sim_time: true` |

`camera_rgb` must mean a ROS **optical** frame: X right, Y down, Z forward.
`lidar` must use ROS axes (X forward, Y left, Z up), with its actual mounting
pose represented in TF. CARLA/Unreal raw left-handed coordinates must be
converted upstream for both point coordinates and transforms. A frame label
alone does not convert coordinates. The current camera publisher uses the
name `camera_rgb`, but does not publish the missing LiDAR or TF.

TF lookup requests **target camera_rgb, source lidar**. It maps LiDAR XYZ into
the reference optical camera frame; CameraInfo.R then rectifies these points,
and CameraInfo.P projects them. Do not apply rectification twice or reuse KITTI
calibration for CARLA. No placeholder extrinsics are supplied.

The three sensor inputs use best-effort, volatile sensor QoS and approximate
header-stamp synchronization (default tolerance 0.02 s, bounded queue of 5).
Matching CARLA frame timestamps are preferred. Approximate matching does not
motion-compensate a rotating LiDAR scan. Under load frames can be dropped;
this node does not promise lossless offline processing. Missing/unsynchronized
inputs produce a warning; missing TF skips the frame without publishing a
fabricated depth. Static transforms should be available before sensor streams.

### Outputs

| Topic | Type | Content |
|---|---|---|
| `/perception/detections_depth` | `std_msgs/msg/String` | JSON report, schema_version 1 |
| `/perception/debug/projection` | `sensor_msgs/msg/Image` | RGB with depth-colored LiDAR points |
| `/perception/debug/image` | `sensor_msgs/msg/Image` | Projection plus boxes, scores, depth and point count |

JSON includes the source image `header` (stamp and optical frame), `lidar_stamp`,
mode, image size, point counts, model/settings and `detections`. Each detection
contains `class_id`, `class_name`, `confidence`, `bbox_xyxy` in original pixels,
`depth_roi_xyxy`, `num_points`, `depth_m`, `depth_iqr_m`, and `status` (`ok` or
`insufficient_points`). Too few points produces JSON `null` depth/IQR. IQR is
spread, not confidence. An empty detection array is a valid processed frame.
Both output images retain the source image header. In projection-only mode,
no model loads and the detection array is empty.

### Build and run

Install ROS dependencies through rosdep in your workspace; the added ROS
requirements are message_filters, sensor_msgs_py and tf2_ros, plus Pillow.
Install `ultralytics` in a Python environment compatible with your ROS Python,
and provide YOLO weights (the default `yolo11n.pt` may download on first use).

```bash
# From the workspace containing this package:
colcon build --symlink-install --packages-select carla_camera_pipeline
source install/setup.bash
ros2 launch carla_camera_pipeline detection.launch.py

# No YOLO dependency required for checking synchronization, TF and projection:
ros2 run carla_camera_pipeline object_detection --ros-args \
  --params-file /path/to/config/detection.yaml -p projection_only:=true
```

The launch file starts only the detector, so remote input nodes can be developed
independently. Edit `config/detection.yaml` or pass `params_file:=/path/to.yaml`.
Input names, model/device, confidence, inference size, min_points, box_margin,
color scale, sync tolerance and queue size are startup parameters.
`output_dir` defaults to empty (no disk writes); set a path to save projection
PNG, annotated PNG and JSON per processed frame in a new timestamped directory.
For remote ROS discovery, both sides must have compatible discovery/network
settings and the same ROS_DOMAIN_ID. The remote CARLA server itself does not
make ROS topics available without a publisher/bridge.

Core algorithm checks: `python3 -m unittest discover -s tests -v`.
Manual ROS smoke check (synthetic sensors, static TF, projection-only output):
`ROS_DOMAIN_ID=137 python3 tests/ros_detection_smoke.py`.
This does not exercise actual YOLO weights or remote CARLA input.
