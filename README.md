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
