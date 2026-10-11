#!/usr/bin/env python3
"""
Spawn an ego vehicle + RGB camera in CARLA and publish frames to ROS 2.
"""
import math
import queue

import carla
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray

from carla_camera_pipeline.state_fields import STATE_FIELDS  # noqa: F401  (documents /carla/ego/state layout)


def sim_time_to_msg(t: float) -> Time:
    """Convert CARLA sim seconds (float) to a ROS Time message."""
    ns = int(round(t * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class CarlaCameraPublisher(Node):
    def __init__(self):
        super().__init__('carla_camera_publisher')

        self.declare_parameters('', [
            ('host', 'localhost'),
            ('port', 2000),
            ('timeout', 10.0),
            ('tm_port', 8000),
            ('fixed_delta_seconds', 0.05),
            ('vehicle_filter', 'vehicle.tesla.model3'),
            ('spawn_point_index', 0),
            ('autopilot', True),
            ('image_width', 1280),
            ('image_height', 720),
            ('fov', 90.0),
            ('camera_x', 1.5),
            ('camera_y', 0.0),
            ('camera_z', 2.4),
            ('camera_pitch', 0.0),
            ('frame_id', 'camera_rgb'),
            ('sensor_timeout', 2.0),
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731

        self.width = p('image_width')
        self.height = p('image_height')
        self.fov = p('fov')
        self.frame_id = p('frame_id')
        self.sensor_timeout = p('sensor_timeout')
        self.dt = p('fixed_delta_seconds')

        self.bridge = CvBridge()
        self.image_queue: queue.Queue = queue.Queue()
        self.ego = None
        self.camera = None
        self.original_settings = None
        self.tm = None

        try:
            self._setup_carla(p)
        except Exception:
            self.cleanup()
            raise

        self.image_pub = self.create_publisher(Image, 'carla/camera/rgb', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'carla/camera/camera_info', 10)
        self.state_pub = self.create_publisher(Float64MultiArray, 'carla/ego/state', 10)
        self.clock_pub = self.create_publisher(Clock, 'clock', 10)
        self.camera_info = self._build_camera_info()

        # Each timer firing advances the simulation by exactly one fixed step.
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f'Publishing {self.width}x{self.height} @ {1.0 / self.dt:.0f} Hz (sim time)')

    def _setup_carla(self, p):
        client = carla.Client(p('host'), p('port'))
        client.set_timeout(p('timeout'))
        self.world = client.get_world()

        # Synchronous mode with a fixed step -> deterministic, frame-aligned data.
        self.original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.dt
        self.world.apply_settings(settings)

        # Traffic Manager must match, or autopilot desyncs from the world.
        self.tm = client.get_trafficmanager(p('tm_port'))
        self.tm.set_synchronous_mode(True)

        bp_lib = self.world.get_blueprint_library()

        # Ego vehicle: try spawn points starting at the configured index.
        vehicle_bps = bp_lib.filter(p('vehicle_filter'))
        if not vehicle_bps:
            raise RuntimeError(f"No vehicle blueprint matches '{p('vehicle_filter')}'")
        vehicle_bp = vehicle_bps[0]
        vehicle_bp.set_attribute('role_name', 'ego')

        spawn_points = self.world.get_map().get_spawn_points()
        start = p('spawn_point_index') % len(spawn_points)
        for sp in spawn_points[start:] + spawn_points[:start]:
            self.ego = self.world.try_spawn_actor(vehicle_bp, sp)
            if self.ego is not None:
                break
        if self.ego is None:
            raise RuntimeError('Could not spawn ego vehicle (all spawn points blocked)')
        if p('autopilot'):
            self.ego.set_autopilot(True, p('tm_port'))

        # RGB camera rigidly attached to the ego vehicle.
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(self.width))
        cam_bp.set_attribute('image_size_y', str(self.height))
        cam_bp.set_attribute('fov', str(self.fov))
        cam_tf = carla.Transform(
            carla.Location(x=p('camera_x'), y=p('camera_y'), z=p('camera_z')),
            carla.Rotation(pitch=p('camera_pitch')))
        self.camera = self.world.spawn_actor(cam_bp, cam_tf, attach_to=self.ego)

        # The callback runs on CARLA's thread; only enqueue here. Publishing
        # happens on the ROS executor thread in step().
        self.camera.listen(self.image_queue.put)

        self.world.tick()  # let actors settle
        self.get_logger().info(f'Spawned {vehicle_bp.id} with RGB camera')

    # Define pinhole camera model
    def _build_camera_info(self) -> CameraInfo:
        # Focal lengths
        fx = self.width / (2.0 * math.tan(math.radians(self.fov) / 2.0))
        fy = fx

        # Image center
        cx, cy = self.width / 2.0, self.height / 2.0
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width, info.height = self.width, self.height
        info.distortion_model = 'plumb_bob'

        # Distortion Coeff, since Carla is a perfect pinhole we don't need any
        info.d = [0.0] * 5

        # Intrinsic Matrix
        # This assumes the ROS optical frame, so we have to do some remapping with our Carla images
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        # Rectification Rotation, identity for monocular camera
        # These matrices are virtual rotations of cameras used for stereo matching, we can look deeper into it if we want to spawn 2+ cameras.
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Projection Matrix

        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _wait_for_frame(self, frame: int):
        """Pull images until the one matching this tick arrives (drop stale ones)."""
        while True:
            image = self.image_queue.get(timeout=self.sensor_timeout)
            if image.frame >= frame:
                return image

    def step(self):
        frame = self.world.tick()
        try:
            image = self._wait_for_frame(frame)
        except queue.Empty:
            self.get_logger().warn(f'No camera frame received for tick {frame}')
            return

        stamp = sim_time_to_msg(image.timestamp)
        self.clock_pub.publish(Clock(clock=stamp))

        # BGRA byte buffer -> HxWx4 array -> drop alpha -> contiguous BGR.
        bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
            image.height, image.width, 4)
        img_msg = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(bgra[:, :, :3]), 'bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        self.camera_info.header.stamp = stamp
        self.info_pub.publish(self.camera_info)

        self.state_pub.publish(self._ego_state(image.timestamp))

    def _ego_state(self, sim_time: float) -> Float64MultiArray:
        """Ego transform, speed and applied control, kept in CARLA's frame."""
        tf = self.ego.get_transform()
        v = self.ego.get_velocity()
        c = self.ego.get_control()
        return Float64MultiArray(data=[
            sim_time,
            tf.location.x, tf.location.y, tf.location.z,
            tf.rotation.roll, tf.rotation.pitch, tf.rotation.yaw,
            math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2),
            c.steer, c.throttle, c.brake,
        ])

    def cleanup(self):
        """Destroy actors and restore async mode so the server doesn't hang."""
        try:
            if self.camera is not None:
                self.camera.stop()
                self.camera.destroy()
                self.camera = None
            if self.ego is not None:
                self.ego.destroy()
                self.ego = None
            if self.tm is not None:
                self.tm.set_synchronous_mode(False)
            if self.original_settings is not None:
                self.world.apply_settings(self.original_settings)
                self.original_settings = None
        except Exception as exc:  # server may already be gone
            self.get_logger().warn(f'Cleanup error: {exc}')

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CarlaCameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
