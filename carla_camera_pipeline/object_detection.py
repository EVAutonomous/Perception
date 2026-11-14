"""Synchronized RGB + LiDAR -> YOLO boxes and median camera-Z depth."""
import json
from datetime import datetime
from pathlib import Path
import time

import message_filters
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException

from .detection_core import (project_lidar, detect_objects, estimate_depth,
                             draw_projection, draw_detections, transform_matrix)


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        defaults = {
            'image_topic': '/carla/camera/rgb',
            'camera_info_topic': '/carla/camera/camera_info',
            'points_topic': '/carla/lidar/points',
            'model': 'yolo11n.pt', 'device': 'cpu', 'conf': 0.25, 'imgsz': 640,
            'min_points': 3, 'box_margin': 0.0, 'color_max_depth': 60.0,
            'projection_only': False, 'sync_slop': 0.02, 'queue_size': 5,
            'output_dir': '',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        p = self.p
        if not (0 < p['conf'] <= 1 and 0 <= p['box_margin'] < .5 and p['min_points'] > 0
                and p['imgsz'] > 0 and p['color_max_depth'] > 0
                and p['sync_slop'] >= 0 and p['queue_size'] > 0):
            raise ValueError('Invalid detection or synchronization parameters')
        self.model = None
        if not p['projection_only']:
            from ultralytics import YOLO
            self.model = YOLO(p['model'])
        self.output = None
        if p['output_dir']:
            self.output = Path(p['output_dir']).expanduser() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.output.mkdir(parents=True, exist_ok=False)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.result_pub = self.create_publisher(String, '/perception/detections_depth', 10)
        self.projection_pub = self.create_publisher(Image, '/perception/debug/projection', 1)
        self.image_pub = self.create_publisher(Image, '/perception/debug/image', 1)
        self.subs = [message_filters.Subscriber(self, cls, p[key], qos_profile=qos_profile_sensor_data)
                     for cls, key in [(Image, 'image_topic'), (PointCloud2, 'points_topic'),
                                      (CameraInfo, 'camera_info_topic')]]
        self.sync = message_filters.ApproximateTimeSynchronizer(
            self.subs, p['queue_size'], p['sync_slop'])
        self.sync.registerCallback(self.on_inputs)
        self.last_input = time.monotonic()
        self.create_timer(10.0, self.check_input)
        self.get_logger().info('Waiting for synchronized RGB, PointCloud2, CameraInfo and camera-from-LiDAR TF')

    def check_input(self):
        if time.monotonic() - self.last_input > 10:
            self.get_logger().warning('No synchronized inputs: check topics, timestamps and sensor QoS')

    def on_inputs(self, image_msg, cloud_msg, info):
        self.last_input = time.monotonic()
        try:
            self.process(image_msg, cloud_msg, info)
        except (TransformException, ValueError, RuntimeError) as exc:
            self.get_logger().warning(f'Frame skipped: {exc}', throttle_duration_sec=5.0)

    def process(self, image_msg, cloud_msg, info):
        if not image_msg.header.frame_id or not cloud_msg.header.frame_id:
            raise ValueError('Image and cloud require nonempty frame_id')
        if info.header.frame_id != image_msg.header.frame_id:
            raise ValueError('CameraInfo and Image must share the camera optical frame')
        if (info.width, info.height) != (image_msg.width, image_msg.height):
            raise ValueError('CameraInfo resolution does not match Image')
        if any(abs(d) > 1e-12 for d in info.d):
            raise ValueError('Rectified/undistorted image required; nonzero distortion is unsupported')
        projection = np.asarray(info.p).reshape(3, 4)
        rectification = np.asarray(info.r).reshape(3, 3)
        if (not np.isfinite(projection).all() or not np.isfinite(rectification).all()
                or projection[0, 0] <= 0 or projection[1, 1] <= 0
                or not np.allclose(rectification @ rectification.T, np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(rectification), 1.0, atol=1e-5)):
            raise ValueError('Invalid CameraInfo P/R')
        # A zero stamp in TF means latest, which is unsafe for dynamic transforms.
        if image_msg.header.stamp.sec == 0 and image_msg.header.stamp.nanosec == 0:
            raise ValueError('Image stamp must be nonzero')
        tf = self.tf_buffer.lookup_transform(
            image_msg.header.frame_id, cloud_msg.header.frame_id,
            Time.from_msg(image_msg.header.stamp)).transform
        t, q = tf.translation, tf.rotation
        calibration = {'P2': projection, 'R0_rect': rectification,
                       'Tr_velo_to_cam': transform_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])}
        points = point_cloud2.read_points_numpy(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
        image = PILImage.fromarray(self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8'))
        projected = project_lidar(points, calibration, (image.height, image.width))
        p = self.p
        detections = [] if self.model is None else estimate_depth(
            detect_objects(image, self.model, p['conf'], p['device'], p['imgsz']),
            projected, p['min_points'], p['box_margin'])
        stamp = image_msg.header.stamp
        report = dict(schema_version=1, header=dict(stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec),
                      frame_id=image_msg.header.frame_id),
                      lidar_stamp=dict(sec=cloud_msg.header.stamp.sec, nanosec=cloud_msg.header.stamp.nanosec),
                      mode='projection_only' if self.model is None else 'yolo_depth',
                      image_size=[image.width, image.height], lidar_points=len(points),
                      projected_points=len(projected),
                      depth_definition='median rectified reference-camera Z in metres; visible surface, not object center',
                      model=None if self.model is None else p['model'],
                      confidence_threshold=p['conf'], min_points=p['min_points'],
                      box_margin=p['box_margin'], detections=detections)
        payload = json.dumps(report, allow_nan=False)
        self.result_pub.publish(String(data=payload))
        overlay = draw_projection(image, projected, p['color_max_depth'])
        annotated = draw_detections(overlay, detections)
        for publisher, canvas in [(self.projection_pub, overlay), (self.image_pub, annotated)]:
            msg = self.bridge.cv2_to_imgmsg(np.asarray(canvas), encoding='rgb8')
            msg.header = image_msg.header
            publisher.publish(msg)
        if self.output:
            name = f'{stamp.sec}_{stamp.nanosec:09d}'
            overlay.save(self.output / f'{name}_projection.png')
            annotated.save(self.output / f'{name}_detections.png')
            (self.output / f'{name}.json').write_text(payload + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectDetection()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
