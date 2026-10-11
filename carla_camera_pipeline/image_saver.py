#!/usr/bin/env python3
"""
Subscribe to the CARLA camera topics and write a training-ready dataset.
"""
import csv
import json
import os
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray

from carla_camera_pipeline.state_fields import STATE_FIELDS


def stamp_key(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def time_key(t: float) -> int:
    return int(round(t * 1e9))


class ImageSaver(Node):
    def __init__(self):
        super().__init__('carla_image_saver')

        self.declare_parameters('', [
            ('output_dir', os.path.expanduser('~/carla_dataset')),
            ('run_name', ''),
            ('image_format', 'png'),   # png (lossless) or jpg
            ('jpg_quality', 95),
            ('save_every_n', 1),       # 1 = every frame, 5 = every 5th frame
            ('max_pending_states', 100),
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731

        run_name = p('run_name') or datetime.now().strftime('run_%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(p('output_dir'), run_name)
        self.image_dir = os.path.join(self.run_dir, 'images')
        os.makedirs(self.image_dir, exist_ok=True)

        self.ext = p('image_format').lower().lstrip('.')
        if self.ext not in ('png', 'jpg', 'jpeg'):
            raise ValueError("image_format must be 'png' or 'jpg'")
        self.imwrite_params = (
            [cv2.IMWRITE_JPEG_QUALITY, p('jpg_quality')] if self.ext in ('jpg', 'jpeg') else [])
        self.save_every_n = max(1, p('save_every_n'))
        self.max_pending = p('max_pending_states')

        self.bridge = CvBridge()
        self.states = {}         # sim-time ns -> list of state values
        self.received = 0
        self.saved = 0
        self.info_written = False

        self.csv_file = open(os.path.join(self.run_dir, 'frames.csv'), 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['index', 'image', *STATE_FIELDS])

        self.create_subscription(Image, 'carla/camera/rgb', self.on_image, 10)
        self.create_subscription(CameraInfo, 'carla/camera/camera_info', self.on_info, 10)
        self.create_subscription(Float64MultiArray, 'carla/ego/state', self.on_state, 10)

        self.get_logger().info(f'Saving dataset to {self.run_dir}')


    """
    On info: is only called on the first iteration and generates the model dictionary.
    On state: contains all our labels under the key denoted by a timestamp
    On image: loads image frames, writes to a .png and queries labels defined through on state via key
    """
    def on_info(self, msg: CameraInfo):
        if self.info_written:
            return
        info = {
            'width': msg.width,
            'height': msg.height,
            'frame_id': msg.header.frame_id,
            'K': list(msg.k),
            'D': list(msg.d),
            'P': list(msg.p),
            'distortion_model': msg.distortion_model,
        }
        with open(os.path.join(self.run_dir, 'camera_info.json'), 'w') as f:
            json.dump(info, f, indent=2)
        self.info_written = True

    def on_state(self, msg: Float64MultiArray):
        self.states[time_key(msg.data[0])] = list(msg.data)
        # Bound memory if images stop arriving.
        while len(self.states) > self.max_pending:
            self.states.pop(next(iter(self.states)))

    def on_image(self, msg: Image):
        self.received += 1
        if (self.received - 1) % self.save_every_n:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        name = f'{self.saved:06d}.{self.ext}'
        if not cv2.imwrite(os.path.join(self.image_dir, name), img, self.imwrite_params):
            self.get_logger().error(f'Failed to write {name}')
            return

        key = stamp_key(msg.header.stamp)
        state = self.states.pop(key, None)
        if state is None:
            # State not received (yet); keep the sim time, leave the rest blank.
            state = [key / 1e9] + [''] * (len(STATE_FIELDS) - 1)

        self.csv.writerow([self.saved, f'images/{name}', *state])
        self.saved += 1
        if self.saved % 100 == 0:
            self.csv_file.flush()
            self.get_logger().info(f'Saved {self.saved} frames')

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f'Done: {self.saved} frames in {self.run_dir}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
