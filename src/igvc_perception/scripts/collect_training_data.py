#!/usr/bin/env python3
"""
Spawns/deletes an obstacle model in Gazebo at random poses, captures the
camera frame at each pose, and computes the ground-truth 2D bounding box
by projecting the obstacle's known 3D pose into the image plane.
Standalone script — not a spinning ROS2 node.
"""

import os
import random
import time

import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from cv_bridge import CvBridge


NUM_SAMPLES = 300
OUTPUT_DIR = os.path.expanduser('~/igvc_ws/yolo_dataset')
IMAGES_DIR = os.path.join(OUTPUT_DIR, 'images')
LABELS_DIR = os.path.join(OUTPUT_DIR, 'labels')

OBSTACLE_MODEL_SDF = os.path.expanduser('~/igvc_ws/models/barrel/model.sdf')  # <- your model
OBSTACLE_CLASS_ID = 0
OBSTACLE_HALF_SIZE = (0.2, 0.2, 0.4)  # <- measure your model, meters (x, y, z half-extents)

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FX, FY = 530.0, 530.0          # <- your camera intrinsics (check /camera/camera_info)
CX, CY = IMAGE_WIDTH / 2.0, IMAGE_HEIGHT / 2.0

CAMERA_POSITION = np.array([0.0, 0.0, 0.5])  # <- your camera's world pose
CAMERA_YAW = 0.0

SPAWN_X_RANGE = (1.0, 6.0)
SPAWN_Y_RANGE = (-2.0, 2.0)
SPAWN_Z = 0.0


class DatasetGenerator(Node):

    def __init__(self):
        super().__init__('dataset_generator')

        self.bridge = CvBridge()
        self.latest_image = None

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10
        )

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_client.wait_for_service(timeout_sec=10.0)
        self.delete_client.wait_for_service(timeout_sec=10.0)

        with open(OBSTACLE_MODEL_SDF, 'r') as f:
            self.model_xml = f.read()

    def _image_callback(self, msg: Image):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def spawn_obstacle(self, name, x, y, z):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = self.model_xml
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def delete_obstacle(self, name):
        request = DeleteEntity.Request()
        request.name = name
        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def wait_for_fresh_frame(self, timeout_sec=2.0):
        self.latest_image = None
        start = time.time()
        while self.latest_image is None and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_image

    def project_bbox(self, obj_x, obj_y, obj_z):
        hx, hy, hz = OBSTACLE_HALF_SIZE
        corners_world = [
            np.array([obj_x + sx, obj_y + sy, obj_z + sz])
            for sx in (-hx, hx) for sy in (-hy, hy) for sz in (0, 2 * hz)
        ]

        cos_yaw, sin_yaw = np.cos(-CAMERA_YAW), np.sin(-CAMERA_YAW)
        pixels = []
        for corner in corners_world:
            rel = corner - CAMERA_POSITION
            cx_ = rel[0] * cos_yaw - rel[1] * sin_yaw
            cy_ = rel[0] * sin_yaw + rel[1] * cos_yaw
            cz_ = rel[2]

            optical_x, optical_y, optical_z = -cy_, -cz_, cx_
            if optical_z <= 0.05:
                continue

            u = FX * (optical_x / optical_z) + CX
            v = FY * (optical_y / optical_z) + CY
            pixels.append((u, v))

        if not pixels:
            return None

        us, vs = [p[0] for p in pixels], [p[1] for p in pixels]
        x_min, x_max = max(min(us), 0), min(max(us), IMAGE_WIDTH)
        y_min, y_max = max(min(vs), 0), min(max(vs), IMAGE_HEIGHT)

        if x_max <= x_min or y_max <= y_min:
            return None
        return x_min, y_min, x_max, y_max

    def write_sample(self, index, image, bbox):
        image_path = os.path.join(IMAGES_DIR, f'{index:05d}.jpg')
        label_path = os.path.join(LABELS_DIR, f'{index:05d}.txt')
        cv.imwrite(image_path, image)

        x_min, y_min, x_max, y_max = bbox
        x_center = ((x_min + x_max) / 2.0) / IMAGE_WIDTH
        y_center = ((y_min + y_max) / 2.0) / IMAGE_HEIGHT
        box_w = (x_max - x_min) / IMAGE_WIDTH
        box_h = (y_max - y_min) / IMAGE_HEIGHT

        with open(label_path, 'w') as f:
            f.write(f'{OBSTACLE_CLASS_ID} {x_center:.6f} {y_center:.6f} {box_w:.6f} {box_h:.6f}\n')

    def run(self):
        os.makedirs(IMAGES_DIR, exist_ok=True)
        os.makedirs(LABELS_DIR, exist_ok=True)

        for i in range(NUM_SAMPLES):
            x = random.uniform(*SPAWN_X_RANGE)
            y = random.uniform(*SPAWN_Y_RANGE)
            name = f'obstacle_{i}'

            self.spawn_obstacle(name, x, y, SPAWN_Z)
            time.sleep(0.3)

            image = self.wait_for_fresh_frame()
            bbox = self.project_bbox(x, y, SPAWN_Z)

            if image is not None and bbox is not None:
                self.write_sample(i, image, bbox)
                self.get_logger().info(f'[{i+1}/{NUM_SAMPLES}] saved, bbox={bbox}')
            else:
                self.get_logger().warn(f'[{i+1}/{NUM_SAMPLES}] skipped (out of view)')

            self.delete_obstacle(name)
            time.sleep(0.2)


def main():
    rclpy.init()
    node = DatasetGenerator()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()