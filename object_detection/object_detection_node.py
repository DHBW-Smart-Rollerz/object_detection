#! /usr/bin/env python3

# Copyright (c) 2024 Smart Rollerz e.V. All rights reserved.

import cv2
import cv_bridge
import numpy as np
import rclpy
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, UInt32
from timing.timer import Timer

from object_detection.detector import SSD

IMAGE_TOPIC = "/camera/image/undistorted"
OBJECT_TOPIC = "/object_detection/object"
SIGN_TOPIC = "/object_detection/sign"
DEBUG_IMAGE_TOPIC = "/object_detection/debug/image"
STATE_MACHINE_TOPIC = "/state_machine/debug/state"
ACTIVE = True
DT = 0.0002
CONFIG_PATH = "config/model.yaml"


class ObjectDetectionNode(Node):
    """SSD object detection node."""

    def __init__(self):
        """Initialize the object detection node."""
        super().__init__("ssd_node")
        self.param = self.declare_parameters(
            namespace="",
            parameters=[
                ("debug", False),
                ("image_topic", IMAGE_TOPIC),
                ("object_topic", OBJECT_TOPIC),
                ("sign_topic", SIGN_TOPIC),
                ("debug_image_topic", DEBUG_IMAGE_TOPIC),
                ("state_machine_topic", STATE_MACHINE_TOPIC),
                ("active", ACTIVE),
                ("dt", DT),
                ("config_path", CONFIG_PATH),
            ],
        )
        self.init_properties()
        self.init_pub_sub()

        # Constants
        self.package_path = get_package_share_directory("object_detection")

        self.start_mode_state = 0
        self.start_ctr = 0

        self.state_machine_mode = None
        self.latest_image = None

        self.cv_bridge = cv_bridge.CvBridge()
        self.ssd = SSD.SSD(self)

        self.loop_timer = self.create_timer(self.dt, self.loop)

    def init_pub_sub(self):
        """Initialize publishers and subscribers."""
        self.object_detection_object_publisher = self.create_publisher(
            Float32MultiArray, self.object_topic, QoSProfile(depth=10)
        )
        self.object_detection_sign_publisher = self.create_publisher(
            Float32MultiArray, self.sign_topic, QoSProfile(depth=10)
        )
        self.debug_publisher = self.create_publisher(
            Image, self.debug_image_topic, QoSProfile(depth=10)
        )

        self.image_subscriber = self.create_subscription(
            Image, self.image_topic, self.camera_image_callback, 10
        )
        self.state_machine_subscriber = self.create_subscription(
            UInt32, self.state_machine_topic, self.state_machine_callback, 10
        )

    def init_properties(self):
        """Set properties from parameters."""
        for param in self.param:
            setattr(
                self.__class__,
                param.name,
                property(lambda self, n=param.name: self.get_parameter(n).value),
            )

    def loop(self):
        """Perception loop."""
        if self.latest_image is None:
            return

        with Timer(name="prediction_timer", filter_strength=40):
            result, mapped_signs, mapped_objects, result_img = self.ssd.get_result(
                self.latest_image, debug=self.debug
            )
        Timer().print()

        if self.start_mode_state == 2:
            self.start_ctr += 1
            if self.start_ctr > 150:
                self.start_mode_state = 3
        else:
            if result and mapped_objects:
                publish_msg = self.create_float32_multi_array([mapped_objects[0]])
                if publish_msg:
                    self.get_logger().debug(result)
                    self.object_detection_object_publisher.publish(publish_msg)

            if result and mapped_signs:
                publish_msg = self.create_float32_multi_array([mapped_signs[0]])
                if publish_msg:
                    self.get_logger().debug(result)
                    self.object_detection_sign_publisher.publish(publish_msg)

        if self.debug:
            self.debug_publisher.publish(
                self.cv_bridge.cv2_to_imgmsg(result_img, encoding="rgb8")
            )

    def create_float32_multi_array(self, objects: list) -> Float32MultiArray:
        """
        Create a Float32MultiArray message from a list of objects.

        Arguments:
            objects -- list of objects

        Returns:
            Float32MultiArray message
        """
        flat_objects = []
        for obj in objects:
            flat_objects.extend(
                [
                    float(obj["id"]),
                    float(obj["bottom-left-distance"]["x"]),
                    float(obj["bottom-left-distance"]["y"]),
                    float(obj["bottom-right-distance"]["x"]),
                    float(obj["bottom-right-distance"]["y"]),
                    obj["score"],
                ]
            )
        float_array_msg = Float32MultiArray()
        float_array_msg.data = flat_objects
        return float_array_msg

    def camera_image_callback(self, image_msg: Image):
        """
        Callback for camera image.

        Arguments:
            image_msg -- Image message
        """
        if not self.active:
            return
        current_image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(
            (image_msg.height, image_msg.width)
        )
        current_image = cv2.cvtColor(current_image, cv2.COLOR_GRAY2BGR)
        self.latest_image = current_image

    def state_machine_callback(self, state: UInt32):
        """
        Callback for state machine.

        Arguments:
            state -- UInt32 message
        """
        if not self.active:
            return
        self.state_machine_mode = state.data
        if self.start_mode_state == 0 and state.data == 10:
            self.start_mode_state = 1
        if self.start_mode_state == 1 and state.data == 11:
            self.start_mode_state = 2


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
