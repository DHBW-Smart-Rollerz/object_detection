#! /usr/bin/env python3

# Copyright (c) 2025 Smart Rollerz e.V. All rights reserved.

import cv2
import cv_bridge
import numpy as np
import rclpy
import rclpy.parameter
from rcl_interfaces.msg import SetParametersResult
import sensor_msgs
from rclpy.qos import QoSProfile
from smarty_utils.enums import NodeState
from smarty_utils.smarty_node import SmartyNode
from std_msgs.msg import Float32MultiArray
from timing.timer import Timer

from object_detection.detector import SSD

IMAGE_TOPIC = "/camera/image/undistorted"
OBJECT_TOPIC = "/object_detection/object"
SIGN_TOPIC = "/object_detection/sign"
DEBUG_IMAGE_TOPIC = "/object_detection/debug/image"
STATE_MACHINE_TOPIC = "/state_machine/debug/state"
ACTIVE = NodeState.ACTIVE.value
DT = 0.0002
CONFIG_PATH = "config/model.yaml"


class ObjectDetectionNode(SmartyNode):
    """SSD object detection node."""

    def __init__(self):
        """Initialize the object detection node."""
        super().__init__(
            "object_detection_node",
            "object_detection",
            node_parameters={
                # Subscriber topics
                "image_subscriber": IMAGE_TOPIC,
                # Publisher topics
                "object_publisher": OBJECT_TOPIC,
                "sign_publisher": SIGN_TOPIC,
                "debug_img_publisher": DEBUG_IMAGE_TOPIC,
                # Parameters
                "config_path": CONFIG_PATH,
                "dt": DT,
                "state": ACTIVE,
                "debug": False,
                # Overall threshold
                "detection_threshold": 0.6,
                # Individual class thresholds - defined explicitly
                "threshold_vehicle": 0.4,  # class_id 0
                "threshold_pedestrian": 0.4,  # class_id 1
                "threshold_stop_sign": 0.6,  # class_id 2
                "threshold_crosswalk_sign": 0.5,  # class_id 3
                "threshold_parking_sign": 0.6,  # class_id 4
                "threshold_thirty_sign": 0.6,  # class_id 5
                "threshold_no_thirty_sign": 0.6,  # class_id 6
                "threshold_go_left_sign": 0.3,  # class_id 7
                "threshold_go_right_sign": 0.3,  # class_id 8
                "threshold_intersection_right_of_way_sign": 0.4,  # class_id 9
                "threshold_intersection_grant_sign": 0.4,  # class_id 10
                "threshold_expressway_start": 0.5,  # class_id 11
                "threshold_expressway_end": 0.5,  # class_id 12
                "threshold_barred_area": 0.5,  # class_id 13
                "threshold_pedestrian_island_right": 0.5,  # class_id 14
                "threshold_no_passing_start": 0.5,  # class_id 15
                "threshold_no_passing_end": 0.5,  # class_id 16
            },
            subscribed_topics={
                "image_subscriber": (
                    sensor_msgs.msg.Image,
                    self.camera_image_callback,
                    QoSProfile(depth=10),
                ),
            },
            published_topics={
                "object_publisher": (Float32MultiArray, QoSProfile(depth=10)),
                "sign_publisher": (Float32MultiArray, QoSProfile(depth=10)),
                "debug_img_publisher": (sensor_msgs.msg.Image, QoSProfile(depth=10)),
            },
        )

        # Define class_id to parameter name mapping for runtime updates
        self.class_mappings = {
            0: "threshold_vehicle",
            1: "threshold_pedestrian",
            2: "threshold_stop_sign",
            3: "threshold_crosswalk_sign",
            4: "threshold_parking_sign",
            5: "threshold_thirty_sign",
            6: "threshold_no_thirty_sign",
            7: "threshold_go_left_sign",
            8: "threshold_go_right_sign",
            9: "threshold_intersection_right_of_way_sign",
            10: "threshold_intersection_grant_sign",
            11: "threshold_expressway_start",
            12: "threshold_expressway_end",
            13: "threshold_barred_area",
            14: "threshold_pedestrian_island_right",
            15: "threshold_no_passing_start",
            16: "threshold_no_passing_end",
        }

        # Create confidence thresholds dictionary for the SSD detector
        self.confidence_thresholds = {
            0: self.get_parameter("threshold_vehicle").value,
            1: self.get_parameter("threshold_pedestrian").value,
            2: self.get_parameter("threshold_stop_sign").value,
            3: self.get_parameter("threshold_crosswalk_sign").value,
            4: self.get_parameter("threshold_parking_sign").value,
            5: self.get_parameter("threshold_thirty_sign").value,
            6: self.get_parameter("threshold_no_thirty_sign").value,
            7: self.get_parameter("threshold_go_left_sign").value,
            8: self.get_parameter("threshold_go_right_sign").value,
            9: self.get_parameter("threshold_intersection_right_of_way_sign").value,
            10: self.get_parameter("threshold_intersection_grant_sign").value,
            11: self.get_parameter("threshold_expressway_start").value,
            12: self.get_parameter("threshold_expressway_end").value,
            13: self.get_parameter("threshold_barred_area").value,
            14: self.get_parameter("threshold_pedestrian_island_right").value,
            15: self.get_parameter("threshold_no_passing_start").value,
            16: self.get_parameter("threshold_no_passing_end").value,
        }

        self.latest_image = None
        self.cv_bridge = cv_bridge.CvBridge()

        # Initialize the SSD detector
        self.ssd = SSD.SSD(self)

        # Pass thresholds to the SSD detector
        self.ssd.overall_threshold = self.get_parameter("detection_threshold").value
        self.ssd.confidence_thresholds = self.confidence_thresholds

        # Register parameter change callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.loop_timer = self.create_timer(self.dt, self.loop)
        self.get_logger().info("🚀 Object detection node initialized.")
        self.get_logger().info(
            f"Overall detection threshold: {self.detection_threshold}"
        )
        self.get_logger().info(
            f"Class-specific thresholds: {self.confidence_thresholds}"
        )

    def parameters_callback(self, params):
        """Handle parameter updates during runtime."""
        result = SetParametersResult()
        result.successful = True

        update_needed = False

        for param in params:
            if param.name == "detection_threshold":
                self.ssd.overall_threshold = param.value
                self.get_logger().info(
                    f"Updated overall detection threshold to {param.value}"
                )
                update_needed = True
            else:
                # Check if this is a class threshold parameter
                for class_id, param_name in self.class_mappings.items():
                    if param.name == param_name:
                        self.confidence_thresholds[class_id] = param.value
                        self.ssd.confidence_thresholds[class_id] = param.value
                        self.get_logger().info(
                            f"Updated threshold for class {class_id} to {param.value}"
                        )
                        update_needed = True
                        break

        if update_needed:
            self.get_logger().info(
                f"Updated confidence thresholds: {self.confidence_thresholds}"
            )

        return result

    @property
    def detection_threshold(self):
        """Return the overall detection threshold parameter."""
        return self.get_parameter("detection_threshold").value

    @property
    def dt(self):
        """Return the dt parameter."""
        return self.get_parameter("dt").value

    @property
    def config_path(self):
        """Return the config_path parameter."""
        return self.get_parameter("config_path").value

    def reset(self):
        """Reset the node to its initial state."""
        self.get_logger().warning("⚠️ Resetting the node ...")
        self.latest_image = None

    def loop(self):
        """Perception loop."""
        # Create empty message
        obj_msg = Float32MultiArray()
        sign_msg = Float32MultiArray()
        obj_msg.data = []
        sign_msg.data = []

        if self.latest_image is None:
            return

        if self.active:
            with Timer(name="prediction_timer", filter_strength=40):
                result, mapped_signs, mapped_objects, result_img = self.ssd.get_result(
                    self.latest_image, debug=self._debug
                )
            Timer().print()

            # If an object (pedestrian or car) is detected publish it to the object topic
            if result and mapped_objects:
                obj_msg = self.create_float32_multi_array([mapped_objects])

            # If a sign is detected publish it to the sign topic
            if result and mapped_signs:
                sign_msg = self.create_float32_multi_array([mapped_signs])

            if self._debug:
                self.debug_img_publisher.publish(
                    self.cv_bridge.cv2_to_imgmsg(result_img, encoding="rgb8")
                )
        else:
            self.get_logger().info(
                "🚫 Node is not active. No object detection performed."
            )

        # Publish the messages
        self.object_publisher.publish(obj_msg)
        self.sign_publisher.publish(sign_msg)

        if obj_msg.data:
            self.get_logger().info(f"Object detected: {obj_msg.data}")
        if sign_msg.data:
            self.get_logger().info(f"Sign detected: {sign_msg.data}")

    def create_float32_multi_array(self, objects: list) -> Float32MultiArray:
        """
        Create a Float32MultiArray message from a list of objects.

        Arguments:
            objects -- list of objects

        Returns:
            Float32MultiArray message
        """
        flat_objects = []
        # Flatten the list of objects
        objects = objects[0]
        print(objects)
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

    def camera_image_callback(self, image_msg: sensor_msgs.msg.Image):
        """
        Callback for camera image.

        Arguments:
            image_msg -- Image message
        """
        current_image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(
            (image_msg.height, image_msg.width)
        )
        current_image = cv2.cvtColor(current_image, cv2.COLOR_GRAY2BGR)
        self.latest_image = current_image


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
