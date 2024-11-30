# Copyright (c) 2024 Smart Rollerz e.V. All rights reserved.

import os

import cv2
import numpy as np
import rclpy
import yaml
from camera_preprocessing.camera_preprocessing.transformation.birds_eyed_view import (
    BirdseyedviewTransformation,
)
from camera_preprocessing.camera_preprocessing.transformation.coordinate_transform import (
    CoordinateTransform,
)
from detect import detect_ssd, preload_model
from PIL import Image
from pycoral.adapters import common

from object_detection.object_detection.detector import *


class SSD:
    """SSD object detection class."""

    def __init__(self, parent: object):
        """Initialize the SSD object detection class."""
        self.parent = parent
        self.birdseyedview = BirdseyedviewTransformation(debug=self.parent.debug)
        self.coordinate_transform = CoordinateTransform()
        self.detection_list = np.zeros(11)
        self.load_config(self.parent.config_path)

        # Paths
        self.labels_path = os.path.join(PKG_PATH, self.config.get("labels_path"))
        self.model_path = os.path.join(PKG_PATH, self.config.get("model_path"))

        # Load model
        self.model = self.load_model()
        print(self.detection_list)

    def load_config(self, config_path: str):
        """
        Loads the configuration file.

        Arguments:
            config_path -- path to the configuration file

        Returns:
            None
        """
        if not os.path.exists(config_path):
            rclpy.get_logger().error(f"Error: The file '{config_path}' does not exist.")
            return None

        with open(config_path, "r") as file:
            self.config = yaml.safe_load(file)

        rclpy.get_logger().info(f"Configuration file loaded: {self.config}")

    def load_model(self):
        """Loads model."""
        return preload_model(self.model_path)

    def get_corresponding_class_label(self, class_id):
        """
        Returns the corresponding class label for the given class id.

        Arguments:
            class_id -- class id

        Returns:
            corresponding class label
        """
        # vehicle
        if class_id == 0:
            return 2
        # pedestrian
        if class_id == 1:
            return 10
        # stop-sign
        elif class_id == 2:
            return 1
        # crosswalk-sign
        elif class_id == 3:
            return 9
        # parking-sign
        elif class_id == 4:
            return 14
        # thirty-sign
        elif class_id == 5:
            return 7
        # no-thirty-sign
        elif class_id == 6:
            return 8
        # go-left-sign
        elif class_id == 7:
            return 15
        # go-right-sign
        elif class_id == 8:
            return 16
        # intersection-right-of-way-sign
        elif class_id == 9:
            return 17
        # intersection-grant-sign
        elif class_id == 10:
            return 18
        else:
            return 0

    def get_result(
        self,
        image: np.ndarray,
        debug=True,
    ):
        """Calculates the average of an image and return the result with a bias."""
        # Convert NumPy array to PIL Image
        img = Image.fromarray(image)
        initial_image_size = img.size
        initial_width, initial_height = initial_image_size
        detection_width, detection_height = tuple(
            self.config.get("detection_size", [640, 640])
        )

        # Calculate scaling factors
        height_scale = initial_height / detection_height
        width_scale = initial_width / detection_width

        # Resize the image to the expected input size of the model
        resized_img = img.resize((detection_width, detection_height), Image.BILINEAR)

        # prepare image for object detection
        _, scale = common.set_resized_input(
            self.model,
            resized_img.size,
            lambda size: resized_img.resize(size, Image.BILINEAR),
        )

        # detect object with SSD MobileNet V2
        objects, result_img = detect_ssd(
            image=resized_img,
            model=self.model,
            scale=scale,
            debug=debug,
            labels_file=self.labels_path,
        )

        ######################
        # FOR MANUAL DEBUGGING
        ######################

        if DEBUG:
            # Convert image to birdseye view
            bird_img = self.birdseyedview.undistorted_to_birdseyedview(img=image)
            # Display the resized image using OpenCV
            cv2.imshow("Resized Image with Bounding Boxes", result_img)
            cv2.waitKey(1)

            print(f"detected objects pre transform: {objects}")

        ######################

        mapped_sign = []
        mapped_objects = []
        self.detection_list -= 1
        self.detection_list[self.detection_list < -5] = -5
        for obj in objects:
            x1, y1, x2, y2 = obj.bbox.xmin, obj.bbox.ymax, obj.bbox.xmax, obj.bbox.ymax
            x1 = int(x1 * width_scale)
            y1 = int(y1 * height_scale)
            x2 = int(x2 * width_scale)
            y2 = int(y2 * height_scale)
            # bottom left
            if obj.id == 0 or obj.id == 1:
                bottom_left_corner = self.coordinate_transform.camera_to_world(
                    points=[[x1, y1]], Z_w=0
                )[0]
                #    bottom right (y1 is correct)

                bottom_right_corner = self.coordinate_transform.camera_to_world(
                    points=[[x2, y2]], Z_w=0
                )[0]
            else:
                bottom_left_corner = self.coordinate_transform.camera_to_world(
                    points=[[x1, y1]], Z_w=130
                )[0]
                # bottom right (y1 is correct)

                bottom_right_corner = self.coordinate_transform.camera_to_world(
                    points=[[x2, y2]], Z_w=130
                )[0]
            self.detection_list[obj.id] += 2

            self.detection_list[self.detection_list > 5] = 5
            new_id = self.get_corresponding_class_label(obj.id)
            mapped_dict = {
                "id": new_id,  # float(obj.id),
                "score": obj.score,
                "bottom-left-distance": {
                    "x": bottom_left_corner[0],
                    "y": bottom_left_corner[1],
                },
                "bottom-right-distance": {
                    "x": bottom_right_corner[0],
                    "y": bottom_right_corner[1],
                },
            }
            if self.detection_list[obj.id] >= 0:
                if new_id == 2 or new_id == 10:
                    mapped_objects.append(mapped_dict)
                else:
                    mapped_sign.append(mapped_dict)
        print(self.detection_list)

        print(f"detected objects post transform: {mapped_sign}")

        ######################
        # FOR MANUAL DEBUGGING
        ######################

        if objects and DEBUG:
            output_path_initial = os.path.join(
                PKG_PATH, f"output/initial_image_with_boxes.jpg"
            )
            output_path_result = os.path.join(
                PKG_PATH, f"output/result_image_with_boxes.jpg"
            )

            # Draw bounding boxes on the initial image
            initial_img_with_boxes = image.copy()
            for box in mapped_bounding_boxes:
                x1, y1, x2, y2 = (
                    box["bbox"]["xmin"],
                    box["bbox"]["ymin"],
                    box["bbox"]["xmax"],
                    box["bbox"]["ymax"],
                )
                cv2.rectangle(
                    initial_img_with_boxes, (x1, y1), (x2, y2), (0, 255, 0), 2
                )
            # Save the initial image with bounding boxes
            cv2.imwrite(output_path_initial, initial_img_with_boxes)

            # Save the resized image with bounding boxes
            cv2.imwrite(output_path_result, result_img)

        ######################

        # return detected objects + bounding boxes in world coordinates + image with bounding boxes
        return objects, mapped_sign, mapped_objects, result_img
