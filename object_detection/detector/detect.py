# Copyright (c) 2024 Smart Rollerz e.V. All rights reserved.

import os

import cv2
import numpy as np
from PIL import ImageDraw
from pycoral.adapters import detect
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter

from object_detection.detector import *


def draw_objects(draw, objs, labels):
    """Draws the bounding box and label for each object."""
    # iterate over all detected objects
    for obj in objs:
        # get bounding box
        bbox = obj.bbox
        # draw bounding box
        draw.rectangle([(bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax)], outline="red")
        # draw label
        draw.text(
            (bbox.xmin + 10, bbox.ymin + 10),
            "%s\n%.2f" % (labels.get(obj.id, obj.id), obj.score),
            fill="red",
        )


def preload_model(model_path: str):
    """
    Preload the model to avoid delays during runtime.

    Arguments:
        model_path -- path to the model file

    Returns:
        interpreter -- the loaded model
    """
    # check if model exists
    if not os.path.exists(model_path):
        # if not, print error message and return None
        print(f"Error: The file '{model_path}' does not exist.")
        return None
    # if model exists, load it
    interpreter = make_interpreter(
        model_path_or_content=model_path
    )  # , device=":0")#, device=":1"
    interpreter.allocate_tensors()
    return interpreter


def detect_ssd(
    image,
    model,
    scale,
    threshold=0.6,
    confidence_thresholds=None,
    debug=False,
    labels_file=os.path.join(PKG_PATH, "models/smarty_labels.txt"),
):
    """
    Detect objects in the image using class-specific confidence thresholds.
    
    Arguments:
        image -- input image
        model -- loaded model
        scale -- image scale
        threshold -- default threshold for all classes
        confidence_thresholds -- dictionary of class-specific thresholds {class_id: threshold}
        debug -- enable debug output
        labels_file -- path to labels file
        
    Returns:
        detected objects and result image
    """
    # get labels
    labels = read_label_file(labels_file)

    # run model
    model.invoke()
    
    # Calculate minimum threshold for initial detection
    min_threshold = 0.1  # Base minimum threshold
    
    # If we have class-specific thresholds, find the minimum value
    if confidence_thresholds and len(confidence_thresholds) > 0:
        min_confidence = min(confidence_thresholds.values())
        min_threshold = max(0.1, min_confidence * 0.5)  # Half of the minimum threshold, but not below 0.1
    else:
        # If no class thresholds, use half of the overall threshold
        min_threshold = max(0.1, threshold * 0.5)
    
    # Get all potential detections with the minimum threshold
    all_objs = detect.get_objects(
        interpreter=model, score_threshold=min_threshold, image_scale=scale
    )
    
    # Filter objects based on their class-specific thresholds
    filtered_objs = []
    for obj in all_objs:
        # Determine the threshold to use for this object
        class_threshold = threshold  # Default to overall threshold
        if confidence_thresholds and obj.id in confidence_thresholds:
            class_threshold = confidence_thresholds[obj.id]  # Use class-specific if available
        
        # Keep only objects that meet their threshold
        if obj.score >= class_threshold:
            filtered_objs.append(obj)
    
    # Draw on image
    image = image.convert("RGB")
    draw_objects(ImageDraw.Draw(image), filtered_objs, labels)
    image_np = np.array(image)

    if debug:
        for obj in filtered_objs:
            used_threshold = confidence_thresholds.get(obj.id, threshold) if confidence_thresholds else threshold
            print(f"Detected {labels.get(obj.id, obj.id)} (ID: {obj.id}) with score {obj.score:.2f} (threshold: {used_threshold:.2f})")
    
    # Return filtered objects and result image
    return filtered_objs, image_np