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
    debug=False,
    labels_file=os.path.join(PKG_PATH, "models/smarty_labels.txt"),
):
    """Detect objects in the image."""
    # get labels
    labels = read_label_file(labels_file)

    # run model
    model.invoke()
    objs = detect.get_objects(
        interpreter=model, score_threshold=threshold, image_scale=scale
    )

    image = image.convert("RGB")
    draw_objects(ImageDraw.Draw(image), objs, labels)
    # Convert the PIL image to a numpy array
    image_np = np.array(image)

    if debug:
        for obj in objs:
            # if obj.score != 0.5:
            if False:
                print(f"obj: {obj}")
                print(labels.get(obj.id, obj.id))
                print("  id:    ", obj.id)
                print("  score: ", obj.score)
                print("  bbox:  ", obj.bbox)

        if False:
            # Display the image using OpenCV
            cv2.imshow("result", image_np)
            cv2.waitKey(1)

        if False:
            # draw bounding boxes and labels on the image
            output_path = os.path.join(PKG_PATH, f"output/result_img.png")
            image = image.convert("RGB")
            draw_objects(ImageDraw.Draw(image), objs, labels)
            # save image
            image.save(output_path)
            image.show()
    # return detected objects
    return objs, image_np
