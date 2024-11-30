# Object Detection

This project contains the Object Detection for the Smarty Project.

## Installation

> Caution: On Ubuntu 24.04 doesn't exist a fitting pycoral build.

First, the `pycoral` library must be installed:

```bash
pyenv activate default # if not already done
./install_pycoral.sh
```

Next, the model must be downloaded from the NAS ([link](http://141.31.112.6:5000/d/f/118s3IhuL7bwffYAdsNlvFhaYe0wULr8)) and placed into the `models` directory. Additionally, the `model_path` must be updated in the `config/ros_params.yaml` file.

## Usage

Place the correct model file in the models directory and specify the correct parameter in the configuration file and make sure, that the coral is plugged in.
Then run:

```bash
ros2 launch object_detection object_detection.launch.py # Simple
ros2 launch object_detection object_detection.launch.py debug:='{True|False}' # Debug
```
