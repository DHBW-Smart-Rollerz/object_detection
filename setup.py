# Copyright (c) 2024 Smart Rollerz e.V. All rights reserved.

import os

from setup_utils import include_directory
from setuptools import find_packages, setup

package_name = "object_detection"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *include_directory(
            install_path=os.path.join("share", package_name, "config"),
            source_path="config",
        ),
        *include_directory(
            install_path=os.path.join("share", package_name, "launch"),
            source_path="launch",
        ),
        *include_directory(
            install_path=os.path.join("share", package_name, "models"),
            source_path="models",
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Smart Rollerz",
    maintainer_email="info@dhbw-smartrollerz.org",
    description="Object Detection of the smarty project",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"ros2_example_node = {package_name}.ros2_example_node:main",
        ],
    },
)
