import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate the launch description.

    Returns:
        LaunchDescription -- The launch description.
    """
    python_executable = os.getenv("PYTHON_EXECUTABLE", "/usr/bin/python3")

    # Declare the launch arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            FindPackageShare("object_detection").find("object_detection"),
            "config",
            "ros_params.yaml",
        ),
        description="Path to the ROS parameters file",
    )

    debug_arg = DeclareLaunchArgument(
        "debug", default_value="false", description="Enable debug mode"
    )

    # Include the parameters file
    params_file = LaunchConfiguration("params_file")

    # Define the node
    camera_preprocessing_node = Node(
        package="object_detection",
        executable="object_detection_node",
        name="object_detection_node",
        output="screen",
        parameters=[params_file, {"debug": LaunchConfiguration("debug")}],
        prefix=[python_executable],
    )

    return LaunchDescription([params_file_arg, debug_arg, camera_preprocessing_node])
