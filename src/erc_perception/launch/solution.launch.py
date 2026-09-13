"""Provide a perception-only solution entry point with the same arguments."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Include perception without starting a simulator or moving the robot."""
    path = (Path(get_package_share_directory('erc_perception'))
            / 'launch' / 'perception.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path))),
    ])
