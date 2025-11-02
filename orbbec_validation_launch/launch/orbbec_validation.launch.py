#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    recording_path_arg = DeclareLaunchArgument(
        'recording_path',
        default_value='/tmp/recordingns',
        description='Path to store recorded rosbags'
    )

    recording_path = LaunchConfiguration('recording_path')

    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'),
                'launch',
                'gemini_330_series.launch.py'
            ])
        ])
    )

    recording_dir_script = ExecuteProcess(
        cmd=['bash', '-c', 'mkdir -p /tmp/recordingns'],
        output='screen'
    )

    rosbag_recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--all',
            '--storage', 'mcap',
            '--output', recording_path
        ],
        output='screen',
        respawn=True
    )

    return LaunchDescription([
        recording_path_arg,
        orbbec_camera_launch,
        recording_dir_script,
        rosbag_recorder,
    ])

