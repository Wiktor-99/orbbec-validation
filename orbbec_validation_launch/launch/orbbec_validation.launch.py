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
        ]),
        launch_arguments={
            'enable_point_cloud': 'true',
            'enable_colored_point_cloud': 'true',
            'enable_depth': 'true',
            'enable_color': 'true',
        }.items()
    )

    # Record only the essential topics
    rosbag_recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/camera/color/image_raw',
            '/camera/depth/image_raw',
            '/camera/depth/points',
            '/camera/color/camera_info',
            '/camera/depth/camera_info',
            '/tf',
            '/tf_static',
            '--storage', 'mcap',
            '--storage', 'mcap',
            '--compression-mode', 'file',
            '--compression-format', 'zstd',
            '--output', recording_path
        ],
        output='screen',
        respawn=True
    )

    return LaunchDescription([
        recording_path_arg,
        orbbec_camera_launch,
        rosbag_recorder,
    ])
