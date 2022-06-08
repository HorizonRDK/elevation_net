from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 启动图片发布pkg
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            output='screen',
            parameters=[
                {"out_format": "nv12"},
                {"image_width": 960},
                {"image_height": 544},
                {"io_method": "shared_mem"},
                {"video_device": "F37"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动elevation_net pkg
        Node(
            package='elevation_net',
            executable='elevation_net',
            output='screen',
            parameters=[
                {"shared_mem": 1},
                {"config_file_path": "./config"}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
