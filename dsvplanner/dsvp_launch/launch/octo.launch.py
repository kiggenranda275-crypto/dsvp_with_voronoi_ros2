import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    param_dir_octomap = os.path.join(
        get_package_share_directory("dsvp_launch"), "config", "octo.yaml"
    )
    print("octomap_dir:: ", param_dir_octomap)
    # print(rviz_path)

    return LaunchDescription(
        [
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server",
                output="screen",
                parameters=[
                    {"frame_id": "map"},
                    {"resolution": 0.3},
                    {"sensor_model/max_range": 10.0},
                    {"occupancy_threshold": 0.7},
                    {"sensor_model/hit": 0.9},
                    {"sensor_model/miss": 0.2},
                    {"sensor_model/min": 0.2},
                    {"sensor_model/max": 0.9},
                    {"occupancy_min_z": 0.4},
                    {"occupancy_max_z": 1.5},
                    {"point_cloud_min_z": -1.0},
                    {"point_cloud_max_z": 1.5},
                    {"filter_ground": True},
                    {"filter_speckles": False},
                    {"automatic_pruning": False},
                ],
                remappings=[
                    ("/cloud_in", "/sensor_scan"),
                    # /lio_sam/mapping/cloud_registered /velodyne_points
                    ("/octomap_binary", "/octomap_binary_octo"),
                    ("/octomap_full", "/octomap_full_octo"),
                ],
            ),
        ]
    )
