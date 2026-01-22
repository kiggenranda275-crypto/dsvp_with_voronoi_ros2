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
    param_dir_planning = os.path.join(
        get_package_share_directory("dsvp_launch"), "config", "exploration_indoor.yaml",
    )
    param_dir_octomap = os.path.join(
        get_package_share_directory("dsvp_launch"), "config", "octomap_indoor.yaml"
    )
    param_dir_boundary = os.path.join(
        get_package_share_directory("dsvp_launch"), "data", "boundary.ply"
    )
    rviz_path = os.path.join(
        get_package_share_directory("dsvp_launch"), "config", "default.rviz"
    )
    use_boundary = LaunchConfiguration("use_boundary", default="false")
    enable_bag_record = LaunchConfiguration("enable_bag_record", default="false")
    bag_name = LaunchConfiguration("bag_name", default="indoor")
    print(param_dir_planning, param_dir_octomap)

    return LaunchDescription(
        [
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server",
                output="screen",
                parameters=[
                    {"frame_id": "map"},
                    {"resolution": 0.1},
                    {"sensor_model/max_range": 10.0},
                    {"occupancy_threshold": 0.8},
                    {"sensor_model/hit": 0.97},
                    {"sensor_model/miss": 0.2},
                    {"sensor_model/min": 0.2},
                    {"sensor_model/max": 0.9},
                    {"occupancy_min_z": 0.3},
                    {"occupancy_max_z": 1.0},
                    {"point_cloud_min_z": -0.7},
                    {"point_cloud_max_z": 2.0},
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
            Node(
                package="dsvp_launch",
                executable="exploration",
                name="exploration",
                prefix=["stdbuf -o L"],
                output="screen",
                parameters=[param_dir_planning],
            ),
            Node(
                package="dsvplanner",
                executable="dsvplanner_exe",
                name="dsvplanner",
                prefix=["stdbuf -o L"],
                output="screen",
                parameters=[param_dir_planning, param_dir_octomap],
            ),
            Node(
                package="dsvp_launch",
                executable="navigationBoundary",
                name="navigationBoundary",
                prefix=["stdbuf -o L"],
                output="screen",
                parameters=[
                    {"boundary_file_dir": param_dir_boundary},
                    {"sendBoundary": "true"},
                    {"sendBoundaryInterval": "2"},
                ],
                condition=IfCondition(use_boundary),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="dsvp_rviz",
                prefix=["stdbuf -o L"],
                output="screen",
                arguments=["-d", rviz_path],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("graph_planner"),
                        "launch/graph_planner.launch",
                    )
                )
            ),
        ]
    )
