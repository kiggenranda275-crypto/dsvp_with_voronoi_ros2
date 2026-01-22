import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    publish_tf = LaunchConfiguration("publish_tf", default="true")
    declare_publish_tf = DeclareLaunchArgument(
        "publish_tf",
        default_value="true",
        description="Publish tf from odom to map",
    )

    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0.0 0.0 0.63 0.0 0.0 0.0 map odom".split(),
        output="screen",
        condition=IfCondition(publish_tf),
    )

    slam_interface_node = Node(
        package="slam_interface",
        executable="slam_interface",
        name="slam_interface_node",
        output="screen",
        parameters=[
            {"registered_scan_topic": "/lio_sam/mapping/cloud_registered_raw"},
            {"odom_topic": "/lio_sam/mapping/odometry"},
            {"odom_tf_name": "odom"},
            {"map_tf_name": "map"},
            {"robot_tf_frame": "body"},
            {"base_tf_name": "base_link"},
            {"sensor_tf_frame": "velodyne_link"},
            {"publish_tf": False},
            {"publish_lidar2base": False},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_publish_tf)
    ld.add_action(tf_node)  # 取消注释以启用条件控制
    ld.add_action(slam_interface_node)

    return ld
