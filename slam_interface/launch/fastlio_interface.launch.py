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
        arguments="0.0 0.0 0.63 0.0 0.0 0.0 map camera_init".split(),
        output="screen",
        condition=IfCondition(publish_tf),
    )

    tf_node_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0.0 0.0 -0.2 0.0 0.0 0.0 base_link velodyne_link".split(),
        output="screen",
        condition=IfCondition(publish_tf),
    )

    slam_interface_node = Node(
        package="slam_interface",
        executable="slam_interface",
        name="slam_interface_node",
        output="screen",
        parameters=[
            {"registered_scan_topic": "/cloud_registered"},
            {"odom_topic": "/Odometry"},
            {"odom_tf_name": "camera_init"},
            {"map_tf_name": "map"},
            {"robot_tf_name": "body"},
            {"base_tf_name": "base_link"},
            {"sensor_tf_frame": "velodyne_link"},
            {"publish_tf": True},
            {"publish_lidar2base": True},  # SLAM不发布odom到base情况下使用   改为ture -by zjh
        ],
    )

    ld = LaunchDescription()
    ld.add_action(tf_node_1) #解除注释  - by zjh
    ld.add_action(declare_publish_tf)
    ld.add_action(tf_node)
    ld.add_action(slam_interface_node)

    return ld
