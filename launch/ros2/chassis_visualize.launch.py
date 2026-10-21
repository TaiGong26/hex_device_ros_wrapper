#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Hexfellow. All rights reserved.
# Author: Jecjune <jecjune@qq.com>
# Date  : 2026-08-21
################################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


WRAPPER_PKG = "hex_device_ros_wrapper"

# Extensible device table: add a new key to support another chassis model.
# Each entry maps a `device` name to the URDF package/file, the RViz config
# (inside this package), and the URDF joint names in motor order.
DEVICES = {
    "maver_x4": {
        "urdf_package": "hex_ros_urdf_maver_x4",
        "urdf_file": "urdf/model.urdf",
        "rviz_config": "display_maver_x4.rviz",
        "joint_names": [
            "joint_wheel1", "joint_yaw1", "joint_wheel2", "joint_yaw2",
            "joint_wheel3", "joint_yaw3", "joint_wheel4", "joint_yaw4",
        ],
    },
}


def _resolve_device(context, *args, **kwargs):
    """Resolve the selected device at launch time and build its nodes."""
    device = LaunchConfiguration('device').perform(context)
    spec = DEVICES.get(device)
    if spec is None:
        raise RuntimeError(
            f"Unknown device '{device}'. Supported devices: {list(DEVICES)}")

    urdf_path = PathJoinSubstitution(
        [FindPackageShare(spec["urdf_package"]), spec["urdf_file"]])
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(WRAPPER_PKG), "config", "ros2", spec["rviz_config"]])

    # Read the URDF directly (plain URDF, no xacro macros) so no external
    # xacro dependency is needed at launch time. package:// URDF references
    # still resolve because robot_state_publisher uses the installed packages.
    with open(urdf_path.perform(context), 'r') as f:
        urdf_content = f.read()
    robot_description = ParameterValue(urdf_content, value_type=str)

    viz_node = Node(
        package=WRAPPER_PKG,
        executable='chassis_viz',
        name='chassis_viz',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_names': spec["joint_names"],
            'frame_id': LaunchConfiguration('frame_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [viz_node, rsp_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='maver_x4',
            description=f'Chassis device to visualize. Supported: {list(DEVICES)}'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Flag to turn on rviz'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Flag to use simulation time (/clock)'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='base_link',
            description='Child TF frame (must match chassis_bringup frame_id)'),
        OpaqueFunction(function=_resolve_device),
    ])
