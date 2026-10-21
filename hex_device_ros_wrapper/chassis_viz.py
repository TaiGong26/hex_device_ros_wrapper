#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Hexfellow. All rights reserved.
# Author: Jecjune <jecjune@qq.com>
# Date  : 2026-08-21
################################################################

import rclpy
import rclpy.node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ChassisViz(rclpy.node.Node):
    """
    Chassis visualization helper node.

    The chassis driver (chassis_trans) publishes /odom and /motor_states, but
    no TF and no URDF-named joint states, so RViz cannot render the chassis.
    This node translates the driver output into what robot_state_publisher +
    RViz need:

      - /odom         (nav_msgs/Odometry) -> /tf  (odom -> frame_id)
      - /motor_states (sensor_msgs/JointState, joint0..N)
                                    -> /joint_states (renamed to URDF joint names)

    Params:
      frame_id   (string, default "base_link")  child TF frame / joint header frame
      use_sim_time (bool, default false)
      joint_names (string array, default empty -> pass through incoming names)
    """

    def __init__(self):
        super().__init__('chassis_viz')

        # Parameters (use_sim_time is auto-declared by rclpy's Node)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter(
            'joint_names',
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))

        self._frame_id = self.get_parameter('frame_id').value
        self._joint_names = list(self.get_parameter('joint_names').value or [])

        # Publishers / subscribers (default reliable QoS, matches the driver)
        self._tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self._motor_states_sub = self.create_subscription(
            JointState, '/motor_states', self._motor_states_cb, 10)

    def _odom_cb(self, msg):
        """Publish odom -> frame_id transform from the odometry message."""
        ts = TransformStamped()
        ts.header.stamp = msg.header.stamp
        ts.header.frame_id = msg.header.frame_id or 'odom'
        ts.child_frame_id = msg.child_frame_id or self._frame_id

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        ts.transform.translation.x = p.x
        ts.transform.translation.y = p.y
        ts.transform.translation.z = p.z
        ts.transform.rotation.x = o.x
        ts.transform.rotation.y = o.y
        ts.transform.rotation.z = o.z
        ts.transform.rotation.w = o.w
        if ts.transform.rotation.x == 0.0 and ts.transform.rotation.y == 0.0 \
                and ts.transform.rotation.z == 0.0 and ts.transform.rotation.w == 0.0:
            ts.transform.rotation.w = 1.0

        self._tf_pub.publish(TFMessage(transforms=[ts]))

    def _motor_states_cb(self, msg):
        """Rename /motor_states joints to the URDF names and republish to /joint_states."""
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id

        names = [n for n in self._joint_names if n]
        if not names:
            names = list(msg.name)

        n = min(len(names), len(msg.position), len(msg.velocity), len(msg.effort))
        out.name = list(names[:n])
        out.position = list(msg.position[:n])
        out.velocity = list(msg.velocity[:n])
        out.effort = list(msg.effort[:n])
        self._js_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
