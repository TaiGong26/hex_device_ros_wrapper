#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import time
import os
import sys

script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)

from ros_interface import DataInterface
from hex_device import HexDeviceApi, public_api_up_pb2, LinearLift
from hex_device.motor_base import CommandType

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray, Bool

class HexLiftApi:
    
    def __init__(self):
        # 1. Create ROS interface
        self.ros_interface = DataInterface(name="xnode_lift", rate_hz=500)
        # Timeout check every 10Hz
        self._watchdog_check_every = max(1, int(0.1 * self.ros_interface.get_rate()))
        self._watchdog_counter = 0
        self._last_lift_timeout_log = 0.0

        # 2. Get parameters
        self.enable_ros_clock = self.ros_interface.get_parameter('enable_ros_clock')

        self.version_check = False
        self.first_time = True

        # 3. Create shared topics (ws_down, ws_up)
        self.ws_down_pub = self.ros_interface.create_publisher('ws_down', UInt8MultiArray, 10)
        self.ws_up_sub = self.ros_interface.create_subscription(
            'ws_up', UInt8MultiArray, self._ws_up_callback, 10)

        self.lift = None
        self.motor_states_pub = None
        self.joint_cmd_sub = None

        # 4. Init HexDeviceApi
        self.api = HexDeviceApi(control_hz=500, send_down_callback=self._pub_ws_down)

    # ========== Common topic callbacks ==========

    def _pub_ws_down(self, data):
        try:
            msg = UInt8MultiArray()
            msg.data = list(data)  # bytes -> list of uint8 for ROS2
            self.ros_interface.publish(self.ws_down_pub, msg)
        except Exception:
            pass

    def _ws_up_callback(self, msg):
        api_up = public_api_up_pb2.APIUp()
        try:
            api_up.ParseFromString(bytes(msg.data))
        except Exception:
            self.ros_interface.logw("Failed to parse ws_up message")
            return
        
        if not self.version_check:
            self.version_check = True
            if not self.api._is_support_version(api_up):
                self.ros_interface.loge("Version mismatch, API closed")
                self.api.close()
                self.ros_interface.shutdown()
                return
        self.api._process_api_up(api_up)

    def _setup_topics(self):
        self.joint_cmd_sub = self.ros_interface.create_subscription(
                '/xtopic_lift/joint_cmd', JointState, self._joint_cmd_callback, 10)
    
    def _joint_cmd_callback(self, msg):
        lift = self._get_lift()
        if lift is not None:
            lift.motor_command(CommandType.POSITION, msg.position)
    
    def _get_Lift(self):
        if self.lift is None:
            for device in self.api.device_list:
                if isinstance(device, LinearLift):
                    self.lift = device
                    return self.lift
        return self.lift
    
    def _get_clock_timestamp(self):
        _timestamp = None

        if self.enable_ros_clock == True:
            _timestamp = self.ros_interface.get_timestamp()
        else:
            _timestamp = self.ros_interface.get_timestamp_from_s_ns(self.lift._last_update_time.s, self.lift._last_update_time.ns)
        
        return _timestamp
    
    def _publish_motor_states(self, lift):
        if self.motor_states_pub is None:
            return
        motor_status = lift.get_motor_positions()
        if motor_status is None:
            return
        msg = JointState()
        msg.header.stamp = self._get_clock_timestamp()
        msg.name = [f"joint{i}" for i in range(len(motor_status['pos']))]
        msg.position = motor_status['pos'].tolist()
        msg.velocity = motor_status['vel'].tolist()
        msg.effort = motor_status['eff'].tolist()
        self.ros_interface.publish(self.motor_states_pub, msg)
        
    def _check_cmd_timeout(self, lift):
        """When no cmd_vel for timeout seconds, stop lift (watchdog)."""
        if lift.is_timeout():
            now = time.monotonic()
            if now - self._last_lift_timeout_log >= 5.0:
                self._last_lift_timeout_log = now
                self.ros_interface.logw("lift command timeout, stopping lift...")
            lift.stop()

# ========== Main Function ==========

def main():
    hex_Lift_api = HexLiftApi()

    try:
        while True:
            if hex_Lift_api.api.is_api_exit():
                print("Public API has exited.")
                break

            for device in hex_Lift_api.api.device_list:
                if isinstance(device, LinearLift):
                    if device.has_new_data():
                        if hex_Lift_api.first_time:
                            hex_Lift_api.first_time = False
                            hex_Lift_api._setup_topics()
                            device.start()
                            hex_Lift_api.ros_interface.logi("Lift initialized successfully")

                        hex_Lift_api._publish_motor_states(device)
                        
                    hex_Lift_api._watchdog_counter += 1
                    if hex_Lift_api._watchdog_counter >= hex_Lift_api._watchdog_check_every:
                        hex_Lift_api._watchdog_counter = 0
                        hex_Lift_api._check_cmd_timeout(device)

            hex_Lift_api.ros_interface.sleep()

    except KeyboardInterrupt:
        print("Received Ctrl-C.")
        hex_Lift_api.api.close()
    finally:
        pass

    print("Resources have been cleaned up.")
    exit(0)


if __name__ == '__main__':
    main()
