#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy import qos

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class MissionNode(Node):

    def __init__(self):
        super().__init__("mission_node")

        # Subscription
        self.current_state = State()
        self.create_subscription(State, "/mavros/state", self.state_callback,
                                 qos.qos_profile_sensor_data)

        # Service clients
        arm_endpoint = "/mavros/cmd/arming"
        self.arming_service = self.create_client(CommandBool, arm_endpoint)
        while not self.arming_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting MAVROS service {}'.format(arm_endpoint))

        setmode_endpoint = "/mavros/set_mode"
        self.setmode_service = self.create_client(SetMode, setmode_endpoint)
        while not self.setmode_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting MAVROS service {}'.format(setmode_endpoint))

        self.mode_msg = SetMode.Request()
        self.mode_msg.custom_mode = 'AUTO.MISSION'
        self.mission_enabled = False
        self.setmode_async_call = None

        self.arm_msg = CommandBool.Request()
        self.arm_msg.value = True
        self.arm_done = False
        self.arming_async_call = None

        self.setmode_async_call = None
        self.arming_async_call = None

        self.last_request = self.get_clock().now()

        # Register main control callback
        self.create_timer(5, self.start_mission_callback)

    def state_callback(self, msg):
        self.current_state = msg

    def start_mission_callback(self):

        # Set "mission" mode
        if not self.mission_enabled:
            if self.setmode_async_call is None:
                if (self.get_clock().now().seconds_nanoseconds()[0] - self.last_request.seconds_nanoseconds()[0]) > 10:
                    self.setmode_async_call = self.setmode_service.call_async(self.mode_msg)
                    self.last_request = self.get_clock().now()
            else:
                if self.setmode_async_call.done():
                    self.get_logger().info("Mission mode enabled")
                    self.mission_enabled = True
        else:
            if not self.arm_done:
                if self.arming_async_call is None:
                    if (self.get_clock().now().seconds_nanoseconds()[0] - self.last_request.seconds_nanoseconds()[0]) > 2:
                        self.arming_async_call = self.arming_service.call_async(self.arm_msg)
                        self.last_request = self.get_clock().now()
                else:
                    if self.arming_async_call.done():
                        self.get_logger().info("Vehicle armed")
                        self.arm_done = True


def main(args=None):
    rclpy.init(args=args)

    mission_node = MissionNode()
    mission_node.get_logger().info('Node started')

    try:
        rclpy.spin(mission_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        mission_node.destroy_node()


if __name__ == "__main__":
    main()
