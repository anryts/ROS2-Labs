import time

import rclpy
import rclpy.qos as qos
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

from simple_ros2_interfaces.srv import DronePosition


class ArduPilotCompanionNode(Node):
    TAKEOFF_ALTITUDE = 10.0  # meters

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.drone_name_ = self.get_namespace()
        self.start_srv_ = self.create_service(
            DronePosition,
            f"{self.drone_name_}/{node_name}/start_flight",
            self.start_flight_callback_,
        )
        self.mavros_client_ = self.create_client(
            SetMode, f"{self.drone_name_}/mavros/set_mode"
        )
        self.takeoff_client_ = self.create_client(
            CommandTOL, f"{self.drone_name_}/mavros/cmd/takeoff"
        )

        self.arm_client_ = self.create_client(
            CommandBool, f"{self.drone_name_}/mavros/cmd/arming"
        )
        self.setpoint_pub_ = self.create_publisher(
            PoseStamped, f"{self.drone_name_}/mavros/setpoint_position/local", 10
        )
        self.setpoint_listener_ = self.create_subscription(
            PoseStamped,
            f"{self.drone_name_}/mavros/local_position/pose",
            self.position_callback_,
            qos.qos_profile_sensor_data,
        )
        self.state_sub_ = self.create_subscription(
            State, f"{self.drone_name_}/mavros/state", self.state_callback_, 10
        )
        self.drone_position_pub = self.create_publisher(
            Bool, f"{self.drone_name_}/companion_node/is_reached", 10
        )
        self.drone_is_ready_pub_ = self.create_publisher(
            Bool,
            f"{self.drone_name_}/companion_node/is_ready",
            qos.qos_profile_sensor_data,
        )
        # Just for test purpose, will see if it works
        self._broadcast_position_subscriber = self.create_subscription(
            PoseStamped,
            "/swarm_control/broadcast/position",
            self._broadcast_position_callback,
            qos.qos_profile_sensor_data,
        )
        self._broadcast_publisher = self.create_publisher(
            PoseStamped,
            "/swarm_control/broadcast/position",
            qos.qos_profile_sensor_data,
        )
        self._gps_listener = self.create_subscription(
            NavSatFix,
            f"{self.drone_name_}/mavros/global_position/global_int",
            self.gps_callback,
            10,
        )
        self.timer_ = self.create_timer(0.1, self.timer_callback_)
        self.get_logger().info(
            f"{node_name} was started and with namespace: {self.get_namespace()}"
        )

        self.target_pose_ = PoseStamped()

        self.target_pose_.header.frame_id = "map"
        self.target_pose_.pose.position.x = 0.0
        self.target_pose_.pose.position.y = 0.0
        self.target_pose_.pose.position.z = 0.0
        self.target_pose_.pose.orientation.w = 1.0  # Neutral orientation

        self.current_pose_ = PoseStamped()
        self.current_pose_.header.frame_id = "map"
        self.current_pose_.pose.position.x = 0.0
        self.current_pose_.pose.position.y = 0.0
        self.current_pose_.pose.position.z = 0.0
        self.current_pose_.pose.orientation.w = 1.0  # Neutral orientation
        self.is_armed_: bool = False
        self.current_mode_: str = ""
        self.is_takeoff_: bool = False
        self.gps_fix: bool = False
        self.set_guided_mode_()

    def gps_callback(self, msg: NavSatFix) -> None:
        """Callback function to handle incoming GPS messages."""
        if msg.status.status >= 2:
            self.gps_fix = True
            self.get_logger().info("GPS fix acquired")

    def _broadcast_position_callback(self, msg: PoseStamped) -> None:
        """Callback function for the broadcast position subscriber."""
        self.target_pose_.pose.position.x = msg.pose.position.x
        self.target_pose_.pose.position.y = msg.pose.position.y
        self.target_pose_.pose.position.z = msg.pose.position.z
        self.target_pose_.pose.orientation.w = msg.pose.orientation.w
        self.target_pose_.header.stamp = msg.header.stamp

    def drone_is_ready_callback_(self):
        if self.is_armed_ and self.current_mode_ == "GUIDED":
            self.drone_is_ready_pub_.publish(Bool(data=True))
        else:
            self.drone_is_ready_pub_.publish(Bool(data=False))
        return

    def timer_callback_(self):
        """Timer callback function to perform periodic tasks."""
        # self.publish_setpoint_()
        self.drone_is_ready_callback_()

    def position_callback_(self, msg: PoseStamped) -> None:
        """Callback function to handle incoming setpoint messages."""
        self.current_pose_ = msg
        self.is_drone_reached_callback_()

    def state_callback_(self, msg: State) -> None:
        # self.get_logger().info(f"Current mode: {msg.mode}, Armed: {msg.armed}")
        self.current_mode_ = msg.mode
        self.is_armed_ = msg.armed

    def set_guided_mode_(self):
        """Set drone in GUIDED mode and arm with additional checks."""

        if self.is_armed_:
            self.get_logger().info("Drone is already armed")
            return

        if not self.gps_fix:
            self.get_logger().error("No GPS fix, cannot set GUIDED mode")
            return

        # Wait for MAVROS to fully initialize
        self.get_logger().info("Waiting for MAVROS to initialize...")
        time.sleep(10)
        # Publish setpoints continuously at 10Hz
        for _ in range(100):
            self.publish_setpoint_()
            time.sleep(0.1)
        # Publish setpoints continuously before switching to GUIDED mode
        self.get_logger().info(
            "Publishing initial setpoints to prepare for GUIDED mode..."
        )
        time.sleep(5)

        # Wait for the set_mode service
        while not self.mavros_client_.wait_for_service(timeout_sec=1):
            self.get_logger().warn(f"Waiting for {self.drone_name_}/mavros/set_mode")

        # Request GUIDED mode
        msg = SetMode.Request()
        msg.base_mode = 1 << 7  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        msg.custom_mode = "GUIDED"

        future = self.mavros_client_.call_async(msg)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"{msg}")
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("GUIDED mode set successfully")
            else:
                self.get_logger().error("Failed to set GUIDED mode")
                return
        except Exception as e:
            self.get_logger().error(f"Failed to call set_mode service: {e}")
            return

        # Check if GUIDED mode is active
        self.get_logger().info("Verifying GUIDED mode...")

        while not self.arm_client_.wait_for_service(timeout_sec=1):
            self.get_logger().warn(f"Waiting for {self.drone_name_}/mavros/cmd/arming")

        # Request arming
        arm_msg = CommandBool.Request()
        arm_msg.value = True
        future = self.arm_client_.call_async(arm_msg)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Drone armed successfully")
                self.is_armed_ = True
            else:
                self.get_logger().error("Failed to arm drone")
                return
        except Exception as e:
            self.get_logger().error(f"Failed to call arming service: {e}")
            return
        self.is_armed_ = True

        # Takeoff command
        takeoff_msg = CommandTOL.Request()
        takeoff_msg.altitude = self.TAKEOFF_ALTITUDE
        future = self.takeoff_client_.call_async(takeoff_msg)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Drone took off successfully")
                self.is_takeoff_ = True
            else:
                self.get_logger().error("Failed to take off drone")
                return
        except Exception as e:
            self.get_logger().error(f"Failed to call takeoff service: {e}")
            return

    def publish_setpoint_(self):
        """Publish setpoints continuously at 10Hz."""
        # self.get_logger().info(
        #     f"Publishing setpoint: x={self.target_pose_.pose.position.x}, "
        #     f"y={self.target_pose_.pose.position.y}"
        # )
        self.target_pose_.header.stamp = self.get_clock().now().to_msg()
        self.is_drone_reached_callback_()
        self.setpoint_pub_.publish(self.target_pose_)
        self._broadcast_publisher.publish(self.target_pose_)

    def start_flight_callback_(
        self, request: DronePosition.Request, response: DronePosition.Response
    ):
        """Update current destination position
        Args:
            request (DronePosition.Request): _description_
            response (DronePosition.Response): _description_

        Returns:
            _type_: _description_
        """
        if self.current_mode_ != "GUIDED" or not self.is_armed_ or not self.is_takeoff_:
            self.get_logger().error(
                "Cannot start flight, Drone is not properly configured"
            )
            response.success = False
            return response

        self.target_pose_.pose.position.x = request.x
        self.target_pose_.pose.position.y = request.y
        self.target_pose_.pose.position.z = request.z
        self.target_pose_.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Received coords:\n x:{request.x} \n y:{request.y} \n z:{request.z}"
        )
        response.success = True
        return response

    def is_drone_reached_callback_(self) -> None:
        """Check if drone reached the target position
        Args:
            threshold (float, optional):. Defaults to 0.25.
        """
        delta_x = abs(
            self.target_pose_.pose.position.x - self.current_pose_.pose.position.x
        )
        delta_y = abs(
            self.target_pose_.pose.position.y - self.current_pose_.pose.position.y
        )
        delta_z = abs(
            self.target_pose_.pose.position.z - self.current_pose_.pose.position.z
        )
        if delta_x > 0.25 or delta_y > 0.25 or delta_z > 0.25:
            return self.drone_position_pub.publish(Bool(data=False))
        return self.drone_position_pub.publish(Bool(data=True))

    # TODO: do this
    def stop_flight_callback_(self):
        # Call mavros to stop drone
        pass


def main(args=None):
    rclpy.init(args=args)
    companion_node = ArduPilotCompanionNode("ardupilot_control")
    rclpy.spin(companion_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
