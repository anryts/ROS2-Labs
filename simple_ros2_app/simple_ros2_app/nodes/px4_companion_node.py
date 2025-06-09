import time

import rclpy
import rclpy.qos as qos
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from pynput import keyboard
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from simple_ros2_interfaces.msg import DroneDetection
from simple_ros2_interfaces.srv import DronePosition


class PX4CompanionNode(Node):
    """Operates a drone in offboard mode with keyboard and YOLO-based following."""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.drone_name_ = self.get_namespace()

        # Declare and get parameters
        self.declare_parameter("use_keyboard", False)
        self.declare_parameter("detect_mode", False)
        self.use_keyboard = (
            self.get_parameter("use_keyboard").get_parameter_value().bool_value
        )
        self.detect_mode = (
            self.get_parameter("detect_mode").get_parameter_value().bool_value
        )

        # Publishers and subscribers
        self._vel_publisher = self.create_publisher(
            TwistStamped,
            f"{self.drone_name_}/mavros/setpoint_velocity/cmd_vel",
            qos.qos_profile_sensor_data,
        )
        self.start_srv_ = self.create_service(
            DronePosition,
            f"{self.drone_name_}/{node_name}/start_flight",
            self.start_flight_callback_,
        )
        self.mavros_client_ = self.create_client(
            SetMode, f"{self.drone_name_}/mavros/set_mode"
        )
        self.arm_client_ = self.create_client(
            CommandBool, f"{self.drone_name_}/mavros/cmd/arming"
        )
        self.setpoint_pub_ = self.create_publisher(
            PoseStamped,
            f"{self.drone_name_}/mavros/setpoint_position/local",
            qos.qos_profile_sensor_data,
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
        # self._broadcast_position_subscriber = self.create_subscription(
        #     PoseStamped,
        #     "/swarm_control/broadcast/position",
        #     self._broadcast_position_callback,
        #     qos.qos_profile_sensor_data,
        # )
        # self._broadcast_publisher = self.create_publisher(
        #     PoseStamped,
        #     "/swarm_control/broadcast/position",
        #     qos.qos_profile_sensor_data,
        # )
        self.drone_detection_sub = self.create_subscription(
            DroneDetection,
            f"{self.drone_name_}/drone_detection",
            self.drone_detection_callback,
            qos.qos_profile_sensor_data,
        )
        self.image_sub = self.create_subscription(
            Image,
            f"{self.drone_name_}/camera/image_raw",
            self.image_callback,
            qos.qos_profile_sensor_data,
        )
        self.timer_ = self.create_timer(0.1, self.timer_callback_)
        self.get_logger().info(
            f"{node_name} was started with namespace: {self.get_namespace()}, "
            f"use_keyboard: {self.use_keyboard}, detect_mode: {self.detect_mode}"
        )

        # Initialize variables
        self.target_pose_ = PoseStamped()
        self.current_vel_ = TwistStamped()
        self.current_vel_.header.frame_id = "base_link"
        self.linear_speed = 0.125  # Speed in m/s, or not (:
        self.following_mode = False  # Set based on detect_mode
        self.detected_drone = None
        self.image_width = 1024
        self.image_height = 768

        # Keyboard handling (only if use_keyboard is True)
        self.movement_keys = {"w", "s", "a", "d", "z", "x", "q", "e", "t"}
        self.pressed_keys = set()
        self.keyboard_listener = None
        if self.use_keyboard:
            self.keyboard_listener = keyboard.Listener(
                on_press=self.on_press, on_release=self.on_release
            )
            self.keyboard_listener.start()
            self.get_logger().info("Keyboard control enabled")
        else:
            self.get_logger().info("Keyboard control disabled")

        self.target_pose_.header.frame_id = "map"
        self.target_pose_.pose.position.x = 0.0
        self.target_pose_.pose.position.y = 0.0
        self.target_pose_.pose.position.z = 0.0
        self.target_pose_.pose.orientation.w = 1.0

        self.current_pose_ = PoseStamped()
        self.current_pose_.header.frame_id = "map"
        self.current_pose_.pose.position.x = 0.0
        self.current_pose_.pose.position.y = 0.0
        self.current_pose_.pose.position.z = 0.0
        self.current_pose_.pose.orientation.w = 1.0
        self._is_auto_follow = False
        self._is_drone_reached: bool = False
        self.is_armed_ = False
        self.current_mode_ = ""
        self.set_offboard_mode()

    def on_press(self, key):
        """Handle key presses for manual control."""
        if not self.use_keyboard or not self.is_armed_:
            return
        try:
            ch = key.char.lower()
            if ch in self.movement_keys:
                self.pressed_keys.add(ch)
                if not self.following_mode:
                    self._update_coordinates_via_keyboard()
        except AttributeError:
            pass

    def on_release(self, key):
        if not self.use_keyboard or not self.is_armed_:
            return
        try:
            ch = key.char.lower()
            if ch in self.movement_keys:
                self.pressed_keys.discard(ch)
                if not self.following_mode:
                    self._update_coordinates_via_keyboard()
        except AttributeError:
            pass

    def _update_coordinates_via_keyboard(self):
        """Update current coordinates based on pressed keys."""
        if "w" in self.pressed_keys:
            self.target_pose_.pose.position.x += self.linear_speed
        if "s" in self.pressed_keys:
            self.target_pose_.pose.position.x -= self.linear_speed
        if "a" in self.pressed_keys:
            self.target_pose_.pose.position.y -= self.linear_speed
        if "d" in self.pressed_keys:
            self.target_pose_.pose.position.y += self.linear_speed
        if "z" in self.pressed_keys:
            self.target_pose_.pose.position.z += self.linear_speed
        if "x" in self.pressed_keys:
            self.target_pose_.pose.position.z -= self.linear_speed
        if "q" in self.pressed_keys:
            self.target_pose_.pose.orientation.z += 0.1
        if "e" in self.pressed_keys:
            self.target_pose_.pose.orientation.z -= 0.1
        if "t" in self.pressed_keys:
            self._is_auto_follow = not self._is_auto_follow
            if self._is_auto_follow:
                self.get_logger().info("Auto-follow mode enabled")
            else:
                self.get_logger().info("Auto-follow mode disabled")

    def drone_detection_callback(self, msg: DroneDetection):
        """Store the latest drone detection."""
        self.detected_drone = msg
        # self.get_logger().info(
        #     f"Received drone detection: x={msg.x_min}-{msg.x_max}, y={msg.y_min}-{msg.y_max}, prob={msg.probability}"
        # )

    def image_callback(self, msg: Image):
        """Update image dimensions."""
        self.image_width = msg.width
        self.image_height = msg.height

    def set_velocity_from_detection(self):
        """Set velocity to follow the detected drone."""
        self.get_logger().info(f"{self._is_auto_follow} {self.detect_mode}")
        if self.detected_drone is not None and self._is_auto_follow:
            self.get_logger().info("Following drone")
            center_x = (self.detected_drone.x_min + self.detected_drone.x_max) / 2
            center_y = (self.detected_drone.y_min + self.detected_drone.y_max) / 2
            image_center_x = self.image_width / 2
            error_x = center_x - image_center_x
            error_y = center_y - (self.image_height / 2)
            self.get_logger().info(
                f"Detected drone center: ({center_x}, {center_y}), "
                f"Image center: ({image_center_x}, {self.image_height / 2}), "
                f"Errors: ({error_x}, {error_y})"
            )
            # Calculate velocity based on error
            # TODO: make it more smooth
            Kp = 0.001
            self.target_pose_.pose.position.x += Kp * error_x
            self.target_pose_.pose.position.y = -Kp * error_y

    def timer_callback_(self):
        """Handle periodic tasks."""
        if self.use_keyboard:
            self.get_logger().info(
                f"{self.current_vel_.twist.linear.x}, {self.current_vel_.twist.linear.y}, {self.current_vel_.twist.linear.z}"
            )
            self._update_coordinates_via_keyboard()
        if self.detect_mode:
            self.set_velocity_from_detection()
        self.publish_setpoint_()
        self.drone_is_ready_callback_()

    def _broadcast_position_callback(self, msg: PoseStamped) -> None:
        self.target_pose_.pose.position.x = msg.pose.position.x
        self.target_pose_.pose.position.y = msg.pose.position.y
        self.target_pose_.pose.position.z = msg.pose.position.z
        self.target_pose_.pose.orientation.w = msg.pose.orientation.w
        self.target_pose_.header.stamp = msg.header.stamp

    def drone_is_ready_callback_(self):
        if self.is_armed_ and self.current_mode_ == "OFFBOARD":
            self.drone_is_ready_pub_.publish(Bool(data=True))
        else:
            self.drone_is_ready_pub_.publish(Bool(data=False))

    def position_callback_(self, msg: PoseStamped) -> None:
        self.current_pose_ = msg
        self.is_drone_reached_callback_()

    def state_callback_(self, msg: State) -> None:
        self.current_mode_ = msg.mode
        self.is_armed_ = msg.armed

    def set_offboard_mode(self):
        if self.is_armed_:
            self.get_logger().info("Drone is already armed")
            return
        self.get_logger().info("Waiting for MAVROS to initialize...")
        for _ in range(100):
            self.publish_setpoint_()
            time.sleep(0.1)
        self.get_logger().info("Publishing initial setpoints for OFFBOARD mode...")
        time.sleep(5)
        while not self.mavros_client_.wait_for_service(timeout_sec=1):
            self.get_logger().warn(f"Waiting for {self.drone_name_}/mavros/set_mode")
        msg = SetMode.Request()
        msg.base_mode = 0
        msg.custom_mode = "OFFBOARD"
        future = self.mavros_client_.call_async(msg)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("OFFBOARD mode set successfully")
            else:
                self.get_logger().error("Failed to set OFFBOARD mode")
                return
        except Exception as e:
            self.get_logger().error(f"Failed to call set_mode service: {e}")
            return
        while not self.arm_client_.wait_for_service(timeout_sec=1):
            self.get_logger().warn(f"Waiting for {self.drone_name_}/mavros/cmd/arming")
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
        self.target_pose_.pose.position.z = 10.0

    def publish_setpoint_(self):
        self.target_pose_.header.stamp = self.get_clock().now().to_msg()
        # self.is_drone_reached_callback_()
        # self.get_logger().info(
        #     f"Publishing setpoint position, "
        #     f"{self.target_pose_.pose.position.x}, "
        #     f"{self.target_pose_.pose.position.y}, "
        #     f"{self.target_pose_.pose.position.z}, "
        #     f"{self.target_pose_.pose.orientation.z}",
        # )
        self.setpoint_pub_.publish(self.target_pose_)

    def start_flight_callback_(
        self, request: DronePosition.Request, response: DronePosition.Response
    ):
        if self.current_mode_ != "OFFBOARD" or not self.is_armed_:
            self.get_logger().error("Cannot start flight, drone not configured")
            response.success = False
            return response
        self.target_pose_.pose.position.x = request.x
        self.target_pose_.pose.position.y = request.y
        self.target_pose_.pose.position.z = request.z
        self.target_pose_.pose.orientation.w = 1.0
        # self.get_logger().info(
        #     f"Received coords: x:{request.x} y:{request.y} z:{request.z}"
        # )
        response.success = True
        return response

    def is_drone_reached_callback_(self) -> None:
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
            self._is_drone_reached = False
            self.drone_position_pub.publish(Bool(data=False))
        else:
            self._is_drone_reached = True
            self.drone_position_pub.publish(Bool(data=True))

    def stop_flight_callback_(self):
        pass

    def destroy_node(self):
        """Clean up resources."""
        if self.keyboard_listener is not None:
            self.keyboard_listener.stop()
        super().destroy_node()


def main():
    rclpy.init()
    ground_node = PX4CompanionNode("companion_node")
    try:
        rclpy.spin(ground_node)
    except KeyboardInterrupt:
        pass
    finally:
        ground_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
