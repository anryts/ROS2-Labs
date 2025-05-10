import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time


class SwarmControlNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.get_logger().info(f"{node_name} has been started")

        self._drone_position_publisher = self.create_publisher(
            PoseStamped, "/swarm_control/broadcast/position", qos_profile_sensor_data
        )
        self._drone_position_subscriber = self.create_subscription(
            PoseStamped,
            "/swarm_control/broadcast/position",
            self._drone_position_callback,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(0.5, self._drone_position_publisher_callback)

        self._current_position = PoseStamped()
        self._current_position.header.frame_id = "map"
        self._current_position.pose.position.x = 0.0
        self._current_position.pose.position.y = 0.0
        self._current_position.pose.position.z = 0.0
        self._current_position.pose.orientation.w = 1.0
        self._current_position.header.stamp.sec = 0
        self._current_position.header.stamp.nanosec = 0

    def _drone_position_publisher_callback(self) -> None:
        self._drone_position_publisher.publish(self._current_position)

    def _drone_position_callback(self, msg: PoseStamped) -> None:
        """Callback function for the drone position subscriber."""
        if not msg.header.stamp or msg.header.stamp.sec == 0:
            return

        # Check if the incoming message is newer than the current one and convert into seconds
        try:
            msg_time = Time.from_msg(msg.header.stamp)
            current_time = Time.from_msg(self._current_position.header.stamp)
            time_diff = (msg_time.nanoseconds - current_time.nanoseconds) / 1e9

            if time_diff > 0:
                self._current_position = msg
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")
            self._current_position = msg


def main(args=None):
    rclpy.init(args=args)
    swarm_control_node = SwarmControlNode("swarm_control")
    rclpy.spin(swarm_control_node)
    swarm_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "main":
    main()
