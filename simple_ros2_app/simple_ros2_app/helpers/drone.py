import rclpy.qos as qos
from rclpy.node import Node
from simple_ros2_app.helpers.coordinate import Coordinate
from std_msgs.msg import Bool

from simple_ros2_interfaces.srv import DronePosition


class Drone:
    """
    This class represents a drone and provides methods to interact with it.
    It allows setting target coordinates, checking if the drone has reached its destination,
    and checking if the drone is ready for flight.
    """

    def __init__(self, drone_name: str, node: Node) -> None:
        self.node = node
        self.drone_name_ = drone_name
        self.start_flight_client_ = node.create_client(
            DronePosition, f"{drone_name}/companion_node/start_flight"
        )
        self.drone_reached_position_sub_ = node.create_subscription(
            Bool,
            f"{drone_name}/companion_node/is_reached",
            self.is_drone_reached_callback_,
            qos.qos_profile_sensor_data,
        )
        self.drone_is_ready_sub_ = node.create_subscription(
            Bool,
            f"{drone_name}/companion_node/is_ready",
            self.is_drone_ready_callback_,
            qos.qos_profile_sensor_data,
        )
        self.drone_is_ready: bool = False
        self.is_drone_reached_coord: bool = False
        self.node.get_logger().info(
            f"service for is_reached: {drone_name}/companion_node/is_reached"
        )

    def is_drone_reached_callback_(self, msg: Bool) -> None:
        """
        Callback function to handle the drone's reached position status.
        """
        self.is_drone_reached_coord = msg.data
        return

    def is_drone_ready_callback_(self, msg: Bool) -> None:
        """
        Callback function to handle the drone's readiness status.
        """
        self.drone_is_ready = msg.data
        return

    def set_target_coordinates(self, position: Coordinate) -> None:
        """
        Set the target coordinates for the drone.
        Args:
            position (DronePosition): The target coordinates.
        """
        if (
            not self.start_flight_client_.wait_for_service(timeout_sec=1.0)
            and not self.drone_is_ready
        ):
            self.node.get_logger().info(
                f"Waiting for {self.drone_name_}/companion_node/start_flight"
            )
            return
        request = DronePosition.Request()
        request.x = position.x
        request.y = position.y
        request.z = position.z
        self.target_coordinates_ = position
        future = self.start_flight_client_.call_async(request)
        future.add_done_callback(self._handle_start_flight_response)
        self.is_drone_reached_coord = False

    def _handle_start_flight_response(self, future):
        try:
            response = future.result()
            self.node.get_logger().info(
                f"Drone {self.drone_name_} received response: {response}"
            )
        except Exception as e:
            self.node.get_logger().error(f"Failed to call service: {e}")
