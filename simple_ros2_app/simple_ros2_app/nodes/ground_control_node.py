import time
from math import cos, sin, pi

import rclpy
from rclpy.node import Node
from simple_ros2_app.helpers.coordinate import Coordinate
from simple_ros2_app.helpers.drone import Drone


class GroundControlNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.drones_names = ["/uav0", "/uav1"]
        self.drones = [Drone(drone_name, self) for drone_name in self.drones_names]

        self.coordinates_: list[Coordinate] = [
            Coordinate(5.0, 5.0, 5.0),
            Coordinate(20.0, 20.0, 20.0),
            Coordinate(35.0, 25.0, 13.0),
            Coordinate(44.0, 30.0, 8.0),
        ]
        self._circle_coordinates: list[Coordinate] = self.generate_circle_coordinates(
            radius=25.0, num_points=20
        )
        self.current_index_: int = 0
        self.timer_ = self.create_timer(1, self.send_command_)
        self._done_with_predefined_coordinates: bool = False
        self.target_coordinates_: Coordinate = Coordinate(0.0, 0.0, 0.0)

    def send_command_(self):
        """Send the next set of coords to the PX4CompanionNode."""

        if not all(drone.is_drone_reached_coord for drone in self.drones):
            self.get_logger().info("Waiting for all drones to reach the coordinates")
            return
        if self.current_index_ == 0 and not self._done_with_predefined_coordinates:
            time.sleep(10)
            self.get_logger().info("All drones reached the coordinates")
            time.sleep(10)
        for drone in self.drones:
            drone.set_target_coordinates(self.coordinates_[self.current_index_])

        if self.current_index_ < len(self.coordinates_) - 1:
            if not self._done_with_predefined_coordinates:
                time.sleep(10)
            self.get_logger().info(
                f"All drones reached the coordinates {self.coordinates_[self.current_index_]}"
            )
            self.target_coordinates_ = self.coordinates_[self.current_index_]
            self.current_index_ = self.current_index_ + 1
        # else:
        #     self.get_logger().info(
        #         "All drones reached the coordinates, starting the circle pattern"
        #     )
        #     self.current_index_ = 0
        #     self.coordinates_ = self._circle_coordinates
        #     self._done_with_predefined_coordinates = True
        #     self.target_coordinates_ = self.coordinates_[self.current_index_]

    def generate_circle_coordinates(
        self, radius: float, num_points: int
    ) -> list[Coordinate]:
        """Generate a list of coordinates in a circle pattern.

        Args:
            radius (float): The radius of the circle.
            num_points (int): The number of points to generate.

        Returns:
            list[Coordinate]: A list of coordinates in a circle pattern.
        """
        coordinates = []
        for i in range(num_points):
            angle = 2 * pi * i / num_points
            x = radius * cos(angle)
            y = radius * sin(angle)
            coordinates.append(Coordinate(x, y, 10.0))
        return coordinates


def main():
    rclpy.init()
    ground_node = GroundControlNode("ground_control")
    rclpy.spin(ground_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
