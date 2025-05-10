import cv2
import rclpy
import rclpy.qos as qos
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class OpticalFlowNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.get_logger().info("Optical Flow Node has been started.")
        # Initialize your optical flow algorithm here
        # For example, you can subscribe to image topics or set up publishers
        # self.image_subscriber = self.create_subscription(...)
        # self.publisher = self.create_publisher(...)
        self.drone_name = self.get_namespace()

        # Camera topic subscription
        self.cv_bridge = CvBridge()
        self.drone_camera = self.create_subscription(
            Image,
            f"{self.drone_name}/camera/image_raw",
            self.image_processing_callback,
            qos.qos_profile_sensor_data,
        )
        self.prev_frame = None

    def image_processing_callback(self, msg: Image):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        if self.prev_frame is None:
            self.prev_frame = gray
            return

        # Compute dense optical flow using Farneback method
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_frame,
            gray,
            None,
            pyr_scale=0.5,
            levels=3,
            winsize=15,
            iterations=3,
            poly_n=5,
            poly_sigma=1.2,
            flags=0,
        )

        # Create an overlay on the original image, drawing lines for sampled grid points.
        overlay = cv_img.copy()
        step = 16  # grid step for sampling flow vectors
        h, w = gray.shape
        for y in range(0, h, step):
            for x in range(0, w, step):
                fx, fy = flow[y, x]
                start_point = (x, y)
                end_point = (int(x + fx), int(y + fy))
                cv2.arrowedLine(
                    overlay, start_point, end_point, (0, 255, 0), 2, tipLength=0.4
                )

        # Display the original video with movement lines overlay
        cv2.imshow("Results", overlay)
        cv2.waitKey(1)

        # Update previous frame for next computation
        self.prev_frame = gray


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode("optical_flow")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "main":
    main(args=None)
