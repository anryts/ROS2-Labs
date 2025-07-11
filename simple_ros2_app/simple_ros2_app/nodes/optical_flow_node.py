import cv2
import rclpy
import rclpy.qos as qos
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

from simple_ros2_interfaces.msg import DroneDetection


class OpticalFlowNode(Node):
    """Processes drone camera images with YOLOv8 to detect another drone and publishes results."""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.get_logger().info("Optical Flow Node with YOLOv8 has been started.")
        self.drone_name = self.get_namespace()
        self.cv_bridge = CvBridge()
        self.prev_frame = None
        self.detected_box: DroneDetection = None

        # Load YOLOv8 model
        self.yolo_model = YOLO("yolov8n.pt")
        self.target_class = "airplane"

        # Camera subscription
        self.drone_camera = self.create_subscription(
            Image,
            f"{self.drone_name}/camera/image_raw",
            self.image_processing_callback,
            qos.qos_profile_sensor_data,
        )

        # Publisher for detected drone bounding box
        self.drone_detection_pub = self.create_publisher(
            DroneDetection,
            f"{self.drone_name}/drone_detection",
            qos.qos_profile_sensor_data,
        )
        self._img = None
        self._timer = self.create_timer(1, self._timer_callback)

    def _timer_callback(self):
        self._model_inference(self._img)

    def image_processing_callback(self, msg: Image):
        """Process camera image for optical flow and YOLO detection."""
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return
        overlay = cv_img.copy()

        self._optical_flow(cv_img, overlay)
        self._tracking_callback(cv_img, overlay)
        cv2.imshow("Optical Flow and YOLO", overlay)
        self._img = cv_img
        cv2.waitKey(1)

    def _tracking_callback(self, img, overlay) -> None:
        """Callback for tracking drone by MOSSE tracker."""
        if self.detected_box is None:
            self._model_inference(img)
            return

        tracker = cv2.TrackerMIL.create()
        width = self.detected_box.x_max - self.detected_box.x_min
        height = self.detected_box.y_max - self.detected_box.y_min
        if width < 0 or height < 0:
            self._model_inference(img)
            return

        tracker.init(
            img,
            (
                self.detected_box.x_min,
                self.detected_box.y_min,
                width,
                height,
            ),
        )
        success, bbox = tracker.update(img)
        if success:
            x, y, w, h = [int(v) for v in bbox]
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            box_msg = DroneDetection()
            box_msg.x_min = x
            box_msg.y_min = y
            box_msg.x_max = x + w
            box_msg.y_max = y + h
            self.detected_box = box_msg
            self.drone_detection_pub.publish(self.detected_box)
            cv2.rectangle(
                overlay,
                (x, y),
                (x + w, y + h),
                (213, 36, 235),
                2,
            )
        else:
            self.get_logger().info("Tracking failed, reinitializing tracker.")
            self.detected_box = None
            self._model_inference(img)

    def _model_inference(self, cv_img) -> None:
        """Run YOLO model inference on the given image."""
        if cv_img is None:
            return
        results = self.yolo_model.predict(cv_img, verbose=False)
        best_box = None
        max_confidence = 0.0
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.yolo_model.names[cls_id]
                if cls_name == self.target_class and box.conf[0] > max_confidence:
                    max_confidence = box.conf[0]
                    best_box = box

        # Publish the best bounding box
        if best_box is not None:
            box_msg = DroneDetection()
            # box_msg.probability = float(best_box.conf[0])
            box_msg.x_min = int(best_box.xyxy[0][0])
            box_msg.y_min = int(best_box.xyxy[0][1])
            box_msg.x_max = int(best_box.xyxy[0][2])
            box_msg.y_max = int(best_box.xyxy[0][3])
            # box_msg.class_name = self.target_class
            self.detected_box = box_msg

        # Draw bounding box if detected
        # if best_box is not None:
        #     cv2.rectangle(
        #         overlay,
        #         (int(best_box.xyxy[0][0]), int(best_box.xyxy[0][1])),
        #         (int(best_box.xyxy[0][2]), int(best_box.xyxy[0][3])),
        #         (255, 0, 0),
        #         2,
        #     )

    def _optical_flow(self, cv_img, overlay: cv2.Mat) -> None:
        """Process the incoming image message for optical flow."""

        # Convert to grayscale
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        if self.prev_frame is None:
            self.prev_frame = gray
            return

        # Calculate optical flow
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

        # Create overlay for visualization
        step = 16
        h, w = gray.shape
        for y in range(0, h, step):
            for x in range(0, w, step):
                fx, fy = flow[y, x]
                start_point = (x, y)
                end_point = (int(x + fx), int(y + fy))
                cv2.arrowedLine(
                    overlay, start_point, end_point, (0, 255, 0), 2, tipLength=0.4
                )
        # Update previous frame
        self.prev_frame = gray

    def destroy_node(self):
        """Clean up OpenCV windows and YOLO model."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode("optical_flow")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
