#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import tf2_ros
import tf_transformations
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector")

        self.declare_parameter("show_image", True)
        self.declare_parameter("min_contour_area", 100.0)
        self.show_image = self.get_parameter("show_image").value
        self.min_contour_area = self.get_parameter("min_contour_area").value

        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.camera_info_callback, 10
        )

        self.coords_pub = self.create_publisher(String, "/color_coordinates", 10)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Defaults match the simulated camera and are replaced by CameraInfo.
        self.fx = 585.756
        self.fy = 585.756
        self.cx = 320.0
        self.cy = 160.0
        self.last_log_ns = {}

        self.color_ranges = {
            "R": [
                (np.array((0, 80, 50)), np.array((10, 255, 255))),
                (np.array((170, 80, 50)), np.array((179, 255, 255))),
            ],
            "G": [(np.array((40, 80, 50)), np.array((85, 255, 255)))],
            "B": [(np.array((90, 80, 50)), np.array((140, 255, 255)))],
        }
        self.morphology_kernel = np.ones((3, 3), np.uint8)

        if self.show_image:
            cv2.namedWindow("Color Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Color Detection", 640, 320)

        self.get_logger().info("Color detector started; waiting for camera images")

    def camera_info_callback(self, msg):
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for color_id, ranges in self.color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, self.morphology_kernel, iterations=1
            )
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(contour) < self.min_contour_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            cx_pix, cy_pix = x + w // 2, y + h // 2

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(
                frame,
                color_id,
                (x, max(y - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

            # Keep the project's existing calibrated mapping into camera_link.
            z = 0.1
            camera_y = (cx_pix - self.cx) * z / self.fx * -10
            camera_x = (cy_pix - self.cy) * z / self.fy

            try:
                transform = self.tf_buffer.lookup_transform(
                    "panda_link0",
                    "camera_link",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                translation = np.array(
                    [
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    ]
                )
                rotation = [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]

                transform_matrix = tf_transformations.quaternion_matrix(rotation)
                transform_matrix[:3, 3] = translation
                point_base = transform_matrix @ np.array(
                    [camera_x, camera_y, z, 1.0]
                )

                if color_id == "B":
                    point_base[1] -= 0.0215
                elif color_id == "G":
                    point_base[1] += 0.01

                message = (
                    f"{color_id},{point_base[0]:.3f},"
                    f"{point_base[1]:.3f},{point_base[2]:.3f}"
                )
                self.coords_pub.publish(String(data=message))
                self.log_detection(color_id, message)
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                self.get_logger().warn(f"TF lookup failed: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error in TF transform: {e}")

        if self.show_image:
            try:
                cv2.imshow("Color Detection", frame)
                cv2.waitKey(1)
            except cv2.error as e:
                self.get_logger().warn(f"OpenCV display error: {e}")
                self.show_image = False

    def log_detection(self, color_id, message):
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self.last_log_ns.get(color_id, -1_000_000_000)
        if now_ns < last_ns or now_ns - last_ns >= 1_000_000_000:
            self.get_logger().info(message)
            self.last_log_ns[color_id] = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
