#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import time
import cv2
import numpy as np

class CubePoseEstimator(Node):
    def __init__(self):
        super().__init__('cube_pose_estimator')

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        self.subscription_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.subscription_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)

        self.get_logger().info("Subscribed to image and camera_info topics")

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_info_received = True
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera intrinsics received")

    def image_callback(self, msg):
        if not self.camera_info_received:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_display = frame.copy()

            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define green range in HSV
            lower_green = np.array([40, 50, 50])
            upper_green = np.array([80, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)

            # Morphological cleaning
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

            # Find contours in the green mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            cube_face = None
            max_area = 0

            for cnt in contours:
                epsilon = 0.02 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)

                if len(approx) == 4 and cv2.isContourConvex(approx):
                    area = cv2.contourArea(approx)
                    if area > max_area and area > 1000:  # Minimum area filter
                        max_area = area
                        cube_face = approx

            if cube_face is not None:
                cv2.drawContours(frame_display, [cube_face], -1, (0, 255, 0), 2)

                corners = cube_face.reshape(4, 2)
                corners = self.sort_corners(corners)

                for i, pt in enumerate(corners):
                    cv2.circle(frame_display, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)
                    cv2.putText(frame_display, str(i), (int(pt[0]), int(pt[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                object_points = np.array([
                    [-0.86741998, -2.54772625,  0.08922], #bottom_left_front
                    [-0.96741975, -2.54751398,  0.089224], # bottom_right_front
                    [-0.96741975, -2.54751398,  0.189224], # top_right_front
                    [-0.86741998, -2.54772625,  0.189224], # top_left_front
                ], dtype=np.float32)

                image_points = corners.astype(np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs
                )

                if success:
                    R, _ = cv2.Rodrigues(rvec)
                    t = tvec

                    R_cam = R.T
                    t_cam = -R_cam @ t

                    print("\n--- Object Pose (Camera-to-Object) ---")
                    print("Rotation Vector (rvec):\n", rvec)
                    print("Translation Vector (tvec):\n", tvec)

                    print("\n--- Estimated Camera Pose in World Coordinates ---")
                    print("Rotation Matrix (R_cam):\n", R_cam)
                    print("Translation Vector (t_cam):\n", t_cam)
                    time.sleep(0.5)

                    cv2.drawFrameAxes(frame_display, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                else:
                    self.get_logger().warn("solvePnP failed")

            else:
                self.get_logger().warn("No green square (cube face) detected")

            cv2.imshow("Camera Pose Estimation", frame_display)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")


    def sort_corners(self, pts):
        # Sort points: top-left, top-right, bottom-right, bottom-left
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)

        rect[0] = pts[np.argmin(s)]  # top-left
        rect[2] = pts[np.argmax(s)]  # bottom-right
        rect[1] = pts[np.argmin(diff)]  # top-right
        rect[3] = pts[np.argmax(diff)]  # bottom-left
        return rect

def main(args=None):
    rclpy.init(args=args)
    node = CubePoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
