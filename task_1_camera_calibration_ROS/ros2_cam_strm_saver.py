import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class TimedImageSaver(Node):
    def __init__(self):
        super().__init__('timed_image_saver')

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/ip_stream_image',
            self.listener_callback,
            10
        )

        self.br = CvBridge()
        self.last_saved_time = time.time()
        self.save_interval = 5  
        self.image_count = 0

        
        self.image_folder = 'images'
        os.makedirs(self.image_folder, exist_ok=True)

    def listener_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_saved_time >= self.save_interval:
            try:
                
                cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                
                filename = os.path.join(self.image_folder, f'image{self.image_count}.png')
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f"Saved: {filename}")

                self.image_count += 1
                self.last_saved_time = current_time

            except Exception as e:
                self.get_logger().error(f"Failed to save image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TimedImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
