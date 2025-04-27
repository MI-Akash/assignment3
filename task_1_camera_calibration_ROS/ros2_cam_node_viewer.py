import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('ip_stream_image_viewer')

        # subscription to the /ip_stream_image topic
        self.subscription = self.create_subscription(
            Image,
            '/ip_stream_image',
            self.listener_callback,
            10
        )
        self.subscription  

        self.br = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image frame...')

        try:
            
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        
        cv2.imshow("IP Stream Viewer", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
