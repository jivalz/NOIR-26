import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class DualCameraSubscriber(Node):
    def __init__(self):
        super().__init__('dual_camera_subscriber')

        self.create_subscription(CompressedImage, "camera1/image_compressed", lambda msg: self.image_callback(msg, "Camera 1"), 10)
        self.create_subscription(CompressedImage, "camera2/image_compressed", lambda msg: self.image_callback(msg, "Camera 2"), 10)
        self.create_subscription(CompressedImage, "camera3/image_compressed", lambda msg: self.image_callback(msg, "Camera 3"), 10)

        self.get_logger().info("Dual-Camera Subscriber Node Started")

    def image_callback(self, msg, window_name):
        frame_bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow(window_name, frame_bgr)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DualCameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down dual-camera subscriber")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
