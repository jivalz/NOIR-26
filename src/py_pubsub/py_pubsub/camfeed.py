import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class MultiCameraPublisher(Node):
    def __init__(self):
        super().__init__('multi_camera_publisher')

        # Declare parameters (adjust anytime via ros2 param set)
        self.declare_parameter('fps', 20)                    # Frames per second
        self.declare_parameter('frame_width', 320)           # Frame width
        self.declare_parameter('frame_height', 240)          # Frame height
        self.declare_parameter('jpeg_quality', 60)           # JPEG compression quality (0–100)
        self.declare_parameter('camera_indices', [0, 2, 4])  # Camera device indices

        # Retrieve parameters
        fps = self.get_parameter('fps').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        quality = self.get_parameter('jpeg_quality').value
        cam_indices = self.get_parameter('camera_indices').value

        # Store parameters
        self.quality = quality
        self.caps = []
        self.pubs = []

        # Initialize each camera and publisher
        for i, idx in enumerate(cam_indices):
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                self.get_logger().error(f"Could not open camera index {idx}")
                continue

            cap.set(cv2.CAP_PROP_FPS, fps)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # Helps USB bandwidth

            self.caps.append(cap)
            topic_name = f"camera{i+1}/image_compressed"
            pub = self.create_publisher(CompressedImage, topic_name, 10)
            self.pubs.append(pub)
            self.get_logger().info(f"Initialized {topic_name} from /dev/video{idx}")

        # Create timer
        self.timer = self.create_timer(1.0 / fps, self.publish_frames)
        self.get_logger().info(
            f"MultiCameraPublisher running at {fps} FPS with {width}x{height} resolution and JPEG {quality}"
        )

    def publish_frames(self):
        for i, cap in enumerate(self.caps):
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f"Failed to grab frame from camera{i+1}")
                continue

            # JPEG compression
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
            success, compressed_frame = cv2.imencode('.jpg', frame, encode_params)
            if not success:
                self.get_logger().warn(f"Compression failed on camera{i+1}")
                continue

            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = compressed_frame.tobytes()
            self.pubs[i].publish(msg)

    def destroy_node(self):
        for cap in self.caps:
            cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MultiCameraPublisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
