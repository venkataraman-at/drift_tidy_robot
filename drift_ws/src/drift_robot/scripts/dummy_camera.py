#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

class DummyCamera(Node):
    def __init__(self):
        super().__init__("dummy_camera")
        self.img_pub  = self.create_publisher(Image, "/drift_robot/camera/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/drift_robot/camera/camera_info", 10)
        self.timer = self.create_timer(1.0/15.0, self._publish)
        self.get_logger().info("Dummy camera publisher started.")

    def _publish(self):
        now = self.get_clock().now().to_msg()
        h = Header()
        h.stamp = now
        h.frame_id = "camera_optical_link"
        img = Image()
        img.header = h
        img.height = 480
        img.width  = 640
        img.encoding = "rgb8"
        img.is_bigendian = 0
        img.step = 640 * 3
        img.data = bytes([100, 120, 160] * (640 * 480))
        self.img_pub.publish(img)

def main(args=None):
    rclpy.init(args=args)
    node = DummyCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
