#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class GazeboCamNode(Node):
    def __init__(self):
        super().__init__('gazebo_cam_sub')
        self.sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',  # Change if needed
            self.callback,
            10)
        self.bridge = CvBridge()

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('Gazebo Camera', frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GazeboCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()