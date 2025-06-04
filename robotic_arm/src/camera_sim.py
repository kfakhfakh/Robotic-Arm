#!/home/khaled/cam/bin/python 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

def calculate_center_error(frame_width, frame_height, bbox):
    frame_center_x = frame_width / 2
    frame_center_y = frame_height / 2
    x1, y1, x2, y2 = bbox
    bbox_center_x = (x1 + x2) / 2
    bbox_center_y = (y1 + y2) / 2
    dx = bbox_center_x - frame_center_x
    dy = bbox_center_y - frame_center_y
    return dx, dy

class GazeboCamNode(Node):
    def __init__(self):
        super().__init__('gazebo_cam_sub')
        self.sub = self.create_subscription(Image, '/camera_sensor/image_raw', self.callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_group_controller/joint_trajectory', 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt') 
        self.joint_positions = {
            'base_waist_joint': 0.0,
            'waist_arm_joint': 1.5,
            'arm_end_joint': 0.0
        }

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_positions:
                self.joint_positions[name] = position

    def publish_joint_angles(self, base_delta, waist_delta, end_delta):
        new_positions = [
            self.joint_positions['base_waist_joint'] + base_delta,
            self.joint_positions['waist_arm_joint'] + waist_delta,
            self.joint_positions['arm_end_joint'] + end_delta
        ]
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['base_waist_joint', 'waist_arm_joint', 'arm_end_joint']
        point = JointTrajectoryPoint()
        point.positions = new_positions
        point.time_from_start.sec = 1
        traj_msg.points.append(point)
        self.joint_pub.publish(traj_msg)

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_height, frame_width = frame.shape[:2]
            results = self.model(frame)[0]
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                dx, dy = calculate_center_error(frame_width, frame_height, (x1, y1, x2, y2))

                # Normalize error to angle deltas
                dx_norm = dx / frame_width  # Range ~ [-0.5, 0.5]
                dy_norm = dy / frame_height
                base_delta = -dx_norm * 4 # Gain factor
                waist_delta = dy_norm * 4
                end_delta = dy_norm * 2  # Less influence for the last joint

                self.get_logger().info(f"dx: {dx:.2f}, dy: {dy:.2f}, base_delta: {base_delta:.3f}")

                self.publish_joint_angles(base_delta, waist_delta, end_delta)

                # Visualization
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                bbox_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                frame_center = (frame_width // 2, frame_height // 2)
                cv2.circle(frame, bbox_center, 5, (0, 0, 255), -1)
                cv2.circle(frame, frame_center, 5, (255, 0, 0), -1)
                cv2.line(frame, frame_center, bbox_center, (255, 0, 255), 2)

            cv2.imshow('YOLO Tracking', frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()