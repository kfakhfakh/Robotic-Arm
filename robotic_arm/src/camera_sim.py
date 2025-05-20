#!/home/khaled/cam/bin/python 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

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
        self.sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',  # Change if needed
            self.callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt') 

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_height, frame_width = frame.shape[:2]
            #cv2.imshow('Gazebo Camera', frame) 
            #results = self.model(frame, conf=0.4, iou=0.5)
            #annotated_frame = results[0].plot()
            results = self.model(frame)[0]  # Take the first result
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                dx, dy = calculate_center_error(frame_width, frame_height, (x1, y1, x2, y2))

                print(f"Object detected:")
                print(f" - Bounding Box: {(x1, y1, x2, y2)}")
                print(f" - Center Offset: dx = {dx:.2f}, dy = {dy:.2f}")
                

                # Optionally draw box and center error
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                bbox_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                frame_center = (frame_width // 2, frame_height // 2)
                cv2.circle(frame, bbox_center, 5, (0, 0, 255), -1)
                cv2.circle(frame, frame_center, 5, (255, 0, 0), -1)
                cv2.line(frame, frame_center, bbox_center, (255, 0, 255), 2)
            cv2.imshow('YOLO', frame)
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