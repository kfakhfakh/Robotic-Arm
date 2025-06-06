#!/home/khaled/cam/bin/python 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image as Img, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import time
from collections import deque

def calculate_center_error(frame_width, frame_height, bbox):
    frame_center_x = frame_width / 2
    frame_center_y = frame_height / 2
    x1, y1, x2, y2 = bbox
    bbox_center_x = (x1 + x2) / 2
    bbox_center_y = (y1 + y2) / 2
    dx = bbox_center_x - frame_center_x
    dy = bbox_center_y - frame_center_y
    return dx, dy

class PIDController:
    def __init__(self, kp=0, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

    def compute(self, error):
        now = time.time()
        dt = now - self.last_time if self.last_time else 0.0
        self.last_time = now

        self.integral += error * dt if dt > 0 else 0.0
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error

        return self.kp * error + self.ki * self.integral + self.kd * derivative

class GazeboCamNode(Node):
    def __init__(self, gui):
        super().__init__('gazebo_cam_sub')
        self.gui = gui
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
        self.dx_history = deque(maxlen=100)
        self.dy_history = deque(maxlen=100)

        self.base_pid = PIDController()
        self.dy_pid = PIDController()

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

            dx, dy = 0, 0
            if results.boxes:
                box = results.boxes[0]
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                dx, dy = calculate_center_error(frame_width, frame_height, (x1, y1, x2, y2))
                self.dx_history.append(dx)
                self.dy_history.append(dy)

                if self.gui.follow_var.get():
                    dx_norm = dx / frame_width
                    dy_norm = dy / frame_height

                    pid = self.gui.get_pid_params()
                    self.base_pid.kp, self.base_pid.ki, self.base_pid.kd = pid['kp1'], pid['ki1'], pid['kd1']
                    self.dy_pid.kp, self.dy_pid.ki, self.dy_pid.kd = pid['kp2'], pid['ki2'], pid['kd2']

                    base_delta = -self.base_pid.compute(dx_norm)
                    dy_output = self.dy_pid.compute(dy_norm)

                    ratio = pid['ratio']
                    waist_delta = dy_output * ratio
                    end_delta = dy_output * (1 - ratio)

                    self.publish_joint_angles(base_delta, waist_delta, end_delta)

                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                bbox_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                frame_center = (frame_width // 2, frame_height // 2)
                cv2.circle(frame, bbox_center, 5, (0, 0, 255), -1)
                cv2.circle(frame, frame_center, 5, (255, 0, 0), -1)
                cv2.line(frame, frame_center, bbox_center, (255, 0, 255), 2)

            self.gui.update_display(frame, list(self.dx_history), list(self.dy_history))
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

class GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Target Tracking PID Controller")
        self.follow_var = tk.BooleanVar(value=False)
        self.graph_var = tk.BooleanVar(value=False)

        self.pid_params = {
            'kp1': 4.0, 'ki1': 0.0, 'kd1': 0.0,
            'kp2': 4.0, 'ki2': 0.0, 'kd2': 0.0,
            'ratio': 0.5
        }

        self.setup_gui()
        self.latest_frame = None

    def setup_gui(self):
        control = tk.Frame(self.root)
        control.pack(side=tk.LEFT, fill=tk.Y)

        tk.Checkbutton(control, text="Enable Target Following", variable=self.follow_var).pack(anchor='w')

        pid_frame = tk.LabelFrame(control, text="PID Parameters")
        pid_frame.pack(pady=5)
        self.entries = {}
        for i, key in enumerate(self.pid_params):
            tk.Label(pid_frame, text=key).grid(row=i, column=0, sticky='e')
            entry = tk.Entry(pid_frame, width=6)
            entry.grid(row=i, column=1)
            entry.insert(0, str(self.pid_params[key]))
            self.entries[key] = entry

        tk.Button(control, text="Submit", command=self.submit_params).pack(pady=5)
        tk.Checkbutton(control, text="Show Graph", variable=self.graph_var, command=self.toggle_graph).pack(anchor='w')

        self.image_label = tk.Label(self.root)
        self.image_label.pack(side=tk.TOP)

        self.fig, self.ax = plt.subplots(figsize=(5, 2))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.plot_widget = self.canvas.get_tk_widget()

    def submit_params(self):
        for k, entry in self.entries.items():
            self.pid_params[k] = float(entry.get())

    def get_pid_params(self):
        return self.pid_params

    def toggle_graph(self):
        if self.graph_var.get():
            self.plot_widget.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        else:
            self.plot_widget.pack_forget()

    def update_display(self, frame, dx_data, dy_data):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_pil = Img.fromarray(rgb)
        img_pil = img_pil.resize((800, 800)) if self.graph_var.get() else img_pil
        img_tk = ImageTk.PhotoImage(img_pil)
        self.image_label.configure(image=img_tk)
        self.image_label.image = img_tk

        if self.graph_var.get():
            self.ax.clear()
            self.ax.plot(dx_data, label='dx')
            self.ax.plot(dy_data, label='dy')
            self.ax.legend()
            self.canvas.draw()

def main():
    rclpy.init()
    root = tk.Tk()
    gui = GUI(root)
    node = GazeboCamNode(gui)

    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()