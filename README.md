# Robotic Arm Object Tracking System

A ROS 2-based robotic arm system with integrated computer vision capabilities for real-time object tracking and autonomous manipulation.

  <div align="center">
    <img width="581" height="508" alt="Screenshot 2025-12-08 192444" src="https://github.com/user-attachments/assets/c51ce74b-f597-4c45-bd8d-f56cc4e722c0" />
  </div>

## Overview

This project implements a 3-DOF (3-axis) robotic arm with the following key features:

- **3D Robotic Arm**: 3-joint revolute manipulator with camera integration
- **Object Detection**: YOLO-based real-time object detection and tracking
- **Motion Planning**: MoveIt 2 integration for collision-free path planning
- **Simulation**: Gazebo and Ignition support for physics simulation
- **Visual Feedback**: RViz2 visualization and live camera feed display



https://github.com/user-attachments/assets/a0fc2517-989e-4973-ac01-3c92100ace9a


## System Architecture

### Core Components

1. **Robotic Arm** ([robotic_arm/](robotic_arm/))
   - URDF/Xacro robot description
   - ROS 2 control interface
   - Gazebo and Ignition simulation support

2. **Motion Planning** ([moveit_robotic_arm/](moveit_robotic_arm/))
   - MoveIt 2 configuration
   - Trajectory planning and execution
   - Kinematics solver (KDL)

3. **Computer Vision** ([robotic_arm/src/camera_sim.py](robotic_arm/src/camera_sim.py))
   - YOLO object detection
   - Visual servoing control
   - Real-time camera feed processing

## Robot Structure

The robotic arm consists of the following links and joints:

| Link | Joint | Type | Range |
|------|-------|------|-------|
| base_link | world_fixed_joint | Fixed | N/A |
| waist_Link | base_waist_joint | Revolute | ±1.571 rad |
| arm_Link | waist_arm_joint | Revolute | ±1.571 rad |
| end_Link | arm_end_joint | Revolute | ±1.571 rad |
| support_Link | end_support_joint | Fixed | N/A |
| cam_Link | support_cam_joint | Fixed | N/A |
| tip_link | support_tip_joint | Fixed | N/A |

## Installation

### Prerequisites

- ROS 2 (Humble or later)
- MoveIt 2
- Gazebo or Ignition
- OpenCV and cv_bridge
- YOLOv8 (Python)

### Build Instructions

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone https://github.com/yourusername/Robotic-Arm.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the setup script
source install/setup.bash
```

## Usage

### 1. Display Robot in RViz2 (No Simulation)

```bash
ros2 launch robotic_arm display.launch.py
```
<img width="551" height="533" alt="Screenshot 2025-12-08 192456" src="https://github.com/user-attachments/assets/df7eddc5-7b63-4d8f-a985-0f8a327c9c8a" />

This launches the robot state publisher and allows manual joint control via the joint state publisher GUI.

### 2. Gazebo Simulation with MoveIt2

```bash
ros2 launch robotic_arm moveit_gazebo.launch.py
```

<img width="573" height="533" alt="Screenshot 2025-12-08 192512" src="https://github.com/user-attachments/assets/8ed3ddff-d679-4c0f-9e7b-3c87d1840fde" />

Includes:
- Gazebo physics simulation
- RViz2 with MoveIt planning plugin
- Motion planning and execution
- Joint state publishing
  
<img width="970" height="486" alt="Screenshot 2025-12-08 192504" src="https://github.com/user-attachments/assets/53d7917f-0535-4787-8b47-9ce58bfa4dbf" />

### 3. Standard Gazebo Launch

```bash
ros2 launch robotic_arm gazebo.launch.py
```

### 4. Run Object Tracking

In a separate terminal:

```bash
ros2 run robotic_arm camera_sim
```

This starts the YOLO-based object tracking system that:
- Detects objects in the camera feed
- Calculates center errors
- Publishes joint trajectory commands
- Visualizes detection results with OpenCV
  
<img width="769" height="388" alt="Screenshot 2025-12-08 192520" src="https://github.com/user-attachments/assets/fb53bd97-1dd2-4078-bf51-89f775ebedf0" />


### 5. Test Motion Planning

```bash
ros2 run robotic_arm position
```

Executes a pre-defined target pose for the end-effector.

## Visual Servoing & PID Control

The robotic arm’s object tracking is powered by **visual servoing** combined with a **PID-based control loop** for smooth and responsive motion.

### How It Works

1. YOLO detects the object and outputs a bounding box.  
2. The pixel error between the object center and image center is calculated:

    `ex = x_target - x_center`  
    `ey = y_target - y_center`

3. The error is fed into a **PID controller** for each joint to produce smooth velocity commands.  
4. The arm moves to reduce the error in real-time, creating a **closed-loop tracking system**.

### PID Controller Diagram

<img width="1020" height="329" alt="Screenshot 2025-12-08 192538" src="https://github.com/user-attachments/assets/a8c32985-292d-49db-a810-b3a3b64cdcf3" />

### Example Gain Parameters (`camera_sim.py`)

```python
Kp = 2.0
Ki = 0.0
Kd = 0.3
```

## Video demo

![demo1-MadewithClipchamp-ezgif com-video-to-gif-converter (2)](https://github.com/user-attachments/assets/9a136317-f6f4-4d5d-b03c-2e53ccbeef1a)

## Configuration Files

### Robot Control
- [ros2_controllers.yaml](moveit_robotic_arm/config/ros2_controllers.yaml) - Controller parameters
- [joint_limits.yaml](moveit_robotic_arm/config/joint_limits.yaml) - Joint velocity/acceleration limits
- [Robot_Arm_control.xacro](robotic_arm/urdf/Robot_Arm_control.xacro) - ROS 2 Control plugin configuration

### Motion Planning
- [kinematics.yaml](moveit_robotic_arm/config/kinematics.yaml) - IK solver configuration
- [moveit_controllers.yaml](moveit_robotic_arm/config/moveit_controllers.yaml) - MoveIt controller definitions

### Simulation
- [Robot_Arm_gazebo.xacro](robotic_arm/urdf/Robot_Arm_gazebo.xacro) - Gazebo/Ignition plugins
- [servo.yaml](moveit_robotic_arm/config/servo.yaml) - Real-time servo control settings

## Key Features

### Object Tracking
The [camera_sim.py](robotic_arm/src/camera_sim.py) script provides:
- Real-time YOLO object detection
- Visual servoing with error-based control
- Joint angle adjustments based on detected object position
- OpenCV visualization with tracking indicators

### Motion Planning
The [test.cpp](robotic_arm/src/test.cpp) demonstrates:
- MoveIt 2 move group interface
- Cartesian pose targeting
- Trajectory planning and execution
- Position and orientation control

### Controllers
- **arm_group_controller**: Joint trajectory controller for planned motions
- **arm_vel_controller**: Velocity controller for servo applications
- **joint_state_broadcaster**: Publishes joint states to ROS 2

## Topics and Services

### Published Topics
- `/robot_description` - URDF robot model
- `/joint_states` - Current joint positions and velocities
- `/arm_group_controller/follow_joint_trajectory/feedback` - Motion feedback
- `/camera_sensor/image_raw` - Camera feed from simulation

### Subscribed Topics
- `/joint_states` - Joint state feedback
- `/camera_sensor/image_raw` - Camera input for tracking

### Action Servers
- `/arm_group_controller/follow_joint_trajectory` - Trajectory execution action

## Parameters

### Joint Limits
```yaml
- base_waist_joint: ±1.571 rad, 5.0 rad/s max velocity
- waist_arm_joint: ±1.571 rad, 5.0 rad/s max velocity
- arm_end_joint: ±1.571 rad, 5.0 rad/s max velocity
```

### Tracking Control Gains
Modify in [camera_sim.py](robotic_arm/src/camera_sim.py):
```python
base_delta = -dx_norm * 2      # Base rotation gain
waist_delta = dy_norm * 4      # Waist pitch gain
end_delta = dy_norm * 2        # End effector adjustment
```

## Troubleshooting

### Robot not moving in Gazebo
- Ensure controllers are spawned: `ros2 control list_controllers`
- Check controller manager status
- Verify joint trajectory commands are being published

### Camera feed not appearing
- Check if Gazebo is running: `gazebo_ros2_control` plugin must be loaded
- Verify camera sensor is configured in [Robot_Arm_gazebo.xacro](robotic_arm/urdf/Robot_Arm_gazebo.xacro)

### MoveIt planning failures
- Verify kinematics solver timeout in [kinematics.yaml](moveit_robotic_arm/config/kinematics.yaml)
- Check joint limits in [joint_limits.yaml](moveit_robotic_arm/config/joint_limits.yaml)
- Ensure start state is valid and not in collision

