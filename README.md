

# 🤖 Object-Tracking Robotic Arm using ROS 2, MoveIt 2, and Gazebo

This project simulates a robotic arm that dynamically **tracks and follows a moving object** in a 3D environment using **ROS 2**, **Gazebo**, and **MoveIt 2**.

---

## 🛠️ Tech Stack

- **ROS 2 Humble (or other version)**
- **Gazebo Classic** (or Gazebo Fortress/Ignition)
- **MoveIt 2**
- **RViz 2**
- **Python / C++** (for nodes and control logic)

---

## 📦 Project Structure

robotic_arm/
├── CMakeLists.txt
├── include
│   └── robotic_arm
├── launch
│   ├── display.launch
│   ├── display.launch.py
│   ├── gazebo.launch.py
│   └── moveit_gazebo.launch.py
├── meshes
│   ├── arm_Link.STL
│   ├── base_link.STL
│   ├── cam_Link.STL
│   ├── end_Link.STL
│   ├── support_Link.STL
│   └── waist_Link.STL
├── package.xml
├── rviz
│   └── rviz_config.rviz
├── src
│   ├── camera_sim.py
│   └── test.cpp
└── urdf
    ├── Robot_Arm_control.xacro
    ├── Robot_Arm_gazebo.xacro
    └── Robot_Arm.xacro

moveit_robotic_arm
├── CMakeLists.txt
├── config
│   ├── initial_positions.yaml
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── moveit_controllers.yaml
│   ├── moveit.rviz
│   ├── pilz_cartesian_limits.yaml
│   ├── robotic_arm.ros2_control.xacro
│   ├── robotic_arm.srdf
│   ├── robotic_arm.urdf.xacro
│   └── ros2_controllers.yaml
├── launch
│   ├── demo.launch.py
│   ├── move_group.launch.py
│   ├── moveit.launch.py
│   ├── moveit_rviz.launch.py
│   ├── robotic_arm_controller.launch.py
│   ├── rsp.launch.py
│   ├── setup_assistant.launch.py
│   ├── spawn_controllers.launch.py
│   ├── static_virtual_joint_tfs.launch.py
│   └── warehouse_db.launch.py
└── package.xml


