# Diablo Bot

## Overview
This project is a robotic system designed for simulation and motion planning using **ROS2 (Humble)** and **Gazebo**. It leverages **MoveIt2** for motion planning, **KDL** for dynamics, and **Gazebo** for simulation. The repository follows a **colcon ROS2 workspace** structure, containing packages for robot description, configuration, and motion planning.

## Dependencies
The project has been tested on **Ubuntu 22.04.3 LTS** and depends on the following software:

- **ROS2 (Humble)** - Framework for robot operation.
  - [Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **Gazebo Classic** - Robot simulation environment.
  - *Note:* Gazebo Classic goes **EOL in Jan 2025**, install the latest version supported for ROS Humble.
- **KDL** - Used for dynamics and motion interpolation.
- **MoveIt2** - Motion planning framework.
  - [Installation Guide](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

## Repository Structure
This repository is organized as a **colcon ROS2 workspace**, consisting of the following packages:

### **diablo_bot**
- **config/** - Configuration files for dynamics, controller manager, and kinematics solver.
- **description/** - URDF description, ROS controllers, and Gazebo plugin files.
- **meshes/** - STL files for visualization in Gazebo and Rviz.
- **scenes/** - includes scene objects for simulation.
- **launch/** - Launch files for Gazebo and Rviz.

### **moveit_config_diablo**
- **config/** - Includes SRDF, URDF, KDL kinematics solver, and helper methods for MoveIt2.
- **launch/** - Launch files for MoveIt2 motion planning.

## Installation & Setup
1. Install **ROS2 Humble** following the official guide.
2. Install **Gazebo Classic** (latest supported version for ROS Humble).
3. Install **MoveIt2** using the provided installation guide.
4. Clone this repository and build the workspace using colcon:
   ```
   git clone git@github.com:Mystique03/diablo_ws.git
   cd diablo_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

## Usage

### **1. Visualize Robot in Rviz**
```
# Terminal 1
ros2 launch diablo_bot rsp.launch.py

# Terminal 2
rviz2

# Terminal 3
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### **2. Visualize Robot in Gazebo**
```
ros2 launch diablo_bot gz.launch.py
```

### **3. Launch Robot with ROS Control**
1. Navigate to `diablo_bot/description/ros2_control.xacro` and uncomment:
   ```
   <plugin>gazebo_ros2_control/GazeboSystem</plugin>
   ```
2. Launch the robot with ROS Control:
   ```
   ros2 launch diablo_bot diablo.launch.py
   ```

### **4. Launch Robot with MoveIt2**
1. Source MoveIt2 environment:
   ```
   source <your_moveit2_package>/install/setup.bash
   ```
2. Navigate to `diablo_bot/description/ros2_control.xacro` and uncomment:
   ```
   <plugin>mock_components/GenericSystem</plugin>
   ```
3. Launch MoveIt2:
   ```
   ros2 launch moveit_config_diablo demo.launch.py
   ```

### **5. Launch Table & Mug Scene**
1. Launch the robot using the MoveIt2 commands above.
2. In **Rviz**, Under **Scene Objects** import the scene:
   ```
   diablo_bot/scenes/coffee_mug.scene
   ```

### **6. Launch Joint State Publisher GUI with Gazebo**
```
# Terminal 1
ros2 run diablo_bot diablo.launch.py

# Terminal 2
ros2 run diablo_bot js_to_trajectory.py

# Terminal 3
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### **7. Run Inverse Kinematics**
```
# Terminal 1
ros2 run diablo_bot diablo.launch.py

# Terminal 2
ros2 run diablo_bot ikSolver.py

# Terminal 3
ros2 topic pub /desired_end_effector_position geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 0.0}"
```

### **8. Run Forward Kinematics**
```
# Terminal 1
ros2 run diablo_bot diablo.launch.py

# Terminal 2
ros2 run diablo_bot dh_fkSolver.py
```
### **Modifying MoveIt Setup**
1. Source the MoveIt2 environment:
   ```
   source moveit/install/setup.bash
   ```
2. Run MoveIt Setup Assistant:
   ```
   ros2 run moveit_setup_assistant moveit_setup_assistant
   ```

### **Contributors**
- **Mystique03** (GitHub: [Mystique03](https://github.com/Mystique03))



