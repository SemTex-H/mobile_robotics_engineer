# Module 5 Assignment: Energy-Efficient Navigation and Robotic Arm Control (ROS 2)


## Task 2: Optimize Goal Selection and Path Planning

### Description

A ROS 2 node was developed to select a navigation goal for TurtleBot3 based on **minimum energy consumption**. The shortest path (in Euclidean distance) is calculated, and a trajectory is planned using a spline-based smoother. The robot then navigates to the selected goal using a **custom energy-aware motion planner**.

### Key Features

- Selects optimal goal by evaluating distance (as a proxy for energy).
- Publishes to `/cmd_vel` to move the robot.
- Logs distance and estimated energy for comparison.

### Files Created

- `goal_linear.cpp`: ROS 2 node to compute goal.
---

## Task 3: Add a Position Controller to Robotic Arm URDF

### Description

The robotic arm URDF was extended with a `ros2_control` configuration and a `JointTrajectoryController`. The arm can now receive multi-joint position commands and simulate motion accurately in a physics simulator.

### Key Features

- Added `<transmission>` and `<ros2_control>` blocks to URDF.
- Configured `JointTrajectoryController` in YAML.
- Implemented a ROS 2 node to publish joint trajectories periodically.
- Successfully simulated joint movement in Gazebo/RViz.

### Files Created

- `robot_arm.urdf`: Updated URDF with joints, limits, and control config.
- `position_controllers.yaml`: Controller configuration file.
- `arm_commander.cpp`: Publishes `JointTrajectory` messages to move arm.
- `position_controllers.launch.py`: Launches controllers using `controller_manager`.

---

## How to Run

### TurtleBot3 Path Planner


```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```bash
colcon build
source install/setup.bash
ros2 run module_5_assignment goal_linear
```
