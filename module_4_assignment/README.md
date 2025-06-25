Created files and named them according to each task.

After clones repo need to build package
  ```sh
  cd assignment
  colcon build --symlink-install --select-packages mobile_robotics_engineer
  ```

**Created box maze named box_maze.world**
  to see file
  ```sh
  cd assignment_ws/src/mobile_robotics_engineer/robot_sensing/world
  ls
  ```

**For running the turtlebot**
  ```sh
  source turtlebot3_ws/install/setup.bash && source assignment_ws/install/setup.bash
  ros2 launch robot_sensing lidar_box_maze_solving.launch.py
  ```

