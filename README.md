# Co-Robotic-Ultrasound-Mammography

## File List
**ur5_ros_control_moveit_rviz.launch**

- Launches moveit and rviz with physical robot loaded in
- Run with: $roslaunch ur_modern_driver ur5_ros_control_moveit_rviz.launch robot_ip:=172.22.22.2
- Place in: src/modern_driver/launch

**motion_planning**

- Motion planning package that allows for motion planning with physical robot.
- Place in: src/
- basic_movement.launch moves the robot to a specified x-y-z-theta coordinate in basic_movement.cpp
- - you need to rebuild the workspace when making changes to basic_movement.cpp

**Progress.odt**

- Progress notes and troubleshooting steps
