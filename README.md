# tiago_robot
tiago_robot_ros2

The entire project is based on ros2 humble

reference: https://github.com/pal-robotics/tiago_simulation

pull the entire repository into the local file named tiago_public_ws, and colcon build.

Don't forget to source install/setup.bash

To start the simulation, ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=pick [arm:=no-arm]

To start the base_movement, ros2 run key_teleop key_teleop, and use arrors to control the robot

To start control of the joints, python3 arm_teleop.py: 1-7 represents 7 joints, 8 is the torso 9 is the gripper. The entire code is based on action server.
