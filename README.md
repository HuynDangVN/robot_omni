1. truy cap thu muc ros2
2. mo terminal 1 va nhap:
colcon build
source install/setup.bash
ros2 launch robot_omni gazebo_control.launch.py
3. cho cho gazebo chay hoan toan (khoang 30s)
4. mo terminal 2 va nhap:
source install/setup.bash
ros2 launch robot_omni nav2_control.launch.py
