#!/bin/bash

ros2 launch turtlebot3_bringup robot.launch.py &
sleep 5

# 2. Start SLAM Mapping
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true &
sleep 5

# 3. Start Nav2 navigation
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True &
sleep 5

# 4. Start ros2 bridge (websocket)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
sleep 2

python3 burger_camera_node.py &
sleep 2
python3 pages/burger_detection_node.py&

wait