#!/bin/sh
xterm -e " source devel/setup.bash; export ROBOT_INITIAL_POSE=\"-x 1.0 -y -1.0 -z 0.12\"; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/map/MyHouse.world " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch "