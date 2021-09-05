#!/bin/sh
xterm -e " source ~/catkin_ws/devel/setup.bash; export ROBOT_INITIAL_POSE=\"-x 1.0 -y -1.0 -z 0.12\"; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/catkin_ws/src/map/MyHouse.world " &
sleep 5
xterm -e " source ~/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5
xterm -e " source ~/catkin_ws/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " source ~/catkin_ws/devel/setup.bash; rosrun add_markers add_markers "