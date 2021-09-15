## robotics nanodegree home service project.
Final Project for robotics nano degree

Create a home service robot that picks a package and delivers it.

#### Packages used:

To complete this project I used the following packages:

* `turtlebot_gazebo`: I used this package to work with turtlebot in my world.
* `turtlebot_rviz_launchers`: This package is used to show the robot and navigation data in rviz.
* `actionlib`, `move_base_msgs`: These packages are used to move the robot toward the pickup and dropoff locations.
* `visualization_msgs`: This package is used to create virtual objects in rviz simulation.
* `nav_msgs`: This package is used to subscribe to odometry

I created this packages too:
* `add_markers`: This package is used to create markers in rviz simulation. I used this package to create the markers for the packages. I created two nodes here: one for the exercise to show and hide virtual objects(`add_virtual_objects`) and another for the home service robot(`add_markers_node`).
I am using the following packages here:
    - `visualization_msgs`: I used this package to create the visual markers for the packages.
    - `nav_msgs`: I used this package to track the robot position.

* `pick_objects`: This package is used to send the robot to pick and drop the package. In this package I used `move_base_msgs` and `actionlib` to command the robot to go to the pickup and drop off locations.