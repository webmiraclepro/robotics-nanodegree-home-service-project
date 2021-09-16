## robotics nanodegree home service project.
Final Project for robotics nano degree

Create a home service robot that picks a package and delivers it.

#### Packages used:

To complete this project I used the following packages:

* `turtlebot_gazebo`: This package is provided by `turtlebot_simulator`, This package is used to load the turtlebot and load the world provided as parameter.
* `turtlebot_rviz_launchers`: This package is got it from `turtlebot_interactions`, this package allows the user to interact with the turtlebot. The `turtlebot_rviz_lauchers` package is used to launch the rviz gui.

I created these packages too:
* `add_markers`: This package is used to create markers in rviz simulation. I used this package to create the markers for the packages. I created two nodes here: one for the exercise to show and hide virtual objects(`add_virtual_objects`) and another for the home service robot(`add_markers_node`).
I am using the following packages here:
    - `visualization_msgs`: This package provides us with a set of messages that allows to deal with visualization data. I used this package to create the visual markers for the packages.
    - `nav_msgs`: this package define a set of messages used to interact with the navigation stack, I used this package to track the robot position.

* `pick_objects`: This package is used to send the robot to pick and drop the package. In this package I used `move_base_msgs` and `actionlib` to command the robot to go to the pickup and drop off location.
    - `move_base_msgs`: this package contains the messages used to communicate with the `move_base` node.
    - `actionlib`: this package provides a standard interface to perform predefined actions.