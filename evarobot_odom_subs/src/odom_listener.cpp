#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

/**
 * This tutorial demonstrates simple receipt of position and of the Evarobot over the ROS system.
 */
 
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position: [%f, %f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation: [%f, %f, %f, %f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Velocity: [%f, %f, %f]", msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/odom", 1000, chatterCallback);

    ros::spin();
}