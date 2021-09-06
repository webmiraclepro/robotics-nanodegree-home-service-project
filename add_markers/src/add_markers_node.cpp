#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

void add_marker(ros::Publisher &marker_pub, float x, float y);
void remove_marker(ros::Publisher &marker_pub);

class MarkerManager {
private:
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    ros::Subscriber odom_sub;
public:
    MarkerManager(/* args */);
    ~MarkerManager();
    void add_marker(float x, float y);
    void remove_marker();
    int getNumSubscribers();
};

MarkerManager::MarkerManager(){
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Rate rate(1);
}

MarkerManager::~MarkerManager(){
}

void MarkerManager::add_marker(float x, float y){
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;


    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);
}

void MarkerManager::remove_marker(){
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::DELETE;

    marker_pub.publish(marker);
}

int MarkerManager::getNumSubscribers(){
    return marker_pub.getNumSubscribers();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "add_markers");
    MarkerManager marker_manager;

    while (ros::ok()) {
        // Publish the marker
        while (marker_manager.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        
        marker_manager.add_marker(-2.5, 1.5);

        ros::Duration(5.0).sleep();

        marker_manager.remove_marker();

        ros::Duration(5.0).sleep();

        marker_manager.add_marker(3.0, 2.5);

        ros::Duration(5.0).sleep();

        marker_manager.remove_marker();

        ros::Duration(5.0).sleep();

        return 0;
    }   
}