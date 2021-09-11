#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

// define a struct to store points with x and y coordinates
struct Point {
    float x;
    float y;
};

enum HomeServiceStatus {
    SETTING_CARGO,
    LOOKING_FOR_CARGO,
    LOOKING_DESTINATION,
    DROPOFF_CARGO,
    DONE
};


class MarkerManager {
private:
    static const float delta = 0.5;
    HomeServiceStatus status;
    Point pickupPoint;
    Point dropoffPoint;
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    ros::Subscriber odom_sub;
    
    void addMarker(Point position);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

public:
    MarkerManager();
    ~MarkerManager();
    void addPickupMarker();
    void addDropoffMarker();
    void removeMarker();
    int getNumSubscribers();
    void subscribe();
};

MarkerManager::MarkerManager(){
    status = SETTING_CARGO;
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    std::string node_name = ros::this_node::getName();
    float x, y;

    nh.getParam(node_name + "/pickup_point_x", pickupPoint.x);
    nh.getParam(node_name + "/pickup_point_y", pickupPoint.y);
    nh.getParam(node_name + "/dropoff_point_x", dropoffPoint.x);
    nh.getParam(node_name + "/dropoff_point_y", dropoffPoint.y);

    ROS_INFO("Pickup point: (%f, %f)", pickupPoint.x, pickupPoint.y);
    ROS_INFO("Dropoff point: (%f, %f)", dropoffPoint.x, dropoffPoint.y);

    ros::Rate rate(1);

    subscribe();
}

MarkerManager::~MarkerManager(){
}

void MarkerManager::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // check velocity equals 0
    bool is_stop = msg->twist.twist.linear.x == 0 && msg->twist.twist.linear.y == 0;
    ROS_WARN_ONCE("Odometry callback called!!");
    if(is_stop){
        ROS_INFO("[%s] is stopped", ros::this_node::getName().c_str());
    }
    
    if (is_stop && status == LOOKING_FOR_CARGO){
        // is pickup point with euclidean distance less than delta
        float distance = sqrt(pow(pickupPoint.x - msg->pose.pose.position.x, 2) + pow(pickupPoint.y - msg->pose.pose.position.y, 2));
        if (distance < delta){
            status = LOOKING_DESTINATION;
            removeMarker();
        }
    }
    else if (is_stop && status == LOOKING_DESTINATION){
        // is dropoff point with euclidean distance less than delta
        float distance = sqrt(pow(dropoffPoint.x - msg->pose.pose.position.x, 2) + pow(dropoffPoint.y - msg->pose.pose.position.y, 2));
        if (distance < delta){
            status = DROPOFF_CARGO;
            addDropoffMarker();
        }
    }
}

void MarkerManager::addMarker(Point position){
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
    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
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

void MarkerManager::removeMarker(){
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
    ROS_INFO("Retrieving Cargo!");
}

int MarkerManager::getNumSubscribers(){
    return marker_pub.getNumSubscribers();
}

void MarkerManager::addPickupMarker(){
    if (status == SETTING_CARGO){
        addMarker(pickupPoint);
        ROS_INFO("Pickup marker added");
        status = LOOKING_FOR_CARGO;
    }
}

void MarkerManager::addDropoffMarker(){
    if (status == DROPOFF_CARGO){
        addMarker(dropoffPoint);
        status = DONE;
    }
}

void MarkerManager::subscribe(){
    ros::Subscriber sub = nh.subscribe("odom", 1000, &MarkerManager::odometryCallback, this);
    ROS_INFO("Waiting for odometry data...");
    while (ros::ok()){
        ros::spinOnce();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_markers");
    MarkerManager marker_manager;
    ros::Rate rate(1);

    while (ros::ok()) {
        // Publish the marker
        while (marker_manager.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        break;
    }

    marker_manager.addPickupMarker();
    ros::spinOnce();
    
    marker_manager.subscribe();

    return 0;
}