#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/AddMarkers.h"
#include "add_markers/ClearMarker.h"
#include <string>

float PICKUP_X = -6.271;
float PICKUP_Y = 1.835;
float DROPOFF_X = -0.42;
float DROPOFF_Y = -1.75;

auto MARKER_NAMESPACE = "service_robot_object";
ros::Publisher marker_pub;

visualization_msgs::Marker make_marker() {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.ns = MARKER_NAMESPACE;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    return marker;
}

void show_marker_on_map(float x, float y) {
    visualization_msgs::Marker marker = make_marker();
    marker.header.frame_id = "map";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;

    marker_pub.publish(marker);
}

void hide_marker_on_map() {
    visualization_msgs::Marker marker = make_marker();
    marker.header.frame_id = "map";
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
}

bool handle_add_marker_request(add_markers::AddMarkers::Request& req, add_markers::AddMarkers::Response& res)
{
    show_marker_on_map(req.x, req.y);
    res.msg_feedback = "Marker set";
    ROS_INFO_STREAM(res.msg_feedback);
    
    return true;
}

bool handle_clear_marker_request(add_markers::ClearMarker::Request& req, add_markers::ClearMarker::Response& res)
{
    hide_marker_on_map();
    res.msg_feedback = "Marker cleared";
    ROS_INFO_STREAM(res.msg_feedback);
    
    return true;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::ServiceServer addService = n.advertiseService("/add_marker/add_marker", handle_add_marker_request); 
  ros::ServiceServer clearService = n.advertiseService("/add_marker/clear_marker", handle_clear_marker_request); 
  ros::spin();
}
