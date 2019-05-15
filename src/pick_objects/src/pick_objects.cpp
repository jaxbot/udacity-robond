#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "add_markers/AddMarkers.h"
#include "add_markers/ClearMarker.h"

float PICKUP_X = 8;
float PICKUP_Y = 2.68;
float DROPOFF_X = 1.4;
float DROPOFF_Y = -7;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::ServiceClient addMarkerClient;
ros::ServiceClient clearMarkerClient;

bool move_to_x_y(float x, float y) {
  MoveBaseClient ac("move_base", true);
  // Wait for the MoveBaseClient to init.
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Still waiting for the MoveBaseClient to ready.");
  }

  move_base_msgs::MoveBaseGoal goal;

  // Pin to the map frame_id
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Set the goal to the provided x, y values.
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal for robot to reach.");
  // Send the goal request and wait for it to complete (or fail).
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Moved to goal successfully.");
    return true;
  } else {
    ROS_INFO("Failed to move to goal.");
    return false;
  }
}

void show_marker(float x, float y) {
  add_markers::AddMarkers srv;
  srv.request.x = x;
  srv.request.y = y;
  addMarkerClient.call(srv);
}

void clear_marker() {
  add_markers::ClearMarker clearSrv;
  clearMarkerClient.call(clearSrv);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  // Create clients for addMarker/clearMarker to display the goal nodes in rviz.
  addMarkerClient = n.serviceClient<add_markers::AddMarkers>("/add_marker/add_marker");
  clearMarkerClient = n.serviceClient<add_markers::ClearMarker>("/add_marker/clear_marker");

  // Show the pickup zone marker.
  show_marker(PICKUP_X, PICKUP_Y);
  // Move to the pickup zone.
  move_to_x_y(PICKUP_X, PICKUP_Y);
  // Clear the marker now that we've reached the pickup zone.
  clear_marker();
  // Wait 5 seconds to simulate the pickup.
  ros::Duration(5.0).sleep();
  // Move to the dropoff zone.
  move_to_x_y(DROPOFF_X, DROPOFF_Y);
  // Show the marker now that we've reached the dropoff zone.
  show_marker(DROPOFF_X, DROPOFF_Y);

  return 0;
}
