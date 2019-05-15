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
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");

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

bool jitter() {
  MoveBaseClient ac("move_base", true);
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");

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
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default

  addMarkerClient = n.serviceClient<add_markers::AddMarkers>("/add_marker/add_marker");
  clearMarkerClient = n.serviceClient<add_markers::ClearMarker>("/add_marker/clear_marker");

  show_marker(PICKUP_X, PICKUP_Y);
  move_to_x_y(PICKUP_X, PICKUP_Y);
  clear_marker();
  ros::Duration(5.0).sleep();
  move_to_x_y(DROPOFF_X, DROPOFF_Y);
  show_marker(DROPOFF_X, DROPOFF_Y);

  return 0;
}
