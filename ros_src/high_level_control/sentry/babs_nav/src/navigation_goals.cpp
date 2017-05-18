#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void pointClickCallback(const geometry_msgs::PointStamped& pointStamped) {

  ROS_INFO("Recieved point-click goal");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";

  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = pointStamped.point.x;
  goal.target_pose.pose.position.y = pointStamped.point.y;


  goal.target_pose.pose.orientation.w = 1.0;
  ROS_INFO("Sending point-click goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("GOAL REACHED");
  else
    ROS_WARN("GOAL FAILED!!!");
}

int main(int argc, char** argv){

  ROS_INFO("first!");

  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle n;
  ros::Subscriber point_click_subscriber = n.subscribe("clicked_point", 1, pointClickCallback);

  ros::spin();
  return 0;
}
