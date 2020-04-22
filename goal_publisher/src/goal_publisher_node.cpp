#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

geometry_msgs::PoseStamped goal;
bool is_goal_came = false;

void goalCallback(const geometry_msgs::PoseStamped& data){
  if (data.pose != goal.pose){
    goal = data;
    ROS_INFO("New data %f %f", goal.pose.position.x, goal.pose.position.y);
    is_goal_came = false;

  }
  is_goal_came = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_publisher_node");
  ros::NodeHandle nh;

  ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 8, goalCallback);
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    if(is_goal_came){
      goal_pub.publish(goal);
      ROS_INFO("Publish goal %f %f", goal.pose.position.x, goal.pose.position.y);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
