#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odometryCallback(const nav_msgs::Odometry &data);
void tf_base_link_to_laser();
void tf_base_footprint_to_base_link();
void tf_odom_to_base_link();
void tf_map_to_odom();

static bool isCameOdom = false;
// map - odom
static geometry_msgs::TransformStamped ts_map2odom;
// odom - base_link
static geometry_msgs::TransformStamped ts_odom2base_link;
// base_link - laser
static geometry_msgs::TransformStamped ts_base_link2laser;
// base_link - base_footprint
static geometry_msgs::TransformStamped ts_base_footprint2base_link;

static ros::Time current_time;
static nav_msgs::Odometry cur_odom;

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");

  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe("/odom", 8, odometryCallback);
  tf::TransformBroadcaster broadcaster_odom2base_link;
  tf::TransformBroadcaster broadcaster_base_link2laser;
  tf::TransformBroadcaster broadcaster_base_link2base_footprint;
  tf::TransformBroadcaster broadcaster_map2odom;

  current_time = ros::Time::now();
  ros::Rate rate(50);
  while(ros::ok()){

    //send the transform
    if(isCameOdom){
      current_time = ros::Time::now();
      //                        tf_odom_to_base_link();
      tf_base_footprint_to_base_link();
      tf_base_link_to_laser();
      tf_map_to_odom();

      //                        broadcaster_odom2base_link.sendTransform(ts_odom2base_link);
      broadcaster_map2odom.sendTransform(ts_map2odom);
      broadcaster_base_link2base_footprint.sendTransform(ts_base_footprint2base_link);
      broadcaster_base_link2laser.sendTransform(ts_base_link2laser);
      ROS_INFO("Publish");
    }
    rate.sleep();
    ros::spinOnce();
  }
}
void tf_map_to_odom(){
  ts_map2odom.header.stamp = current_time;
  ts_map2odom.header.frame_id = "map";
  ts_map2odom.child_frame_id = "odom";

  ts_map2odom.transform.translation.x = 0;
  ts_map2odom.transform.translation.y = 0;
  ts_map2odom.transform.translation.z = 0.0;
  ts_map2odom.transform.rotation.x = 0.0;
  ts_map2odom.transform.rotation.y = 0.0;
  ts_map2odom.transform.rotation.z = 0.0;
  ts_map2odom.transform.rotation.w = 1.0;
}
void odometryCallback(const nav_msgs::Odometry &data){
  cur_odom = data;
  isCameOdom = true;
}
void tf_odom_to_base_link(){
  ts_odom2base_link.header.stamp = current_time;
  ts_odom2base_link.header.frame_id = "odom";
  ts_odom2base_link.child_frame_id = "base_link";

  ts_odom2base_link.transform.translation.x = cur_odom.pose.pose.position.x;
  ts_odom2base_link.transform.translation.y = cur_odom.pose.pose.position.y;
  ts_odom2base_link.transform.translation.z = 0.0;
  ts_odom2base_link.transform.rotation = cur_odom.pose.pose.orientation;
}

void tf_base_footprint_to_base_link(){
  ts_base_footprint2base_link.header.stamp = current_time;
  ts_base_footprint2base_link.header.frame_id = "base_footprint";
  ts_base_footprint2base_link.child_frame_id = "base_link";

  ts_base_footprint2base_link.transform.translation.x = 0.0;
  ts_base_footprint2base_link.transform.translation.y = 0.0;
  ts_base_footprint2base_link.transform.translation.z = 0.0;
  ts_base_footprint2base_link.transform.rotation.x = 0.0;
  ts_base_footprint2base_link.transform.rotation.y = 0.0;
  ts_base_footprint2base_link.transform.rotation.z = 0.0;
  ts_base_footprint2base_link.transform.rotation.w = 1.0;

}

void tf_base_link_to_laser(){
  ts_base_link2laser.header.stamp = current_time;
  ts_base_link2laser.header.frame_id = "base_link";
  ts_base_link2laser.child_frame_id = "laser";

  ts_base_link2laser.transform.translation.x = 0.25;
  ts_base_link2laser.transform.translation.y = 0.0;
  ts_base_link2laser.transform.translation.z = 0.0;
  geometry_msgs::Quaternion tf_quat = tf::createQuaternionMsgFromYaw(0);
  ts_base_link2laser.transform.rotation = tf_quat;
}
