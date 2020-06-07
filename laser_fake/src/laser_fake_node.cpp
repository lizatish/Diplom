#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unior_laser_fake_node");
  ros::NodeHandle nh;
  ros::Publisher ls_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);

  ros::Rate r(15);
  while(ros::ok()) {

    sensor_msgs::LaserScan ls;
    ls.header.frame_id = "laser";
    ls.header.stamp = ros::Time::now();
    ls.angle_min = -M_PI/2;
    ls.angle_max = M_PI/2;
    ls.angle_increment = M_PI/180;
    for(float angle = ls.angle_min; angle <= ls.angle_max; angle += ls.angle_increment ) {
      ls.ranges.push_back(10);
      ls.intensities.push_back(10);
    }
    ls.range_max = 11;
    ls.range_min = 9;
    ls.scan_time = 1/15;
    ls.time_increment = 1/15/180;

    ls_pub.publish(ls);
    ROS_INFO("Publish");
        r.sleep();
    ros::spinOnce();
  }
}
