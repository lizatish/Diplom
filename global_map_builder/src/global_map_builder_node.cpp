///*           Описание модуля
// Модуль предназначен для получения и обработки
// локальной карты и одометрии, и
// соединения локальной и глобальной карт
///*

#include <ros/ros.h>
#include<geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Polygon.h>
#include <vector>
#include <math.h>
#include <iostream>
using namespace std;

void odometryCallback(const nav_msgs::Odometry data);
void connectLocalAndGlobalMaps();
void localMapCallback(nav_msgs::OccupancyGrid data);
void globalMapMessageInitParams();
void formGlobalMapMessage();

// Текущая локальная карта
static nav_msgs::OccupancyGrid localMap;
// Текущая открытая глобальная карта
static nav_msgs::OccupancyGrid globalMap;

// Размер карты
static double mapResolution = 0.04;
static int globalMapSize = 20 / mapResolution;

//Текущее положение платформы
static geometry_msgs::Pose currentPosition;

static nav_msgs::GridCells obstacles;


static bool isCameOdom = false;
static bool isCameLocalMap = false;

int main(int argc, char **argv){
  ros::init(argc, argv, "global_map_node");

  ros::NodeHandle m;
  ros::Subscriber local_map_sub = m.subscribe("/local_map", 8, localMapCallback);
  ros::Subscriber odom_sub = m.subscribe("/odom", 8, odometryCallback);
  ros::Publisher global_map_pub = m.advertise<nav_msgs::OccupancyGrid>("/global_map", 2);
  ros::Publisher obstacles_pub = m.advertise<nav_msgs::GridCells>("/obstacles", 2);

  bool isAllowProcess = true;
  ros::Rate rate(100);
  while(isAllowProcess && ros::ok()) {

    if(isCameOdom && isCameLocalMap){
      connectLocalAndGlobalMaps();
      global_map_pub.publish(globalMap);
      obstacles_pub.publish(obstacles);
      cout << "Global map is coming" << endl;
    }

    rate.sleep();
    ros::spinOnce();
  }
}
void odometryCallback(const nav_msgs::Odometry data){
  // Получение одометрии
  double yawAngle = tf::getYaw(data.pose.pose.orientation);
  currentPosition = data.pose.pose;

  // Координата смещения лазера относительно центра платформы
  double laserOffsetX = 0.24;
  double laserOffsetY = 0;
  // Составляющая поворота
  double laserRotationX = laserOffsetX * cos(yawAngle) - laserOffsetY * sin(yawAngle);
  double laserRotationY = laserOffsetX * sin(yawAngle) + laserOffsetY * cos(yawAngle);
  // Текущие координаты с учетом местоположения лазерного скана
  currentPosition.position.x += laserRotationX;
  currentPosition.position.y += laserRotationY;

  //    cout << currentPosition.x << " " << currentPosition.y << " " << yawAngle << endl;
  isCameOdom = true;
}
void localMapCallback( nav_msgs::OccupancyGrid data){
  localMap = data;

  if (!isCameLocalMap){
    obstacles.cell_width = mapResolution;
    obstacles.cell_height = mapResolution;
    obstacles.header.frame_id = "/odom";
    obstacles.header.stamp = ros::Time::now();

    globalMap.info.height = globalMapSize;
    globalMap.info.width = globalMapSize;
    globalMap.info.resolution = data.info.resolution;
    globalMap.info.origin.position.x = -globalMapSize * data.info.resolution/2;
    globalMap.info.origin.position.y = -globalMapSize * data.info.resolution/2;
    globalMap.header.frame_id = "/map";
    globalMap.header.stamp = ros::Time::now();
    globalMap.data.resize(globalMap.info.height * globalMap.info.width);
    for(int i = 0; i < globalMapSize; i++){
      for(int j = 0; j < globalMapSize; j++){
        globalMap.data[globalMapSize * i + j] = 50;
      }
    }
  }
  isCameLocalMap = true;
}

void connectLocalAndGlobalMaps(){
  double localMapSize = localMap.info.width;
  double yawAngle = tf::getYaw(currentPosition.orientation);

  obstacles.cells.clear();
  float k = 0.9;
  for(int i = 0; i < localMapSize; i++){
    for(int j = 0; j < localMapSize; j++){
      if(i < globalMapSize && j < globalMapSize){

        int rotationX = (i - localMapSize/2) * cos(yawAngle) - (j - localMapSize/2) * sin(yawAngle);
        int rotationY = (i - localMapSize/2) * sin(yawAngle) + (j - localMapSize/2) * cos(yawAngle);

        int x = globalMapSize / 2 + currentPosition.position.x/mapResolution + (rotationX);
        int y = globalMapSize / 2 + currentPosition.position.y/mapResolution + (rotationY);

        int value = globalMap.data[globalMapSize * y + x] * k
            + localMap.data[localMapSize * j + i] * (1 - k);

        if(localMap.data[localMapSize * j + i] == 50){
          continue;
        }
        if (value >= 60){
          geometry_msgs::Point p;
          p.x =( x  - globalMapSize / 2)* mapResolution;
          p.y = (y - globalMapSize / 2)* mapResolution ;
          obstacles.cells.push_back(p);
        }
        globalMap.data[globalMapSize * y + x] = value;
      }
    }
  }
}
