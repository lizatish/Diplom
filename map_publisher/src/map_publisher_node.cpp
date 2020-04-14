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
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <math.h>
#include <iostream>
using namespace std;

void laserScanCallback(const sensor_msgs::LaserScan data);
void odometryCallback(const nav_msgs::Odometry data);
void setMap(vector<float> theta, vector<float> r);
void createLocalMap();
void connectLocalAndGlobalMaps();
void updateObstacles();
void init_params();

// Размеры и разрешение карт
static double mapResolution = 0.04;
static int localMapSize = int(12 / mapResolution);
static int globalMapSize = 20 / mapResolution;

// Текущий скан
static sensor_msgs::LaserScan current_scan;
// Текущая локальная карта
static nav_msgs::OccupancyGrid localMap;
// Текущая открытая глобальная карта
static nav_msgs::OccupancyGrid globalMap;
// Текущее положение платформы
static geometry_msgs::Pose currentPosition;
// Текущие препятствия
static nav_msgs::GridCells obstacles;

static bool isActualScanData = false;
static bool isCameOdom = false;

int main(int argc, char **argv){
  ros::init(argc, argv, "global_map_node");

  ros::NodeHandle m;
  ros::Subscriber odom_sub = m.subscribe("/odom", 8, odometryCallback);
  ros::Subscriber scan_sub = m.subscribe("/scan", 8, laserScanCallback);
  ros::Publisher global_map_pub = m.advertise<nav_msgs::OccupancyGrid>("/global_map", 2);
  ros::Publisher obstacles_pub = m.advertise<nav_msgs::GridCells>("/obstacles", 2);
  ros::Publisher local_map_pub = m.advertise<nav_msgs::OccupancyGrid>("/local_map", 2);

  init_params();
  bool isAllowProcess = true;
  ros::Rate rate(100);
  while(isAllowProcess && ros::ok()) {

    if(isCameOdom && isActualScanData){

      // Построние карты со сканера
      createLocalMap();
      connectLocalAndGlobalMaps();


      global_map_pub.publish(globalMap);
      local_map_pub.publish(localMap);
      obstacles_pub.publish(obstacles);
      ROS_INFO("Publish global map");

//      isCameOdom = false;
//      isActualScanData = false;

    }

    rate.sleep();
    ros::spinOnce();
  }
}

// Получение данных с лазерного сканера
void laserScanCallback(const sensor_msgs::LaserScan data) {
  current_scan = data;
  isActualScanData = true;
}

void odometryCallback(const nav_msgs::Odometry data){
  // Получение одометрии
  double yawAngle = tf::getYaw(data.pose.pose.orientation);
  currentPosition = data.pose.pose;

  // Координата смещения лазера относительно центра платформы
  double laserOffsetX = 0.25;
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

void connectLocalAndGlobalMaps(){
  double localMapSize = localMap.info.width;
  double yawAngle = tf::getYaw(currentPosition.orientation);

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

        globalMap.data[globalMapSize * y + x] = value;
      }
    }
  }
  updateObstacles();
}
void updateObstacles(){
  obstacles.cells.clear();
  for(int i = 0; i < globalMapSize; i++){
    for(int j = 0; j < globalMapSize; j++){
      if (globalMap.data[globalMapSize * j + i] >= 60){
        geometry_msgs::Point p;
        p.x = (i  - globalMapSize / 2) * mapResolution;
        p.y = (j - globalMapSize / 2) * mapResolution ;
        obstacles.cells.push_back(p);
      }
    }
  }
}

// Сортировка данных с лазерного сканера
void createLocalMap(){
  if(isActualScanData) {
    vector<float> angles;
    vector<float> ranges;
    for(int id = 0; id < current_scan.ranges.size(); id++) {
      float theta = current_scan.angle_min + id * current_scan.angle_increment;
      float range = current_scan.ranges.at(id);
      angles.push_back(theta);
      ranges.push_back(range);
    }
    // Задать карту по данным со сканера
    setMap(angles,ranges);
  }
}

// Формирование карты с лазерного сканера
void setMap(vector<float> theta, vector<float> r){
  for(int i = 0; i < localMapSize; i++)
    for(int j = 0; j < localMapSize; j++){
      localMap.data[localMapSize * j + i] = 50;
    }

  for(uint m = 0; m < r.size(); m++){
    if(r[m] < 6 && r[m] > 0.5){
      // Преобразование координат новой точки из ПСК в ДСК
      int x2 = int(r[m] * cos(theta[m])/mapResolution + localMapSize/2);
      int y2 = int(r[m] * sin(theta[m])/mapResolution + localMapSize/2);

      if(x2 >= 0 && x2 < localMapSize && y2 >= 0 && y2 < localMapSize) {
        for(float dist = 0; dist < r[m]; dist += mapResolution) {
          int x3 = dist * cos(theta[m])/mapResolution + localMapSize/2;
          int y3 = dist * sin(theta[m])/mapResolution + localMapSize/2;
          localMap.data[localMapSize * y3 + x3] = 0;
        }
        localMap.data[localMapSize * y2 + x2] = 100;
        localMap.data[localMapSize * (y2 + 1) + (x2 + 1)] = 100;
        localMap.data[localMapSize * (y2 + 1) + x2] = 100;
        localMap.data[localMapSize * (y2 + 1) + (x2 - 1)] = 100;
        localMap.data[localMapSize * (y2 - 1) + (x2 + 1)] = 100;
        localMap.data[localMapSize * (y2 - 1) + (x2 - 1)] = 100;
        localMap.data[localMapSize * (y2 - 1) + x2] = 100;
        localMap.data[localMapSize * y2 + (x2 - 1)] = 100;
        localMap.data[localMapSize * y2 + (x2 + 1)] = 100;
      }
    }
  }
}
void init_params(){
  obstacles.cell_width = mapResolution;
  obstacles.cell_height = mapResolution;
  obstacles.header.frame_id = "/odom";
  obstacles.header.stamp = ros::Time::now();

  globalMap.info.height = globalMapSize;
  globalMap.info.width = globalMapSize;
  globalMap.info.resolution = mapResolution;
  globalMap.info.origin.position.x = -globalMapSize * mapResolution/2;
  globalMap.info.origin.position.y = -globalMapSize * mapResolution/2;
  globalMap.header.frame_id = "/map";
  globalMap.header.stamp = ros::Time::now();
  globalMap.data.resize(globalMap.info.height * globalMap.info.width);
  for(int i = 0; i < globalMapSize; i++){
    for(int j = 0; j < globalMapSize; j++){
      globalMap.data[globalMapSize * i + j] = 50;
    }
  }

  localMap.info.height = localMapSize;
  localMap.info.width = localMapSize;
  localMap.info.resolution = mapResolution;
  localMap.info.origin.position.x = -localMapSize * mapResolution/2;
  localMap.info.origin.position.y = -localMapSize * mapResolution/2;
  localMap.header.frame_id = "/odom";
  localMap.header.stamp = ros::Time::now();
  localMap.data.resize(localMap.info.height * localMap.info.width);
}
