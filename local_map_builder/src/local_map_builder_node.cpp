///*           Описание модуля
// Модуль предназначен для обработки информации, поступаюшей
// с лазерного сакнера,
// и построения по этим данным локальной карты проходимости
///*

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Polygon.h>
#include <vector>
#include <math.h>
#include <iostream>
using namespace std;

void laserScanCallback(const sensor_msgs::LaserScan data);
void setMap(vector<float> theta, vector<float> r);
void buildMapFromLaserScaner();
void localMapMessageInitParams();
void initLocalMap();

// Сообщение с картой
static nav_msgs::OccupancyGrid localMap;
// Текущий скан
static sensor_msgs::LaserScan current_scan;
// Пришел ли скан
static bool isActualScanData = false;

// Размер карты
// Разрешение карты
static double mapResolution = 0.04;
static int mapSize = int(12 / mapResolution);

int main(int argc, char **argv){
    ros::init(argc, argv, "kuka_local_map_node");

    ros::NodeHandle k;
    ros::Subscriber ls_sub = k.subscribe("/scan", 8, laserScanCallback);
    ros::Publisher map_pub = k.advertise<nav_msgs::OccupancyGrid>("/local_map", 2);

    ros::Rate rate = 100;
    bool isAllowProcess = true;
    while(isAllowProcess && ros::ok()) {
        localMapMessageInitParams();
        // Построние карты со сканера
        buildMapFromLaserScaner();

        map_pub.publish(localMap);
        ros::spinOnce();
        rate.sleep();
        cout << "Local map is coming" << endl;
    }
}

// Получение данных с лазерного сканера
void laserScanCallback(const sensor_msgs::LaserScan data) {
    current_scan = data;
    isActualScanData = true;
}

// Сортировка данных с лазерного сканера
void buildMapFromLaserScaner(){
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
    for(int i = 0; i < mapSize; i++)
        for(int j = 0; j < mapSize; j++){
            localMap.data[mapSize * j + i] = 50;
        }

    for(uint m = 0; m < r.size(); m++){
        if(r[m] < 6 && r[m] > 0.5){
            // Преобразование координат новой точки из ПСК в ДСК
            int x2 = int(r[m] * cos(theta[m])/mapResolution + mapSize/2);
            int y2 = int(r[m] * sin(theta[m])/mapResolution + mapSize/2);

            if(x2 >= 0 && x2 < mapSize && y2 >= 0 && y2 < mapSize) {
                for(float dist = 0; dist < r[m]; dist += mapResolution) {
                    int x3 = dist * cos(theta[m])/mapResolution + mapSize/2;
                    int y3 = dist * sin(theta[m])/mapResolution + mapSize/2;
                    localMap.data[mapSize * y3 + x3] = 0;
                }
                localMap.data[mapSize * y2 + x2] = 100;
                localMap.data[mapSize * (y2 + 1) + (x2 + 1)] = 100;
                localMap.data[mapSize * (y2 + 1) + x2] = 100;
                localMap.data[mapSize * (y2 + 1) + (x2 - 1)] = 100;
                localMap.data[mapSize * (y2 - 1) + (x2 + 1)] = 100;
                localMap.data[mapSize * (y2 - 1) + (x2 - 1)] = 100;
                localMap.data[mapSize * (y2 - 1) + x2] = 100;
                localMap.data[mapSize * y2 + (x2 - 1)] = 100;
                localMap.data[mapSize * y2 + (x2 + 1)] = 100;
            }
        }
    }
}
void localMapMessageInitParams(){
    localMap.info.height = mapSize;
    localMap.info.width = mapSize;
    localMap.info.resolution = mapResolution;
    localMap.info.origin.position.x = -mapSize * mapResolution/2;
    localMap.info.origin.position.y = -mapSize * mapResolution/2;
    localMap.header.frame_id = "/laser";
    localMap.header.stamp = ros::Time::now();
    localMap.data.resize(localMap.info.height * localMap.info.width);
}
