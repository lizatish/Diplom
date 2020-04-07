#include "rrt.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

void odometryCallback(const nav_msgs::Odometry data);
void globalMapCallback(const nav_msgs::OccupancyGrid& data);
void goalFromRvizCallback(const geometry_msgs::PoseStamped& data);
void slamOutPoseCallback(const geometry_msgs::PoseStamped& data);
bool checkObstaclesOnPath();
nav_msgs::Path formPath(geometry_msgs::Pose s, geometry_msgs::Pose f);

// Сообщение с путем
static nav_msgs::Path current_path;
static nav_msgs::Path previous_path;

static double ROBOT_HEIGHT = 0.7;
static double ROBOT_WIDTH = 0.5;
static const double CURVATURE = 0.3;

//Текущее положение платформы
static geometry_msgs::Pose currentPosition;
static nav_msgs::OccupancyGrid globalMap;

static bool isCameGlobalMap = false;
static bool isCameOdom = false;
static bool isGoalCame = false;
static bool isObstaclesOnPath = true;

static geometry_msgs::Pose goal;
static geometry_msgs::PoseStamped goal_rviz;

int main(int argc, char **argv){
    ros::init(argc, argv, "kuka_path_searcher_node");

    // Создание публикатора пути
    ros::NodeHandle l;
    ros::Publisher path_for_rviz_pub = l.advertise<nav_msgs::Path>("/path_rrt", 8);
    ros::Publisher path_for_control_pub = l.advertise<nav_msgs::Path>("/target_path", 8);
    ros::Subscriber global_map_sub = l.subscribe("/global_map", 8, globalMapCallback);
    ros::Subscriber odom_sub = l.subscribe("/odom", 8, odometryCallback);
    ros::Publisher goal_pub = l.advertise<geometry_msgs::PoseStamped>("/rrt_goal", 8);
        ros::Subscriber goal_from_rviz_sub = l.subscribe("/move_base_simple/goal", 8, goalFromRvizCallback);
    //    ros::Subscriber slam_current_pose_sub = l.subscribe("/slam_out_pose", 8, slamOutPoseCallback);


//    goal.position.x = 5;
//    goal.position.y = 5;
//    goal.orientation = tf::createQuaternionMsgFromYaw(1.5);
//    isGoalCame = true;


    ros::Rate rate(100);
    bool isAllowProcess = true;
    while(ros::ok() && isAllowProcess){
        if(isCameOdom && isCameGlobalMap && isGoalCame){
            if(isObstaclesOnPath){
                ros::Time start_time = ros::Time::now();
                cout << "Start path find... ";
                current_path = formPath(currentPosition, goal);
                cout << ros::Time::now().toSec() - start_time.toSec() << endl;
                path_for_control_pub.publish(current_path);
            }
            checkObstaclesOnPath();

            goal_rviz.pose = goal;
            goal_rviz.header.frame_id = "odom";
            goal_rviz.header.stamp = ros::Time::now();
        }
        path_for_rviz_pub.publish(current_path);
        goal_pub.publish(goal_rviz);
        ros::spinOnce();
        rate.sleep();
    }
}
void slamOutPoseCallback(const geometry_msgs::PoseStamped& data){
    currentPosition = data.pose;
    isCameOdom = true;
}

void odometryCallback(const nav_msgs::Odometry data){
    currentPosition = data.pose.pose;
    isCameOdom = true;
}
void goalFromRvizCallback(const geometry_msgs::PoseStamped& data){
  cout << data << endl;

    goal = data.pose;
    isGoalCame = true;
    goal_rviz.pose = goal;
    goal_rviz.header.frame_id = "odom";
    goal_rviz.header.stamp = ros::Time::now();
}

void globalMapCallback(const nav_msgs::OccupancyGrid& data){
    globalMap = data;
    isCameGlobalMap = true;
}

bool checkObstaclesOnPath(){
    uint map_width = globalMap.info.width;
    uint map_height = globalMap.info.height;
    float mapResolution = globalMap.info.resolution;
    int robot_border_down = -ROBOT_HEIGHT/(2*mapResolution);
    int robot_border_left = -ROBOT_WIDTH/(2*mapResolution);
    int robot_border_up = -robot_border_down;
    int root_border_right = -robot_border_left;

    if (!current_path.poses.size()){
        isObstaclesOnPath = true;
        return true;
    }

    for(int k = 0; k < current_path.poses.size(); k++){

        int path_x = int(current_path.poses.at(k).pose.position.x/mapResolution) + map_width/2;
        int path_y = int(current_path.poses.at(k).pose.position.y/mapResolution) + map_height/2;
        float robot_yaw = tf::getYaw(current_path.poses.at(k).pose.orientation);

        if(path_x + robot_border_down >= 0 && path_x + robot_border_up < map_width
                && path_y + robot_border_left >= 0 && path_y + root_border_right < map_height)
            for(int i = robot_border_down; i <= robot_border_up; i++){
                for(int j = robot_border_left; j <= root_border_right; j++){

                    // Составляющая поворота
                    int robot_area_x = path_x + i * cos(robot_yaw) + j * sin(robot_yaw);
                    int robot_area_y = path_y - i * sin(robot_yaw) + j * cos(robot_yaw);

                    if(int(globalMap.data[map_width * robot_area_y + robot_area_x]) > 75){
                        isObstaclesOnPath = true;
                        return true;
                    }
                }
            }
    }
    isObstaclesOnPath = false;
    return false;
}

nav_msgs::Path formPath(geometry_msgs::Pose s, geometry_msgs::Pose f){

    vector<nav_msgs::Path> path_various;
    nav_msgs::Path path;
    int counter = 0;
    while(counter <= 3){
        RRT* rrt = new RRT();
        path = rrt->Planning(s, f, globalMap, CURVATURE, ROBOT_HEIGHT, ROBOT_WIDTH);
        if(path.poses.size() != 0)
            path_various.push_back(path);
        counter++;
        delete rrt;
    }
    cout << path_various.size() << endl;

    nav_msgs::Path shortest_path;
    if(path_various.size() != 0){
        isObstaclesOnPath = false;
        shortest_path = path_various.at(0);
        for(int i = 1; i < path_various.size() - 1; i++){
            if(path_various.at(i).poses.size() < path_various.at(i+1).poses.size())
                shortest_path = path_various.at(i);
        }
    }
    return shortest_path;
}
