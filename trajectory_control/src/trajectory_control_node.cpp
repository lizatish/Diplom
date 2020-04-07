#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <signal.h>
#include <termios.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <random>
using namespace std;

void odometryCallback(const nav_msgs::Odometry &data);
void targetPathCallback(const nav_msgs::Path &data);
void goToNewCoordinates();
void slamOutPoseCallback(const geometry_msgs::PoseStamped& data);
// Получить рандомное число в диапазоне
float getRandom(float minValue, float maxValue);
// Разгладить шумы [для корректной одометрии]
float noiseSmoothing(float oldValue, float newValue, float koeff);

void mySigintHandler(int sig);
static int kfd = 0;
static struct termios cooked, raw;

//Текущее положение и угол платформы
static geometry_msgs::Pose currentPosition;
static double yawAngle;

static bool isCameOdom = false;
static bool isCameTargetPath = false;
static geometry_msgs::PoseStamped goal_rviz;

// Управляющее сообщение
static geometry_msgs::Twist commandVelocities;
// Публикатор желаемой скорости базы
static ros::Publisher kuka_movebase_publisher;
// Для хранения целевого путя
static nav_msgs::Path targetPath;

//// ИЗМЕНЯЕМЫЕ ПАРАМЕТРЫ
// Радиус поворота
static const double CURVATURE = 0.3;
// Минимальне расстояние для достижения цели
static double DIST_TO_TARGET_MIN = 0.07;

// Угловая погрешность +- 10 градусов
static const double YAW_MIN = -0.074533;
static const double YAW_MAX = 0.074533;
// Погрешность по x +- 30 см
static const double X_MIN = -0.3;
static const double X_MAX = 0.3;
// Погрешность по y +- 30 см
static const double Y_MIN = -0.3;
static const double Y_MAX = 0.3;
/////////////////////////////

int main(int argc, char **argv){
    // Инициализация ROS
    ros::init(argc, argv, "kuka_traffic_control");

    ros::NodeHandle m;
    ros::Subscriber odom_sub = m.subscribe("/odom", 8, odometryCallback);
    ros::Subscriber target_path_sub = m.subscribe("/target_path", 8, targetPathCallback);
    //    ros::Subscriber slam_current_pose_sub = m.subscribe("/slam_out_pose", 8, slamOutPoseCallback);
    ros::Publisher goal_pub = m.advertise<geometry_msgs::PoseStamped>("/goal0000", 8);

    kuka_movebase_publisher = m.advertise<geometry_msgs::Twist> ("kuk_keyboard_control/pose_commands", 32);

    //Функция при завершении работы
    signal(SIGINT, mySigintHandler);

    bool isAllowProcess = true;
    ros::Rate rate(100);
    while(isAllowProcess && ros::ok()) {
        if(isCameTargetPath && isCameOdom){

            goToNewCoordinates();

            // Параметры текущей точки, к которой движемся
            if(targetPath.poses.size()){
                goal_rviz.pose = targetPath.poses.at(0).pose;
                goal_rviz.header.frame_id = "odom";
                goal_rviz.header.stamp = ros::Time::now();
            }
        }

        kuka_movebase_publisher.publish(commandVelocities);
        cout<<"Publish command velocities "
           << commandVelocities.linear.x << " " << commandVelocities.angular.z << endl;
        goal_pub.publish(goal_rviz);
        rate.sleep();
        ros::spinOnce();
    }
}

void goToNewCoordinates(){
    cout << "Tar path size " << targetPath.poses.size() << endl;
    if(targetPath.poses.size()){
        // Координаты: текущие и целевые
        double current_x = currentPosition.position.x;
        double current_y = currentPosition.position.y;
        double target_x = targetPath.poses.at(0).pose.position.x;
        double target_y = targetPath.poses.at(0).pose.position.y;
        double dist = sqrt(pow((current_y) - target_y, 2) + pow((current_x) - target_x, 2));

        cout << "x " << current_x << " y " << current_y  << " dist " << dist << endl;
        cout << "Target x: " << target_x << " y: " << target_y << endl;

        // Если не достигнуто предельное расстояние до цели
        if(dist > DIST_TO_TARGET_MIN){
            dist = sqrt(pow((current_y) - target_y, 2)+ pow((current_x) - target_x, 2));
            double target_yaw = atan2(target_y - current_y, target_x - current_x);
            double yaw_diff = yawAngle - target_yaw;

            if(yaw_diff > M_PI)
                yaw_diff -= 2 * M_PI;
            if(yaw_diff < -M_PI)
                yaw_diff += 2 * M_PI;
            cout << " curAng " << yawAngle << " tarAng " << target_yaw << " diff " << yaw_diff << endl;

            // Задаем движение
            commandVelocities.linear.x = 0.5;
            if(abs(yaw_diff) > 0.01){ // Поворот
                if(yaw_diff > 0.01)
                    commandVelocities.angular.z = -commandVelocities.linear.x/CURVATURE;
                else
                    commandVelocities.angular.z = commandVelocities.linear.x/CURVATURE;
            }
            else{ // Прямолинейное
                commandVelocities.angular.z = 0;
            }
        }
        // Если расстояние достигнуто
        else{
            // Удаляем достигнутую точку
            targetPath.poses.erase(targetPath.poses.begin());
        }
    }
    // Если пути нет, посылаем нулевые скорости
    else{
        commandVelocities.angular.z = 0;
        commandVelocities.linear.x = 0;
    }
}

// Подписчик на запланированный путь
void targetPathCallback(const nav_msgs::Path &data){
    targetPath = data;
    isCameTargetPath = true;
    cout << "Target path changed, new size is " << targetPath.poses.size() << endl;
}

// Подписчик на одометрию из СЛАМ
void slamOutPoseCallback(const geometry_msgs::PoseStamped& data){
    currentPosition = data.pose;
    isCameOdom = true;
    yawAngle = tf::getYaw(data.pose.orientation);
}

// Подписчик на одометрию
void odometryCallback(const nav_msgs::Odometry &data){
    //    currentPosition = data.pose.pose;

    // Добавление рандомных шумов в заданном диапазоне
    float yawAngle_noise = tf::getYaw(data.pose.pose.orientation) + getRandom(YAW_MIN, YAW_MAX);
    geometry_msgs::Point currentPosition_noise;
    currentPosition_noise.x = data.pose.pose.position.x + getRandom(X_MIN, X_MAX);
    currentPosition_noise.y = data.pose.pose.position.y + getRandom(Y_MIN, Y_MAX);

    geometry_msgs::Point previous_currentPosition = currentPosition.position;
    double previous_yawAngle = yawAngle;

    // Сглаживание шумов
    currentPosition.position.x = noiseSmoothing(currentPosition.position.x, currentPosition_noise.x, 0.9);
    currentPosition.position.y = noiseSmoothing(currentPosition.position.y, currentPosition_noise.y, 0.9);
    yawAngle = noiseSmoothing(yawAngle, yawAngle_noise, 0.9);


    //  cout << "x: previous " << setw(11) << previous_currentPosition.x
    //       << setw(6) << "   new " << setw(11) << currentPosition_noise.x
    //       << "   general " << setw(11) << currentPosition.x
    //       << setw(6) << "   diff "<< setw(11) <<currentPosition.x - previous_currentPosition.x<< endl;

    //  cout << "y: previous " << setw(11) <<previous_currentPosition.y
    //       << setw(6) << "   new "  << setw(11) << currentPosition_noise.y
    //       << setw(6) <<"   general " <<setw(11) << currentPosition.y
    //       << setw(6) << "   diff "<< setw(11) <<currentPosition.y - previous_currentPosition.y<< endl;

    //  cout << "z: previous " << setw(11) <<previous_yawAngle
    //       << setw(6) << "   new " << setw(11) << yawAngle_noise
    //       << setw(6) << "   general " << setw(11) <<yawAngle
    //       << setw(6) << "   diff "<< setw(11) <<yawAngle - previous_yawAngle<< endl;

    isCameOdom = true;
    //    yawAngle = tf::getYaw(data.pose.pose.orientation);
}
float noiseSmoothing(float oldValue, float newValue, float koeff){
    return (oldValue * koeff + newValue * (1 - koeff));
}
float getRandom(float minValue, float maxValue)
{
    static std::default_random_engine e;
    static std::uniform_real_distribution<> dis(minValue, maxValue);
    return dis(e);
}
// Подача нулевой скорости при закрытии терминала Ctrl+C
void mySigintHandler(int sig) {

    ROS_INFO("KUK KEYBOARD CONTROL stopped");
    tcsetattr(kfd, TCSANOW, &cooked);

    commandVelocities.linear.x = 0;
    commandVelocities.linear.y = 0;
    commandVelocities.angular.z = 0;

    kuka_movebase_publisher.publish(commandVelocities);
    ros::spinOnce();
    exit(0);
}
