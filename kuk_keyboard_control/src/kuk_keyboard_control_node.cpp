/*Описание модуля
* Модуль предназначен для захвата команд с клавиатуры
* и передачи их на моудль управления базой и (по необходимости) рукой
*/



#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include <cstdlib>
#include <deque>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdlib.h>
#include <iostream>
#include <iostream>
#include <assert.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <brics_actuator/CartesianWrench.h>
#include <brics_actuator/JointVelocities.h>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <iostream>
#include <brics_actuator/JointPositions.h>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>
#include <signal.h>


#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdlib.h>


#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_U 0x75
#define KEYCODE_J 0x6A
#define KEYCODE_I 0x69
#define KEYCODE_K 0x6B
#define KEYCODE_SAPCEBAR 0x20

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45



using namespace std;

//Текущая ориентация платформы
static geometry_msgs::Quaternion currentOrientation;

//Скорость
static geometry_msgs::Vector3 baseLinearVelosities;
static geometry_msgs::Vector3 baseAngularVelosities;

//Публикатор желаемой скорости базы
ros::Publisher baseVelocityPublisher;

//Управляющее сообщение
geometry_msgs::Twist commandVelocities;

static int kfd = 0;
static struct termios cooked, raw;

static bool isSetCommand;


//Функция вызова при выключении
 void mySigintHandler(int sig) {

     ROS_INFO("KUK KEYBOARD CONTROL stopped");
     tcsetattr(kfd, TCSANOW, &cooked);
     exit(0);
 }

int main(int argc, char **argv) {

    //Начальная инициализация узла
    ros::init(argc, argv, "kuk_keyboard_control");
    ros::NodeHandle n;
    std::string frame_id;
    n.param<std::string> ("frame_id", frame_id, "/odom");

    //Сообщение о запуске
    ROS_INFO("KUKA KEYBOARD CONTROL started");

    //Функция при завершении работы
    signal(SIGINT, mySigintHandler);

    //Публикатор клавиатурных команд
    baseVelocityPublisher = n.advertise<geometry_msgs::Twist > ("kuk_keyboard_control/pose_commands", 512);

    //Частота работы
    ros::Rate rate(20); //Hz

    //Принимаемая команда
    char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use 'WASD' to translate");
    puts("Use 'QE' to yaw");

    isSetCommand = false;

    while(ros::ok()) {

        if(read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }
            ROS_INFO("key code %d}\n", c);
            switch(c) {

            /* Move commands */
            case KEYCODE_W:
                baseLinearVelosities.x = 1;
                baseLinearVelosities.y = 0;
                baseAngularVelosities.x = 0;
                baseAngularVelosities.y = 0;
                baseAngularVelosities.z = 0;
                isSetCommand = true;
                break;
            case KEYCODE_S:
                baseLinearVelosities.x = - 1;
                baseLinearVelosities.y = 0;
                baseAngularVelosities.x = 0;
                baseAngularVelosities.y = 0;
                baseAngularVelosities.z = 0;
                isSetCommand = true;
                break;
            case KEYCODE_A:
                baseLinearVelosities.y = 1;
                baseLinearVelosities.x = 0;
                baseAngularVelosities.x = 0;
                baseAngularVelosities.y = 0;
                baseAngularVelosities.z = 0;
                isSetCommand = true;
                break;
            case KEYCODE_D:
                baseLinearVelosities.y = - 1;
                baseLinearVelosities.x = 0;
                baseAngularVelosities.x = 0;
                baseAngularVelosities.y = 0;
                baseAngularVelosities.z = 0;
                isSetCommand = true;
                break;
            case KEYCODE_Q:
                baseAngularVelosities.z = 1;
                baseLinearVelosities.x = 0;
                baseLinearVelosities.y = 0;
                baseAngularVelosities.x = 0;
                baseAngularVelosities.y = 0;
                isSetCommand = true;
                break;
            case KEYCODE_E:
                baseAngularVelosities.z = - 1;
                baseLinearVelosities.x = 0;
                baseLinearVelosities.y = 0;
                baseAngularVelosities.x = 0;
                baseAngularVelosities.y = 0;
                isSetCommand = true;
                break;
            }

            if (isSetCommand == true) {

                cout<<"Publish "<<endl;
                //Формируем команду

                commandVelocities.linear = baseLinearVelosities;
                commandVelocities.angular = baseAngularVelosities;

                //Публикуем
                baseVelocityPublisher.publish(commandVelocities);
                isSetCommand = false;
            }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}
