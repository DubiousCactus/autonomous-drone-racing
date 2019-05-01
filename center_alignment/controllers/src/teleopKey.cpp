#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <pthread.h>

#define KEYCODE_A       97
#define KEYCODE_D       100
#define KEYCODE_W       119
#define KEYCODE_S       115
#define KEYCODE_H       104
#define KEYCODE_K       107
#define KEYCODE_U       117
#define KEYCODE_J       106
#define KEYCODE_LEFT    68
#define KEYCODE_RIGHT   67
#define KEYCODE_UP      65
#define KEYCODE_DOWN    66
#define KEYCODE_Q       113
#define KEYCODE_R       114
#define KEYCODE_T       116
#define KEYCODE_L       108
#define KEYCODE_ESC     27

#define MAX_SPEED       1.0

using namespace std;

int kfd = 0;
char c = 0;
struct termios cooked, raw;

class Teleop{
private:
    ros::NodeHandle node_handle;
    ros::Publisher command_publisher;
    ros::Publisher velocity_publisher;
    ros::Publisher camera_publisher;
    std_msgs::Int8 command;
    geometry_msgs::Quaternion velocity;
    geometry_msgs::Twist velocity_twist;
    geometry_msgs::Vector3 camera;

public:
    Teleop(){
        command.data = 0;
        velocity.x = 0;
        velocity.y = 0;
        velocity.z = 0;
        velocity.w = 0;

        velocity_twist.linear.x = 0;
        velocity_twist.linear.y = 0;
        velocity_twist.linear.z = 0;
        velocity_twist.angular.z = 0;

        command_publisher = node_handle.advertise<std_msgs::Int8>("/uav/command", 1);
        velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/uav/command_velocity_keyboard", 1);
        camera_publisher = node_handle.advertise<geometry_msgs::Vector3>("/uav/command_camera", 1);
    }

    ~Teleop(){
        tcsetattr(kfd, TCSANOW, &cooked);
        ros::shutdown();
        exit(0);
    }

    void keyLoop(){
        ros::Rate rate(100);
        while(ros::ok()){
            rate.sleep();
            //cout << "[Teleop]: c = " << (int)c << endl;
            switch(c){
            /* UAV movement */
            case KEYCODE_A:
                velocity.w = min(MAX_SPEED, velocity.w + MAX_SPEED);
                break;
            case KEYCODE_D:
                velocity.w = max(-MAX_SPEED, velocity.w - MAX_SPEED);
                break;
            case KEYCODE_W:
                velocity.z = min(MAX_SPEED, velocity.z + MAX_SPEED);
                break;
            case KEYCODE_S:
                velocity.z = max(-MAX_SPEED, velocity.z - MAX_SPEED);
                break;
            case KEYCODE_LEFT:
                velocity.y = min(MAX_SPEED, velocity.y + MAX_SPEED);
                break;
            case KEYCODE_RIGHT:
                velocity.y = max(-MAX_SPEED, velocity.y - MAX_SPEED);
                break;
            case KEYCODE_UP:
                velocity.x = min(MAX_SPEED, velocity.x + MAX_SPEED);
                break;
            case KEYCODE_DOWN:
                velocity.x = max(-MAX_SPEED, velocity.x - MAX_SPEED);
                break;


            /* camera movement */
            case KEYCODE_U:
                camera.y = min(20.0, camera.y + 1);
                break;
            case KEYCODE_J:
                camera.y = max(-40.0, camera.y - 1);
                break;
            case KEYCODE_H:
                camera.z = max(-34.0, camera.z - 1);
                break;
            case KEYCODE_K:
                camera.z = min(34.0, camera.z + 1);
                break;

            /* UAV commands */
            case KEYCODE_R:
                command.data = 1;
                velocity.x = 0;
                velocity.y = 0;
                velocity.z = 0;
                velocity.w = 0;
                command_publisher.publish(command);
                command_publisher.publish(command);
                break;
            case KEYCODE_T:
                command.data = 2;
                velocity.x = 0;
                velocity.y = 0;
                velocity.z = 0;
                velocity.w = 0;
                command_publisher.publish(command);
                command_publisher.publish(command);
                break;
            case KEYCODE_L:
                command.data = 3;
                velocity.x = 0;
                velocity.y = 0;
                velocity.z = 0;
                velocity.w = 0;
                command_publisher.publish(command);
                command_publisher.publish(command);
                break;
            case KEYCODE_ESC:
                command.data = 4;
                velocity.x = 0;
                velocity.y = 0;
                velocity.z = 0;
                velocity.w = 0;
                command_publisher.publish(command);
                command_publisher.publish(command);
                break;
            }
            velocity_twist.linear.x = velocity.x;
            velocity_twist.linear.y = velocity.y;
            velocity_twist.linear.z = velocity.z;
            velocity_twist.angular.z = velocity.w;
            if(command.data != 0 || velocity.x != 0 || velocity.y != 0 || velocity.z != 0 || velocity.w != 0 || c != 0)
                velocity_publisher.publish(velocity);
            camera_publisher.publish(camera);
            c = 0;
            command.data = 0;
        }
    }
};

void quit(int sig){
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

//This function will be called from a thread
void *readKey(void *) {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    while(true){
        // get the next event from the keyboard
        read(kfd, &c, 1);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "keyboard_teleop");

    cout << "Press key: \n";
    cout << "r \t - reset \n";
    cout << "t \t - take-off \n";
    cout << "l \t - land \n";
    cout << "UP \t - move forward \n";
    cout << "DOWN \t - move backward \n";
    cout << "LEFT \t - move left \n";
    cout << "RIGHT \t - move right \n";
    cout << "w \t - move up \n";
    cout << "s \t - move down \n";
    cout << "a \t - rotate clockwise \n";
    cout << "d \t - rotate counterclockwise \n";

    pthread_t t;
    // Launch a thread
    pthread_create(&t, NULL, readKey, NULL);

    signal(SIGINT, quit);

    Teleop teleop;
    teleop.keyLoop();

    ros::spin();

    return 0;
}
