#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Quaternion.h>
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
	ros::Publisher motor1_publisher;
	ros::Publisher motor1_invert_publisher;
    ros::Publisher motor2_publisher;
	ros::Publisher motor2_invert_publisher;
    ros::Publisher motor3_publisher;
	//ros::Subscriber sub("replying", &reply_cb);
    std_msgs::Int8 command;
	std_msgs::Int8 command_grip_motor;
	std_msgs::Int8 command_yaw_motor;
	//std_msgs::Int8 elevator_motor;
	//std_msgs::Int8 yaw_motor;
	//std_msgs::Int8 grip_motor;
    //geometry_msgs::Quaternion velocity;
    //geometry_msgs::Vector3 camera;

public:
    Teleop(){
        command.data = 0;
		command_grip_motor.data = 0;
		command_yaw_motor.data = 0;
        /*elevator_motor.data = 0;
        yaw_motor.data = 0;
        grip_motor.data = 0;
		*/
        command_publisher = node_handle.advertise<std_msgs::Int8>("command", 1);
        motor1_publisher = node_handle.advertise<std_msgs::Int8>("command_elevator_motor", 1);
		motor1_invert_publisher = node_handle.advertise<std_msgs::Int8>("command_elevator_motor_invert", 1);
        motor2_publisher = node_handle.advertise<std_msgs::Int8>("command_yaw_motor", 1);
		motor2_invert_publisher = node_handle.advertise<std_msgs::Int8>("command_yaw_motor_invert", 1);
		motor3_publisher = node_handle.advertise<std_msgs::Int8>("command_grip_motor", 1);
		//wait = node_handle.advertise("replying", &reply_cb);
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
            /* individual motor control */
            case KEYCODE_DOWN:
                //elevator_motor.data = 1;
				command.data = 1;
				cout << command;
				motor1_publisher.publish(command);
				break;
			case KEYCODE_UP:
                //elevator_motor.data = 1;
				command.data = 1;
				cout << command;
				motor1_invert_publisher.publish(command);
				break;
            case KEYCODE_LEFT:
				//yaw_motor.data = 1;
				command.data = 1;
				cout << command;
				motor2_publisher.publish(command);
                break;
			case KEYCODE_RIGHT:
				//yaw_motor.data = 1;
				command.data = 1;
				cout << command;
				motor2_invert_publisher.publish(command);
                break;
            case KEYCODE_W:
                //grip_motor.data = 1;
				command.data = 1;
				cout << command;
				motor3_publisher.publish(command);
                break;

            /* action sequence */
            case KEYCODE_U:
                command.data = 1;
                command_publisher.publish(command);
                break;
            case KEYCODE_H:
                command.data = 1;
                command_publisher.publish(command);
                //command_publisher.publish(command);
                break;
			}
            /* publish message */

            //camera_publisher.publish(camera);
			/*motor1_publisher.publish(elevator_motor);
			motor2_publisher.publish(yaw_motor);
			motor3_publisher.publish(grip_motor);*/
            c = 0;
			command.data = 0;
        }
    }
};

/*
void reply_cb( const std_msgs::Empty& toggle_msg){
     cout << toggle_msg;
}
*/
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
    cout << "U \t - reset \n";
    cout << "H \t - gripping/retracting sequence \n";
    cout << "DOWN \t - Descend \n";
	cout << "UP \t - Elevate \n";
    cout << "LEFT \t - yaw forward \n";
	cout << "RIGHT \t - yaw backward \n";
    cout << "w \t - gripper \n";

    pthread_t t;
    // Launch a thread
    pthread_create(&t, NULL, readKey, NULL);

    signal(SIGINT, quit);

    Teleop teleop;
    teleop.keyLoop();

    ros::spin();

    return 0;
}
