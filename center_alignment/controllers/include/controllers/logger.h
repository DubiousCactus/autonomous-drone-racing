#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <std_msgs/Int8.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::Vector3d;
using Eigen::Vector4d;

// Subscribers
ros::Subscriber trajectory_subscriber;
ros::Subscriber trajectory_velocity_subscriber;
ros::Subscriber mocap_subscriber;
ros::Subscriber bebop_subscriber;
ros::Subscriber bebop_altitude_subscriber;
ros::Subscriber vision_subscriber;
ros::Subscriber vision_quality_subscriber;
ros::Subscriber controller_subscriber;
ros::Subscriber command_subscriber;
ros::Subscriber teleop_subscriber;

// Data
Time beginning(0);
Vector4d pose_d;
Vector3d velocity_d;
Vector3d position_mocap;
Vector3d orientation_mocap;
Vector3d velocity_mocap;
Vector3d rate_mocap;
Vector3d position_bebop;
double altitude_bebop;
Vector3d orientation_bebop;
Vector3d velocity_bebop;
Vector3d rate_bebop;
Vector3d position_vision;
Vector3d orientation_vision;
Vector3d velocity_vision;
Vector3d rate_vision;
int tracking_quality;
Vector4d command_controller;
Vector4d command_bebop;

// File
ofstream results;

bool flying;

class Logger{
        public:
          Logger(int, char**);
          ~Logger();
          void run();
};
