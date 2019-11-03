#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <aruco_mapping/ArucoMarker.h>

#define MAX_X           3
#define MAX_Y           3
#define MAX_Z           3
#define MAX_V           5
#define MAX_BEBOP_V     18

using namespace geometry_msgs;
using namespace std;
using namespace ros;

// Publishers
Publisher odometry_publisher;
Publisher noisy_odometry_publisher;

// Subsribers
Subscriber optitrack_subscriber;
Subscriber noise_subscriber;
Subscriber aruco_subscriber;
Subscriber gps_subscriber;
Subscriber bebop_subscriber;

// Variables
Point position;
Point orientation;
Point velocity;
vector<Twist> velocities;
nav_msgs::Odometry odometry_optitrack;
nav_msgs::Odometry odometry_aruco;
nav_msgs::Odometry odometry_gps;
nav_msgs::Odometry odometry_bebop;

nav_msgs::Odometry noise;
geometry_msgs::Pose pose_last;
geometry_msgs::Pose pose_initial_aruco;
geometry_msgs::Pose pose_initioal_gps;
geometry_msgs::Pose pose_initial_bebop;
Time time_old;
int sequence;
double yaw;

// Flags
bool land;
bool optitrack_ok;
bool aruco_ok;
bool gps_ok;
bool bebop_ok;
bool odometry_ok;

class Odometry{
    public:
          Odometry(int, char**);
          ~Odometry();
          void run();
    private:
          void push(double vx, double vy, double vz, double p, double q, double r);
          Twist filter();
};
