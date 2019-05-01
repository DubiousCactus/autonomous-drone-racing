#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <aruco_mapping/ArucoMarker.h>
#include <ptam_com/ptam_info.h>
#include <Eigen/Dense>
#include <bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>

// Indoor
#define MAX_X           7
#define MAX_Y           7
#define MAX_Z           4
#define MAX_V           5
#define MAX_TILT        50
#define MAX_ROTATION    180
#define MAX_I           0.01
/*// Outdoor
#define MAX_X           30
#define MAX_Y           30
#define MAX_Z           10
#define MAX_V           10
#define MAX_TILT        50
#define MAX_ROTATION    100
#define MAX_I           0.01*/

// SpeedSettingsMaxVerticalSpeedCurrent = 1

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::Vector4d;

// Publishers
Publisher reset_publisher;
Publisher takeoff_publisher;
Publisher land_publisher;
Publisher velocity_publisher;
Publisher camera_publisher;
Publisher odometry_publisher;
Publisher noisy_odometry_publisher;
Publisher pose_publisher;
Publisher imu_publisher;

// Subsribers
Subscriber command_subscriber;
Subscriber command_velocity_subscriber;
Subscriber command_camera_subscriber;
Subscriber optitrack_subscriber;
Subscriber vicon_subscriber;
Subscriber aruco_subscriber;
Subscriber noise_subscriber;
Subscriber gps_subscriber;
Subscriber odometry_subscriber;
Subscriber vslam_subscriber;
Subscriber vslam_info_subscriber;
Subscriber altitude_subscriber;
Subscriber battery_subscriber;

// Velocity PID
Vector4d k_p;
Vector4d k_i;
Vector4d k_d;
Vector4d error;
Vector4d error_old;
Vector4d error_i;
Vector4d error_d;
Vector4d velocity;
Vector4d velocity_old;
Vector4d velocity_d;

// Variables
Point position;
Point orientation;
vector<Twist> velocities;
nav_msgs::Odometry noise;
Time time_old(0);
int sequence;
double yaw;
bool land;
bool marker_visibile;
Point vslam_initial_position;
double vslam_initial_yaw = 0;
bool vslam_initialased = false;
int battery;

class SafeBebop{
    public:
          SafeBebop(int, char**);
          ~SafeBebop();
          void run();
    private:
          void push(double vx, double vy, double vz, double p, double q, double r);
          Twist filter();
};
