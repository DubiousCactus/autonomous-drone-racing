#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setSafeIntelDroneConfig.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/BatteryState.h>

#define MAX_X   7
#define MAX_Y   7
#define MAX_Z   4
#define MAX_V   0.5
#define MAX_RP  (M_PI / 10)

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
Publisher odometry_publisher;
Publisher noisy_odometry_publisher;
Publisher attitude_publisher;
Publisher throttle_publisher;
Publisher position_publisher;
Publisher pose_publisher;

// Subsribers
Subscriber command_subscriber;
Subscriber command_position_subscriber;
Subscriber command_velocity_subscriber;
Subscriber command_attitude_subscriber;
Subscriber optitrack_subscriber;
Subscriber noise_subscriber;
Subscriber battery_subscriber;
Subscriber vicon_subscriber;

// Services
ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;

//MavROS
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandTOL land_cmd;

// Variables
Point position;
Point orientation;
Vector4d velocity;
Vector4d velocity_d;
Vector4d position_d;
Vector4d attitude_d;
vector<Twist> velocities;
nav_msgs::Odometry noise;
Time time_old;
int sequence;
double yaw;
bool stop;
int controller;
bool marker_visibile;

class SafeIntelDrone{
    public:
          SafeIntelDrone(int, char**);
          ~SafeIntelDrone();
          void run();
    private:
          void push(double vx, double vy, double vz, double p, double q, double r);
          Twist filter();
};
