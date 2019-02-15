#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/QuaternionStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setDIT1FLCConfig.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::Vector4d;

// Subscribers
ros::Subscriber odometry_subscriber;
ros::Subscriber trajectory_subscriber;
ros::Subscriber trajectory_velocity_subscriber;

// Publishers
ros::Publisher velocity_publisher;

// Actual state
Vector4d pose;
Vector4d velocity;
Vector4d pose_d;
Vector4d velocity_d;

// Error
Vector4d error;
Vector4d error_old;
Vector4d error_d;

// Fuzzy variables
Vector4d sigma1;
Vector4d sigma2;
Vector4d phi_p;
Vector4d phi_i;

// Gains
double k_p;
double k_d;
double k_a;
double k_b;

bool new_odometry;

class DI_T1_FLC_FM{
        public:
          DI_T1_FLC_FM(int, char**);
          ~DI_T1_FLC_FM();
          void run();
        private:
          double denormalizeAngle(double a1, double a2);
          double phi(double sigma1, double sigma2);
          double bound(double n);
};
