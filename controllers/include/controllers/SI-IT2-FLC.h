#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setSIT2FLCConfig.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::Matrix3d;
using Eigen::Vector4d;

#define E_MAX 	2
#define DE_MAX 	2

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

Vector4d error;
Vector4d error_i;
Vector4d error_d;
Vector4d error_old;
Vector4d sigma;
Vector4d sigma_d;
Vector4d phi_p;
Vector4d phi_i;
Vector4d phi_d;
Vector4d phi0_d;
Vector4d phi0_i;
Vector4d phi0_d_old;

// Gains
double k_p;
double k_i;
double k_d;
double k_e;
double k_de;
double alpha_p;
double alpha_i;
double alpha_d;

bool new_odometry;

class SIT2FLC{
        public:
          SIT2FLC(int, char**);
          ~SIT2FLC();
          void run();

        private:
          double T2(double x1, double m);
          Vector3d beta(double x, Matrix3d& d);
};
