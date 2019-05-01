#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setDIT2FLCConfig.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector4d;

#define E_MAX 10

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
Vector4d sigma_p;
Vector4d sigma_d;
Vector4d phi;
Vector4d phi_i;

// Gains
double k_p;
double k_i;
double k_d;
double k_a;
double k_b;
double alpha_p;
double alpha_d;

bool new_odometry;

class DIT2FLC{
public:
    DIT2FLC(int, char**);
    ~DIT2FLC();
    void run();
    double evalt2(double ep, double ed, double m_p, double m_d);

private:
    double km(MatrixXd F, MatrixXd Y);
    double trimf(double x, double a, double b, double c);
    MatrixXd sort(VectorXd x, MatrixXd M);
    VectorXd sum(MatrixXd M);
    double denormalizeAngle(double a1, double a2);
};
