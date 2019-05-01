#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setFNNConfig.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector4d;
using Eigen::Vector3d;

#define CENTER_E 		-1, 0, 1
#define CENTER_E_D 		-1, 0, 1
#define VARIANCE_E 		0.5, 0.5, 0.5
#define VARIANCE_E_D            0.5, 0.5, 0.5
#define WEIGHTS 		0.001, -0.001, 0.001, -0.001, 0.001, -0.001, 0.001, -0.001, 0.001
#define ALPHA 			0.001
#define GAMMA                   1.5
#define NU                      0.2

// Subscribers
ros::Subscriber odometry_subscriber;
ros::Subscriber trajectory_subscriber;
ros::Subscriber trajectory_velocity_subscriber;
ros::Subscriber pd_subscriber;

// Publishers
ros::Publisher velocity_publisher;
ros::Publisher fnn_velocity_publisher;

// Actual state
Vector4d pose;
Vector4d velocity;
Vector4d pose_d;
Vector4d velocity_d;

Vector4d tau_c;
Vector4d tau_f;

Vector4d e;
Vector4d e_d;
Vector4d e_dd;
Vector4d e_d_old;
MatrixXd c1(4, 3);
MatrixXd c2(4, 3);
MatrixXd sigma1(4, 3);
MatrixXd sigma2(4, 3);
MatrixXd f(4, 9);

// Support
VectorXd W(9);
VectorXd W_bar(9);
Vector3d mu1;
Vector3d mu2;
Vector3d c1_d;
Vector3d c2_d;
Vector3d sigma1_d;
Vector3d sigma2_d;
VectorXd f_d(9);
double alpha_d;

// Gains
Vector4d alpha;
Vector4d gamma1;
Vector4d nu;

short fnn_type;

bool new_odometry;

class FNN{
public:
    FNN(int, char**);
    ~FNN();
    void run();

private:
    double SMC(short axis, double x1, double d_x1, double x2, double d_x2, double tau_c, double dt);
    double LM(short axis, double x1, double d_x1, double x2, double d_x2, double tau_c, double dt);
    double Gaussian(double c, double sigma, double x);
    double elliptic(double a, double c, double d, double x);
    short sign(double x);
    double bound(double v, double min, double max);
    VectorXd bound(VectorXd v, double min, double max);
};
