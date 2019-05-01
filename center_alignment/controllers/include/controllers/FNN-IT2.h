#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setFNN_IT2Config.h>
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
using Eigen::Vector2d;

#define A1          1.5, 1.5, 1.5
#define A2          0.5, 0.5, 0.5
#define C           -1, 0, 1
#define D           1, 1, 1

#define SIGMA1      1.5, 1.5, 1.5
#define SIGMA2      0.5, 0.5, 0.5

#define WEIGHTS     0.001, -0.001, 0.001, -0.001, 0.001, -0.001, 0.001, -0.001, 0.001

#define ALPHA       0.001
#define GAMMA       0.1

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

Vector4d u_c;
Vector4d u_f;

Vector4d e;
Vector4d e_d;
MatrixXd f(4, 9);
Vector4d q;

MatrixXd a11(4, 3);
MatrixXd a12(4, 3);
MatrixXd a21(4, 3);
MatrixXd a22(4, 3);
MatrixXd c1(4, 3);
MatrixXd c2(4, 3);
MatrixXd d1(4, 3);
MatrixXd d2(4, 3);

MatrixXd sigma11(4, 3);
MatrixXd sigma12(4, 3);
MatrixXd sigma21(4, 3);
MatrixXd sigma22(4, 3);

// Support
VectorXd W_upper(9);
VectorXd W_lower(9);
VectorXd W_upper_bar(9);
VectorXd W_lower_bar(9);
VectorXd Pi(9);
Vector3d mu1_upper;
Vector3d mu1_lower;
Vector3d mu2_upper;
Vector3d mu2_lower;
Vector3d c1_d;
Vector3d c2_d;
Vector3d sigma1_d;
Vector3d sigma2_d;
VectorXd f_d(9);
double q_d;
double alpha_d;

// Gains
Vector4d alpha;
Vector4d gamma1;
Vector4d nu;

short mf_type;

bool new_odometry;

class FNN_IT2{
        public:
          FNN_IT2(int, char**);
          ~FNN_IT2();
          void run();

        private:
          double update(short axis, double x1, double x2, double u_c, double dt);
          double Gaussian(double c, double sigma, double x);
          double elliptic(double a, double c, double d, double x);
          short sign(double x);
          double bound(double v, double min, double max);
          VectorXd bound(VectorXd v, double min, double max);
};
