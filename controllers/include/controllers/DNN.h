#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <math.h>
#include <time.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <dynamic_reconfigure/Config.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setDNNConfig.h>
#include <mavros_msgs/Thrust.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
//#include <tensorflow/c/c_api.h>
#include <opennn.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;
using namespace OpenNN;

using Eigen::Vector4d;

// Subscribers
ros::Subscriber odometry_subscriber;
ros::Subscriber trajectory_subscriber;
ros::Subscriber trajectory_velocity_subscriber;
ros::Subscriber controller_type_subscriber;
ros::Subscriber thrust_subscriber;

// Publishers
ros::Publisher velocity_publisher;

// Data
Vector3d position;
Vector4d pose;
Vector4d pose_d;
Vector3d orientation;
Vector3d velocity;
Vector3d angular_velocity;
Vector3d velocity_d;
Vector4d pid;
float thrust = 0, thrust_old = 0;

// Errors
Vector4d error;
Vector4d error_old;
Vector4d error_old_old;
Vector3d error_d;
Vector3d error_d_old;
Vector3d error_d_old_old;

int controller_type;

double alpha;

bool new_odometry;

class DNN{
        public:
          DNN(int, char**);
          ~DNN();
          void run();
        private:
          double denormalizeAngle(double a1, double a2);
          short sign(double x);
};


