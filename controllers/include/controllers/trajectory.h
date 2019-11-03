#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setTrajectoryConfig.h>
#include <Eigen/Dense>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::Vector4d;

// Publishers
ros::Publisher trajectory_publisher;
ros::Publisher velocity_publisher;
ros::Publisher pose_publisher;

// Pose
Vector4d pose_d;

// Trajectory type
int trajectory_type;

// Trajectory speed
double speed;

// Waypoints
MatrixXd waypoints;
int waypoint;

// Time
double t;

class Trajectory{
        public:
          Trajectory(int, char**);
          ~Trajectory();
          void run();
        private:
          double distance(Vector4d v1, Vector4d v2);
};
