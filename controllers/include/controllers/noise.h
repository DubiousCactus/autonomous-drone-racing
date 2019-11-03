#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setNoiseConfig.h>
#include <tf/transform_datatypes.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;

// Publishers
ros::Publisher noise_publisher;

double noise_position;
double noise_orientation;
double noise_velocity;
double noise_angular_velocity;

class Noise{
        public:
          Noise(int, char**);
          ~Noise();
          double generateGaussianNoise(double sigma);
          void run();
};
