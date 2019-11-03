#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/package.h>

//#include <vtol/DroneInputStamped.h>

using namespace geometry_msgs;
using namespace std;

#define MAP_HEIGHT 5
#define MAP_LENGTH 20
#define MAP_WIDTH 20
#define MAX_VELOCITY 1

// Subscribers
ros::Subscriber position_subscriber;
ros::Subscriber attitude_subscriber;

// Publishers
ros::Publisher velocity_publisher;

// Variables
Vector3 position;
Vector3 attitude;
Vector3 velocity;
Vector3 velocity_prev;
char policy[MAP_HEIGHT][MAP_LENGTH][MAP_WIDTH];

ofstream results;

class q_learning{
        public:
          q_learning(int, char**);
          ~q_learning();
          void run();
      
        private:        
            void setVelocity(double vx, double vy, double vz);
            Vector3 computeVelocity();
            void saveData();
};
