#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/package.h>
#include <Eigen/Dense>

using namespace geometry_msgs;
using namespace std;

using Eigen::Matrix3d;
using Eigen::Vector3d;

class SE3 {
public:
    SE3(int argc, char** argv);

    void run();
    void setGains(double kx, double kv, double kR, double kOmega);

private:
    double m;
    double g;
    Matrix3d I;

    Vector3d v;
    Vector3d vd;
    Matrix3d R;
    Matrix3d Rd;
    Vector3d Omega;
    Vector3d eOmega;
    Vector3d e3;
    Vector3d ev;
    Vector3d eR;
    double T;
    Vector3d torque;
    Vector3d x_vec;
    Vector3d y_vec;
    Vector3d z_vec;

    QuaternionStamped msg_command;

    ros::Subscriber velocity_subscriber;
    ros::Subscriber attitude_subscriber;
    ros::Subscriber angular_velocity_subscriber;
    ros::Subscriber desired_velocity_subscriber;

    ros::Publisher velocity_publisher;

    double kv;
    double kR;
    double kOmega;

    Vector3d veeMap(Matrix3d M);
    Matrix3d getRotationMatrix(double phi, double theta, double psi);

    void velocityCallback(const Vector3Stamped& msg);
    void attitudeCallback(const Vector3Stamped& msg);
    void angularVelocityCallback(const Vector3Stamped& msg);
    void desiredVelocityCallback(const Vector3Stamped& msg);
};
