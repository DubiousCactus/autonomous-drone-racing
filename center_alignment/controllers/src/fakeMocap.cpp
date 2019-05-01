#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;

// Publishers
Publisher mocap_publisher;

// Subsribers
Subscriber mocap_subscriber;

void mocapCallback(const nav_msgs::OdometryConstPtr& mocap_msg){
    geometry_msgs::PoseStamped pose;
    pose.header = mocap_msg->header;
    pose.pose = mocap_msg->pose.pose;
    mocap_publisher.publish(pose);
}

class FakeMocap{
public:
    // Constructor
    FakeMocap(int argc, char** argv){
        ros::init(argc, argv, "FakeMocap");
        ros::NodeHandle node_handle;

        mocap_subscriber = node_handle.subscribe("/in_odometry", 1, mocapCallback);

        mocap_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/out_odometry", 1);
    }

    // Destructor
    ~FakeMocap(){
        ros::shutdown();
        exit(0);
    }

    void run(){
        ros::Rate rate(100);
        while(ros::ok()){
            rate.sleep();
            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv){
    cout << "[FakeMocap] FakeMocap is running..." << endl;

    FakeMocap* fm = new FakeMocap(argc, argv);

    fm->run();
}
