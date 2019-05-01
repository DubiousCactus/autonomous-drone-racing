#include "controllers/odometry.h"

void push(double vx, double vy, double vz, double p, double q, double r){
    Twist t;
    t.linear.x = vx;
    t.linear.y = vy;
    t.linear.z = vz;
    t.angular.x = p;
    t.angular.y = q;
    t.angular.z = r;

    velocities.erase(velocities.begin());
    velocities.push_back(t);
}

/* Average filter */
Twist filter(){
    Twist t;
    t.linear.x = 0;
    t.linear.y = 0;
    t.linear.z = 0;
    t.angular.x = 0;
    t.angular.y = 0;
    t.angular.z = 0;
    for(vector<Twist>::iterator i = velocities.begin(); i != velocities.end(); ++i){
        t.linear.x += i->linear.x;
        t.linear.y += i->linear.y;
        t.linear.z += i->linear.z;
        t.angular.x += i->angular.x;
        t.angular.y += i->angular.y;
        t.angular.z += i->angular.z;
    }
    t.linear.x /= 10;
    t.linear.y /= 10;
    t.linear.z /= 10;
    t.angular.x /= 10;
    t.angular.y /= 10;
    t.angular.z /= 10;
    return t;
}

void optitrackCallback(const geometry_msgs::PoseStamped& optitrack_msg){
    Duration dt = optitrack_msg.header.stamp - time_old;

    odometry_optitrack.header.seq = optitrack_msg.header.seq;
    odometry_optitrack.header.stamp = optitrack_msg.header.stamp;
    odometry_optitrack.header.frame_id = optitrack_msg.header.frame_id;
    odometry_optitrack.child_frame_id = "bebop";

    odometry_optitrack.pose.pose.position.x = -optitrack_msg.pose.position.x;
    odometry_optitrack.pose.pose.position.y = -optitrack_msg.pose.position.y;
    odometry_optitrack.pose.pose.position.z = optitrack_msg.pose.position.z;
    odometry_optitrack.pose.pose.orientation = optitrack_msg.pose.orientation;

    tf::Quaternion q(optitrack_msg.pose.orientation.x, optitrack_msg.pose.orientation.y, optitrack_msg.pose.orientation.z, optitrack_msg.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double x, y, z, roll, pitch;
    m.getEulerZYX(z, y, x);
    roll = x;
    pitch = y;
    yaw = z;

    double vx = (odometry_optitrack.pose.pose.position.x - position.x) / (dt.toNSec() / pow(10, 9));
    double vy = (odometry_optitrack.pose.pose.position.y - position.y) / (dt.toNSec() / pow(10, 9));
    double vz = (odometry_optitrack.pose.pose.position.z - position.z) / (dt.toNSec() / pow(10, 9));
    double omega_x = (roll - orientation.x) / (dt.toNSec() / pow(10, 9));
    double omega_y = (pitch - orientation.y) / (dt.toNSec() / pow(10, 9));
    double omega_z = (yaw - orientation.z) / (dt.toNSec() / pow(10, 9));
    push(vx, vy, vz, omega_x, omega_y, omega_z);
    odometry_optitrack.twist.twist = filter();

    if((Time::now() - odometry_optitrack.header.stamp).operator <(Duration(0.5))){
        optitrack_ok = false;//true;
        odometry_ok = false;//true;
    }
}

void arucoCallback(const aruco_mapping::ArucoMarkerConstPtr& aruco_msg){
    if(pose_initial_aruco.orientation.w == 0)
        pose_initial_aruco = aruco_msg->global_camera_pose;

    odometry_aruco.header = aruco_msg->header;
    odometry_aruco.pose.pose = aruco_msg->global_camera_pose;

    odometry_aruco.pose.pose.position.x -= pose_initial_aruco.position.x;
    odometry_aruco.pose.pose.position.y -= pose_initial_aruco.position.y;
    odometry_aruco.pose.pose.position.z -= pose_initial_aruco.position.z;
    /*odometry_aruco.pose.pose.orientation.x -= pose_initial_aruco.orientation.x;
    odometry_aruco.pose.pose.orientation.y -= pose_initial_aruco.orientation.y;
    odometry_aruco.pose.pose.orientation.z -= pose_initial_aruco.orientation.z;
    odometry_aruco.pose.pose.orientation.w -= pose_initial_aruco.orientation.w;*/

    aruco_ok = aruco_msg->marker_visibile;
    odometry_ok = aruco_msg->marker_visibile;
}

void gpsCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    if(pose_initioal_gps.orientation.w == 0)
        pose_initioal_gps = odometry_msg->pose.pose;

    odometry_gps = *odometry_msg;

    odometry_gps.pose.pose.position.x -= pose_initioal_gps.position.x;
    odometry_gps.pose.pose.position.y -= pose_initioal_gps.position.y;
    odometry_gps.pose.pose.position.z -= pose_initioal_gps.position.z;

    gps_ok = true;
    odometry_ok = true;
}

void bebopCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    odometry_bebop = *odometry_msg;

    if(odometry_ok)
        pose_initial_bebop = odometry_msg->pose.pose;

    odometry_bebop.pose.pose.position.x += (pose_last.position.x - pose_initial_bebop.position.x);
    odometry_bebop.pose.pose.position.y += (pose_last.position.y - pose_initial_bebop.position.y);
    odometry_bebop.pose.pose.position.z += (pose_last.position.z - pose_initial_bebop.position.z);
    /*odometry_bebop.pose.pose.orientation.x += (pose_last.orientation.x - pose_initial_bebop.orientation.x);
    odometry_bebop.pose.pose.orientation.y += (pose_last.orientation.y - pose_initial_bebop.orientation.y);
    odometry_bebop.pose.pose.orientation.z += (pose_last.orientation.z - pose_initial_bebop.orientation.z);
    odometry_bebop.pose.pose.orientation.w += (pose_last.orientation.w - pose_initial_bebop.orientation.w);*/

    /*tf::Quaternion q(odometry_msg->pose.pose.orientation.x, odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z, odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double x, y, z;
    m.getEulerZYX(z, y, x);
    cout << "z = " << z / M_PI * 180 << endl;*/

    bebop_ok = true;
    odometry_ok = false;
}

void noiseCallback(const nav_msgs::OdometryConstPtr& noise_msg){
    noise = *noise_msg;
}

// Constructor
Odometry::Odometry(int argc, char** argv){
    ros::init(argc, argv, "Odometry");
    ros::NodeHandle node_handle;

    optitrack_subscriber = node_handle.subscribe("/optitrack/odometry", 1, optitrackCallback);
    gps_subscriber = node_handle.subscribe("/gps/odometry", 1, gpsCallback);
    aruco_subscriber = node_handle.subscribe("/aruco/odometry", 1, arucoCallback);
    bebop_subscriber = node_handle.subscribe("/bebop/odom", 1, bebopCallback);
    noise_subscriber = node_handle.subscribe("/bebop/noise", 1, noiseCallback);

    odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/bebop/ground_truth/odometry", 1);
    noisy_odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/bebop/odometry", 1);

    position.x = 0;
    position.y = 0;
    position.z = 0;

    yaw = 0;

    land = false;

    optitrack_ok = false;
    aruco_ok = false;
    gps_ok = false;
    bebop_ok = false;
    odometry_ok = true;

    Twist t;
    velocities.clear();
    for(int i = 0; i < 10; ++i)
        velocities.push_back(t);
}

// Destructor
Odometry::~Odometry(){
    ros::shutdown();
    exit(0);
}

void Odometry::run(){
    double dt = (double)1/100;
    ros::Rate rate(100);
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        if(optitrack_ok){
            ROS_INFO("[Odometry] optitrack_ok");
            pose_last = odometry_optitrack.pose.pose;
            odometry_publisher.publish(odometry_optitrack);
            noisy_odometry_publisher.publish(odometry_optitrack);
        }else{
            if(aruco_ok){
                ROS_INFO("[Odometry] aruco_ok");
                pose_last = odometry_aruco.pose.pose;
                odometry_publisher.publish(odometry_aruco);
                noisy_odometry_publisher.publish(odometry_aruco);
            }else{
                if(gps_ok){
                    ROS_INFO("[Odometry] gps_ok");
                    pose_last = odometry_gps.pose.pose;
                    odometry_publisher.publish(odometry_gps);
                    noisy_odometry_publisher.publish(odometry_gps);
                }else{
                    if(bebop_ok){
                        ROS_INFO("[Odometry] bebop_ok");
                        odometry_publisher.publish(odometry_bebop);
                        noisy_odometry_publisher.publish(odometry_bebop);
                    }
                }
            }
        }
        optitrack_ok = false;
        aruco_ok = false;
        gps_ok = false;
        bebop_ok = false;
    }
}

int main(int argc, char** argv){
    cout << "[Odometry] Odometry is running..." << endl;

    Odometry* odometry = new Odometry(argc, argv);

    odometry->run();
}
