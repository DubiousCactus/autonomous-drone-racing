#include "controllers/safeBebop.h"

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
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.seq = optitrack_msg.header.seq;
    odometry_msg.header.stamp = optitrack_msg.header.stamp;
    odometry_msg.header.frame_id = optitrack_msg.header.frame_id;
    odometry_msg.child_frame_id = "bebop";

    odometry_msg.pose.pose.position.x = optitrack_msg.pose.position.x;
    odometry_msg.pose.pose.position.y = optitrack_msg.pose.position.y;
    odometry_msg.pose.pose.position.z = optitrack_msg.pose.position.z;

    tf::Quaternion q(optitrack_msg.pose.orientation.x, optitrack_msg.pose.orientation.y, optitrack_msg.pose.orientation.z, optitrack_msg.pose.orientation.w);
    //q.normalize();
    tf::Matrix3x3 m(q);
    double x, y, z, roll, pitch;
    m.getEulerZYX(z, y, x);
    roll = x;
    pitch = y;
    yaw = z;
    //q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    odometry_msg.pose.pose.orientation = optitrack_msg.pose.orientation;

    double vx = (odometry_msg.pose.pose.position.x - position.x) / (dt.toNSec() / pow(10, 9));
    double vy = (odometry_msg.pose.pose.position.y - position.y) / (dt.toNSec() / pow(10, 9));
    double vz = (odometry_msg.pose.pose.position.z - position.z) / (dt.toNSec() / pow(10, 9));
    double omega_x = (roll - orientation.x) / (dt.toNSec() / pow(10, 9));
    double omega_y = (pitch - orientation.y) / (dt.toNSec() / pow(10, 9));
    double omega_z = (yaw - orientation.z) / (dt.toNSec() / pow(10, 9));
    push(vx, vy, vz, omega_x, omega_y, omega_z);
    odometry_msg.twist.twist = filter();
    odometry_publisher.publish(odometry_msg);

    tf::Quaternion qc(noise.pose.pose.orientation.x, noise.pose.pose.orientation.y, noise.pose.pose.orientation.z, noise.pose.pose.orientation.w);
    tf::Quaternion rqc = q * qc;
    odometry_msg.pose.pose.position.x += noise.pose.pose.position.x;
    odometry_msg.pose.pose.position.y += noise.pose.pose.position.y;
    odometry_msg.pose.pose.position.z += noise.pose.pose.position.z;
    odometry_msg.pose.pose.orientation.x = rqc.getX(); // DO NOT WORK
    odometry_msg.pose.pose.orientation.y = rqc.getY(); // DO NOT WORK
    odometry_msg.pose.pose.orientation.z = rqc.getZ(); // DO NOT WORK
    odometry_msg.pose.pose.orientation.w = rqc.getW(); // DO NOT WORK
    odometry_msg.twist.twist.linear.x += odometry_msg.twist.twist.linear.x;
    odometry_msg.twist.twist.linear.y += odometry_msg.twist.twist.linear.y;
    odometry_msg.twist.twist.linear.z += odometry_msg.twist.twist.linear.z;
    odometry_msg.twist.twist.angular.x += odometry_msg.twist.twist.angular.x;
    odometry_msg.twist.twist.angular.y += odometry_msg.twist.twist.angular.y;
    odometry_msg.twist.twist.angular.z += odometry_msg.twist.twist.angular.z;
    noisy_odometry_publisher.publish(odometry_msg);

    time_old = odometry_msg.header.stamp;
    position = odometry_msg.pose.pose.position;
    orientation.x = roll;
    orientation.y = pitch;
    orientation.z = yaw;

    velocity << vx, vy, vz, omega_z;

    marker_visibile = true;

    //cout << "[SafeBebop] yaw = " << yaw << endl;
    /*tf::Quaternion q1(odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    m1.getEulerZYX(z, y, x);
    cout << "[SafeBebop]:" << endl;
    cout << "x = " << odometry_msg.pose.pose.position.x << ",\t y =  " << odometry_msg.pose.pose.position.y << ",\t z = " << odometry_msg.pose.pose.position.z << endl;
    cout << "roll = " << x << ",\t pitch =  " << y << ",\t yaw = " << z << endl;
    cout << "vx = " << odometry_msg.twist.twist.linear.x << ",\t vy =  " << odometry_msg.twist.twist.linear.y << ",\t vz = " << odometry_msg.twist.twist.linear.z << endl;
    cout << "p = " << odometry_msg.twist.twist.angular.x << ",\t q =  " << odometry_msg.twist.twist.angular.y << ",\t r = " << odometry_msg.twist.twist.angular.z << endl;*/
}

void viconCallback(const geometry_msgs::TransformStamped& vicon_msg){
    Duration dt = vicon_msg.header.stamp - time_old;
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.seq = vicon_msg.header.seq;
    odometry_msg.header.stamp = vicon_msg.header.stamp;
    odometry_msg.header.frame_id = vicon_msg.header.frame_id;
    odometry_msg.child_frame_id = "bebop";

    odometry_msg.pose.pose.position.x = vicon_msg.transform.translation.x;
    odometry_msg.pose.pose.position.y = vicon_msg.transform.translation.y;
    odometry_msg.pose.pose.position.z = vicon_msg.transform.translation.z;

    tf::Quaternion q(vicon_msg.transform.rotation.x, vicon_msg.transform.rotation.y, vicon_msg.transform.rotation.z, vicon_msg.transform.rotation.w);
    //q.normalize();
    tf::Matrix3x3 m(q);
    double x, y, z, roll, pitch;
    m.getEulerZYX(z, y, x);
    roll = x;
    pitch = y;
    yaw = z;
    //q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    odometry_msg.pose.pose.orientation = vicon_msg.transform.rotation;

    double vx = (odometry_msg.pose.pose.position.x - position.x) / (dt.toNSec() / pow(10, 9));
    double vy = (odometry_msg.pose.pose.position.y - position.y) / (dt.toNSec() / pow(10, 9));
    double vz = (odometry_msg.pose.pose.position.z - position.z) / (dt.toNSec() / pow(10, 9));
    double omega_x = (roll - orientation.x) / (dt.toNSec() / pow(10, 9));
    double omega_y = (pitch - orientation.y) / (dt.toNSec() / pow(10, 9));
    double omega_z = (yaw - orientation.z) / (dt.toNSec() / pow(10, 9));
    push(vx, vy, vz, omega_x, omega_y, omega_z);
    odometry_msg.twist.twist = filter();
    odometry_publisher.publish(odometry_msg);

    tf::Quaternion qc(noise.pose.pose.orientation.x, noise.pose.pose.orientation.y, noise.pose.pose.orientation.z, noise.pose.pose.orientation.w);
    tf::Quaternion rqc = q * qc;
    odometry_msg.pose.pose.position.x += noise.pose.pose.position.x;
    odometry_msg.pose.pose.position.y += noise.pose.pose.position.y;
    odometry_msg.pose.pose.position.z += noise.pose.pose.position.z;
    odometry_msg.pose.pose.orientation.x = rqc.getX(); // DO NOT WORK
    odometry_msg.pose.pose.orientation.y = rqc.getY(); // DO NOT WORK
    odometry_msg.pose.pose.orientation.z = rqc.getZ(); // DO NOT WORK
    odometry_msg.pose.pose.orientation.w = rqc.getW(); // DO NOT WORK
    odometry_msg.twist.twist.linear.x += odometry_msg.twist.twist.linear.x;
    odometry_msg.twist.twist.linear.y += odometry_msg.twist.twist.linear.y;
    odometry_msg.twist.twist.linear.z += odometry_msg.twist.twist.linear.z;
    odometry_msg.twist.twist.angular.x += odometry_msg.twist.twist.angular.x;
    odometry_msg.twist.twist.angular.y += odometry_msg.twist.twist.angular.y;
    odometry_msg.twist.twist.angular.z += odometry_msg.twist.twist.angular.z;
    //noisy_odometry_publisher.publish(odometry_msg);

    time_old = odometry_msg.header.stamp;
    position = odometry_msg.pose.pose.position;
    orientation.x = roll;
    orientation.y = pitch;
    orientation.z = yaw;

    velocity << vx, vy, vz, omega_z;

    marker_visibile = true;

    geometry_msgs::QuaternionStamped pose;
    pose.header = vicon_msg.header;
    pose.quaternion.x = vicon_msg.transform.translation.x;
    pose.quaternion.y = vicon_msg.transform.translation.y;
    pose.quaternion.z = vicon_msg.transform.translation.z;
    pose.quaternion.w = yaw / M_PI * 180;
    pose_publisher.publish(pose);

    //cout << "[SafeBebop] yaw = " << yaw << endl;
    /*tf::Quaternion q1(odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    m1.getEulerZYX(z, y, x);
    cout << "[SafeBebop]:" << endl;
    cout << "x = " << odometry_msg.pose.pose.position.x << ",\t y =  " << odometry_msg.pose.pose.position.y << ",\t z = " << odometry_msg.pose.pose.position.z << endl;
    cout << "roll = " << x << ",\t pitch =  " << y << ",\t yaw = " << z << endl;
    cout << "vx = " << odometry_msg.twist.twist.linear.x << ",\t vy =  " << odometry_msg.twist.twist.linear.y << ",\t vz = " << odometry_msg.twist.twist.linear.z << endl;
    cout << "p = " << odometry_msg.twist.twist.angular.x << ",\t q =  " << odometry_msg.twist.twist.angular.y << ",\t r = " << odometry_msg.twist.twist.angular.z << endl;*/
}


void arucoCallback(const aruco_mapping::ArucoMarker& aruco_msg){
    marker_visibile = aruco_msg.marker_visibile;
    if(aruco_msg.marker_visibile){
        Duration dt = aruco_msg.header.stamp - time_old;
        nav_msgs::Odometry odometry_msg;
        odometry_msg.header.seq = aruco_msg.header.seq;
        odometry_msg.header.stamp = aruco_msg.header.stamp;
        odometry_msg.header.frame_id = aruco_msg.header.frame_id;
        odometry_msg.child_frame_id = "bebop";

        odometry_msg.pose.pose.position.x = -aruco_msg.global_camera_pose.position.z;
        odometry_msg.pose.pose.position.y = aruco_msg.global_camera_pose.position.y;
        odometry_msg.pose.pose.position.z = aruco_msg.global_camera_pose.position.x;

        tf::Quaternion q(aruco_msg.global_camera_pose.orientation.x, aruco_msg.global_camera_pose.orientation.y, aruco_msg.global_camera_pose.orientation.z, aruco_msg.global_camera_pose.orientation.w);
        //q.normalize();
        tf::Matrix3x3 m(q);
        double x, y, z, roll, pitch;
        m.getEulerZYX(z, y, x);
        roll = x;
        pitch = y;
        yaw = z;
        //q = tf::createQuaternionFromRPY(roll, pitch, yaw);
        odometry_msg.pose.pose.orientation = aruco_msg.global_camera_pose.orientation;

        // Computes linear and angular velocities by the numerical derivation and filtering
        double vx = (odometry_msg.pose.pose.position.x - position.x) / (dt.toNSec() / pow(10, 9));
        double vy = (odometry_msg.pose.pose.position.y - position.y) / (dt.toNSec() / pow(10, 9));
        double vz = (odometry_msg.pose.pose.position.z - position.z) / (dt.toNSec() / pow(10, 9));
        double omega_x = (roll - orientation.x) / (dt.toNSec() / pow(10, 9));
        double omega_y = (pitch - orientation.y) / (dt.toNSec() / pow(10, 9));
        double omega_z = (yaw - orientation.z) / (dt.toNSec() / pow(10, 9));
        push(vx, vy, vz, omega_x, omega_y, omega_z);
        odometry_msg.twist.twist = filter();
        noisy_odometry_publisher.publish(odometry_msg);

        time_old = odometry_msg.header.stamp;
        position = odometry_msg.pose.pose.position;
        orientation.x = roll;
        orientation.y = pitch;
        orientation.z = yaw;

        velocity << vx, vy, vz, omega_z;
    }
}

void vslamCallback(const geometry_msgs::PoseWithCovarianceStamped pose_msg){
    if(!vslam_initialased){
        vslam_initial_position.x = -pose_msg.pose.pose.position.z;
        vslam_initial_position.y = pose_msg.pose.pose.position.x;
        vslam_initial_position.z = pose_msg.pose.pose.position.y;
        vslam_initial_yaw = yaw;
        vslam_initialased = true;
    }

    Duration dt = pose_msg.header.stamp - time_old;
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.seq = pose_msg.header.seq;
    odometry_msg.header.stamp = pose_msg.header.stamp;
    odometry_msg.header.frame_id = pose_msg.header.frame_id;
    odometry_msg.child_frame_id = "bebop";

    tf::Quaternion q(pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w);
    //q.normalize();
    tf::Matrix3x3 m(q);
    double x, y, z, roll, pitch;
    m.getEulerZYX(z, y, x);
    roll = x;
    pitch = y;
    //yaw = z;

    // cout << "[SafeBebop] current_yaw = " << (yaw - vslam_initial_yaw) / M_PI * 180 << ", \t yaw_bebop = " << yaw / M_PI * 180 << ", \t yaw_vslam = " << z / M_PI * 180 << endl;

    double current_x = -pose_msg.pose.pose.position.z;
    double current_y = pose_msg.pose.pose.position.x;
    double current_z = pose_msg.pose.pose.position.y;
    double current_yaw = 0;//yaw - vslam_initial_yaw;
    double rotated_x = current_x * cos(current_yaw) + current_y * sin(current_yaw);
    double rotated_y = -current_x * sin(current_yaw) + current_y * cos(current_yaw);
    double rotated_z = current_z;

    //cout << "[SafeBebop] current_x = " << current_x << ", \t rotated_x = " << rotated_x << endl;
    //cout << "[SafeBebop] current_y = " << current_y << ", \t rotated_y = " << rotated_y << endl;


    odometry_msg.pose.pose.position.x = rotated_x - vslam_initial_position.x;
    odometry_msg.pose.pose.position.y = rotated_y - vslam_initial_position.y;
    odometry_msg.pose.pose.position.z = rotated_z - vslam_initial_position.z;

    //q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    odometry_msg.pose.pose.orientation = pose_msg.pose.pose.orientation;

    // Computes linear and angular velocities by the numerical derivation and filtering
    /*double vx = (odometry_msg.pose.pose.position.x - position.x) / (dt.toNSec() / pow(10, 9));
    double vy = (odometry_msg.pose.pose.position.y - position.y) / (dt.toNSec() / pow(10, 9));
    double vz = (odometry_msg.pose.pose.position.z - position.z) / (dt.toNSec() / pow(10, 9));
    double omega_x = (roll - orientation.x) / (dt.toNSec() / pow(10, 9));
    double omega_y = (pitch - orientation.y) / (dt.toNSec() / pow(10, 9));
    double omega_z = (yaw - orientation.z) / (dt.toNSec() / pow(10, 9));
    push(vx, vy, vz, omega_x, omega_y, omega_z);
    odometry_msg.twist.twist = filter();*/
    //noisy_odometry_publisher.publish(odometry_msg);

    /*time_old = odometry_msg.header.stamp;
    position = odometry_msg.pose.pose.position;
    orientation.x = roll;
    orientation.y = pitch;
    orientation.z = yaw;*/

    //velocity << vx, vy, vz, omega_z;
}

void vslamInfoCallback(const ptam_com::ptam_info info_msg){
    marker_visibile = info_msg.mapQuality; // CHECK info_msg.trackingQuality !!!
}

void odometryCallback(const nav_msgs::Odometry odometry_msg){
    tf::Quaternion q(odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double x, y, z;
    m.getEulerZYX(z, y, x);
    yaw = z;

    Duration dt = odometry_msg.header.stamp - time_old;

    position.x = odometry_msg.pose.pose.position.x;
    position.y = odometry_msg.pose.pose.position.y;
    position.z = odometry_msg.pose.pose.position.z;

    double velocity_x = cos(yaw) * odometry_msg.twist.twist.linear.x - sin(yaw) * odometry_msg.twist.twist.linear.y;
    double velocity_y = sin(yaw) * odometry_msg.twist.twist.linear.x + cos(yaw) * odometry_msg.twist.twist.linear.y;

    velocity << velocity_x, velocity_y, odometry_msg.twist.twist.linear.z, 0;

    nav_msgs::Odometry odometry;
    odometry = odometry_msg;
    odometry.pose.pose.position.x = position.x;
    odometry.pose.pose.position.y = position.y;
    odometry.pose.pose.position.z = position.z;
    odometry.twist.twist.linear.x = velocity_x;
    odometry.twist.twist.linear.y = velocity_y;
    odometry_publisher.publish(odometry);

    sensor_msgs::Imu imu;
    imu.header = odometry_msg.header;
    imu.orientation = odometry_msg.pose.pose.orientation;
    /*imu.orientation.x = 0;
    imu.orientation.y = 0;
    imu.orientation.z = 0;
    imu.orientation.w = 1;*/
    imu.orientation_covariance[0] = odometry_msg.pose.covariance[21];
    imu.orientation_covariance[1] = odometry_msg.pose.covariance[22];
    imu.orientation_covariance[2] = odometry_msg.pose.covariance[23];
    imu.orientation_covariance[3] = odometry_msg.pose.covariance[27];
    imu.orientation_covariance[4] = odometry_msg.pose.covariance[28];
    imu.orientation_covariance[5] = odometry_msg.pose.covariance[29];
    imu.orientation_covariance[6] = odometry_msg.pose.covariance[33];
    imu.orientation_covariance[7] = odometry_msg.pose.covariance[34];
    imu.orientation_covariance[8] = odometry_msg.pose.covariance[35];
    imu.angular_velocity = odometry_msg.twist.twist.angular;
    imu.angular_velocity_covariance[0] = odometry_msg.twist.covariance[0];
    imu.angular_velocity_covariance[1] = odometry_msg.twist.covariance[1];
    imu.angular_velocity_covariance[2] = odometry_msg.twist.covariance[2];
    imu.angular_velocity_covariance[3] = odometry_msg.twist.covariance[6];
    imu.angular_velocity_covariance[4] = odometry_msg.twist.covariance[7];
    imu.angular_velocity_covariance[5] = odometry_msg.twist.covariance[8];
    imu.angular_velocity_covariance[6] = odometry_msg.twist.covariance[12];
    imu.angular_velocity_covariance[7] = odometry_msg.twist.covariance[13];
    imu.angular_velocity_covariance[8] = odometry_msg.twist.covariance[14];
    imu.linear_acceleration.x = velocity(0) - velocity_old(0) / dt.toSec();
    imu.linear_acceleration.y = velocity(1) - velocity_old(1) / dt.toSec();
    imu.linear_acceleration.z = velocity(2) - velocity_old(2) / dt.toSec();
    imu.linear_acceleration_covariance[0] = odometry_msg.twist.covariance[21];
    imu.linear_acceleration_covariance[1] = odometry_msg.twist.covariance[22];
    imu.linear_acceleration_covariance[2] = odometry_msg.twist.covariance[23];
    imu.linear_acceleration_covariance[3] = odometry_msg.twist.covariance[27];
    imu.linear_acceleration_covariance[4] = odometry_msg.twist.covariance[28];
    imu.linear_acceleration_covariance[5] = odometry_msg.twist.covariance[29];
    imu.linear_acceleration_covariance[6] = odometry_msg.twist.covariance[33];
    imu.linear_acceleration_covariance[7] = odometry_msg.twist.covariance[34];
    imu.linear_acceleration_covariance[8] = odometry_msg.twist.covariance[35];
    imu_publisher.publish(imu);

    velocity_old = velocity;
    time_old = odometry_msg.header.stamp;

    marker_visibile = true;

    //ros::param::set("camera_pitch", y);
}

void altitudeCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChanged altitude_msg){
    //position.z = altitude_msg.altitude;

    //ros::param::set("camera_height", altitude_msg.altitude);
}

void batteryCallback(const bebop_msgs::CommonCommonStateBatteryStateChanged battery_msg){
    battery = battery_msg.percent;


    if(battery <= 10){
        cout << "***** BATTERY: " << battery << endl;
        cout << "***** BATTERY: " << battery << endl;
        cout << "***** BATTERY: " << battery << endl;
    }
    else
        cout << "***** Battery: " << battery << endl;

    //ros::param::set("camera_height", altitude_msg.altitude);
}

void gpsCallback(const nav_msgs::Odometry gps_msg){
    nav_msgs::Odometry odometry_msg;
    odometry_msg = gps_msg;
    odometry_publisher.publish(odometry_msg);

    tf::Quaternion q(gps_msg.pose.pose.orientation.x, gps_msg.pose.pose.orientation.y, gps_msg.pose.pose.orientation.z, gps_msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getEulerZYX(yaw, pitch, roll);

    orientation.x = roll;
    orientation.y = pitch;
    orientation.z = yaw;

    velocity << gps_msg.twist.twist.linear.x, gps_msg.twist.twist.linear.y, gps_msg.twist.twist.linear.z, gps_msg.twist.twist.angular.z;

    marker_visibile = true;
}

float bound(float v, float b){
    if(v < -b)
        return -b;
    if(v > b)
        return b;
    return v;
}

void commandCallback(const std_msgs::Int8& command_msg){
    std_msgs::Empty empty_msg;
    geometry_msgs::Twist velocity_msg;
    switch(command_msg.data){
    case 1:
        velocity_msg.linear.x = 0;
        velocity_msg.linear.y = 0;
        velocity_msg.linear.z = 0;
        velocity_msg.angular.x = 0;
        velocity_msg.angular.y = 0;
        velocity_msg.angular.z = 0;
        velocity_publisher.publish(velocity_msg);
        velocity_publisher.publish(velocity_msg);
        land = true;
        break;
    case 2:
        takeoff_publisher.publish(empty_msg);
        takeoff_publisher.publish(empty_msg);
        Duration(5).sleep();
        land = false;
        break;
    case 3:
        velocity_msg.linear.x = 0;
        velocity_msg.linear.y = 0;
        velocity_msg.linear.z = 0;
        velocity_msg.angular.x = 0;
        velocity_msg.angular.y = 0;
        velocity_msg.angular.z = 0;
        velocity_publisher.publish(velocity_msg);
        velocity_publisher.publish(velocity_msg);
        land_publisher.publish(empty_msg);
        land_publisher.publish(empty_msg);

        land = true;
        velocity << 0, 0, 0, 0;
        velocity_d << 0, 0, 0, 0;
        error_i << 0, 0, 0, 0;
        yaw = 0;
        marker_visibile = false;

        break;
    case 4:
        reset_publisher.publish(empty_msg);
        reset_publisher.publish(empty_msg);
    }
}

void commandVelocityCallback(const geometry_msgs::Quaternion& command_msg){   
    float vx = bound(command_msg.x, MAX_V) / ((double)MAX_TILT * M_PI / 180);
    float vy = bound(command_msg.y, MAX_V) / ((double)MAX_TILT * M_PI / 180);
    float vz = bound(command_msg.z, MAX_V);
    float vw = bound(command_msg.w / M_PI * 180, MAX_ROTATION) * M_PI / 180;

    velocity_d << vx, vy, vz, vw;
}

void commandCameraCallback(const geometry_msgs::Vector3& command_msg){
    geometry_msgs::Twist camera_msg;
    camera_msg.linear.x = 0;
    camera_msg.linear.y = 0;
    camera_msg.linear.z = 0;
    camera_msg.angular.x = 0;
    camera_msg.angular.y = command_msg.y;
    camera_msg.angular.z = command_msg.z;
    camera_publisher.publish(camera_msg);
}

void noiseCallback(const nav_msgs::OdometryConstPtr& noise_msg){
    noise = *noise_msg;
}

// Constructor
SafeBebop::SafeBebop(int argc, char** argv){
    ros::init(argc, argv, "safeBebop");
    ros::NodeHandle node_handle;

    //optitrack_subscriber = node_handle.subscribe("/optitrack/odometry", 1, optitrackCallback);
    vicon_subscriber = node_handle.subscribe("/vicon/bebop/bebop", 1, viconCallback);
    //aruco_subscriber = node_handle.subscribe("/aruco/odometry", 1, arucoCallback);
    //odometry_subscriber = node_handle.subscribe("/bebop/odom", 1, odometryCallback);
    //vslam_subscriber = node_handle.subscribe("/vslam/pose", 1, vslamCallback);
    //vslam_info_subscriber = node_handle.subscribe("/vslam/info", 1, vslamInfoCallback);
    //gps_subscriber = node_handle.subscribe("/odometry/gps", 1, gpsCallback);
    command_subscriber = node_handle.subscribe("/bebop/command", 1, commandCallback);
    command_velocity_subscriber = node_handle.subscribe("/bebop/command_velocity", 1, commandVelocityCallback);
    command_camera_subscriber = node_handle.subscribe("/bebop/command_camera", 1, commandCameraCallback);
    noise_subscriber = node_handle.subscribe("/bebop/noise", 1, noiseCallback);
    //altitude_subscriber = node_handle.subscribe("/bebop/states/ardrone3/PilotingState/AltitudeChanged", 1, altitudeCallback);
    battery_subscriber = node_handle.subscribe("/bebop/states/common/CommonState/BatteryStateChanged", 1, batteryCallback);

    reset_publisher = node_handle.advertise<std_msgs::Empty>("/bebop/reset", 1);
    takeoff_publisher = node_handle.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
    land_publisher = node_handle.advertise<std_msgs::Empty>("/bebop/land", 1);
    velocity_publisher = node_handle.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
    camera_publisher = node_handle.advertise<geometry_msgs::Twist>("/bebop/camera_control", 1);
    odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/bebop/odometry", 1);
    noisy_odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/bebop/odometry_noisy", 1);
    pose_publisher = node_handle.advertise<geometry_msgs::QuaternionStamped>("/bebop/pose", 1);
    imu_publisher = node_handle.advertise<sensor_msgs::Imu>("/bebop/imu", 1);

    position.x = 0;
    position.y = 0;
    position.z = 0;

    k_p << 0.2, 0.2, 0.6, 0.9;
    k_i << 0.5, 0.5, 0.0, 0.0;
    k_d << 0.0, 0.0, 0.0, 0.0;
    velocity << 0, 0, 0, 0;
    velocity_old << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;
    error_i << 0, 0, 0, 0;

    yaw = 0;

    land = true;
    marker_visibile = false;

    Twist t;
    velocities.clear();
    for(int i = 0; i < 10; ++i)
        velocities.push_back(t);
}

// Destructor
SafeBebop::~SafeBebop(){
    std_msgs::Empty empty_msg;
    geometry_msgs::Twist velocity_msg;
    velocity_msg.linear.x = 0;
    velocity_msg.linear.y = 0;
    velocity_msg.linear.z = 0;
    velocity_msg.angular.x = 0;
    velocity_msg.angular.y = 0;
    velocity_msg.angular.z = 0;
    velocity_publisher.publish(velocity_msg);
    velocity_publisher.publish(velocity_msg);
    land_publisher.publish(empty_msg);
    land_publisher.publish(empty_msg);
    land = true;

    ros::shutdown();
    exit(0);
}

void SafeBebop::run(){
    double dt = (double)1/100;
    ros::Rate rate(100);

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        geometry_msgs::Twist velocity_msg;
        if(position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > -MAX_Z
                && (velocity_d(0) != 0 || velocity_d(1) != 0 || velocity_d(2) != 0 || velocity_d(3) != 0) && marker_visibile){

            error = velocity_d - velocity;
            //error = velocity_d; // FOR DEMO

            error_i += error * dt;
            error_i << bound(error_i(0), MAX_I), bound(error_i(1), MAX_I), bound(error_i(2), MAX_I), bound(error_i(3), MAX_I);
            error_d = (error - error_old) / dt;
            error_old = error;

            double vx = bound(k_p(0) * error(0) + k_i(0) * error_i(0) + k_d(0) * error_d(0), 1);
            double vy = bound(k_p(1) * error(1) + k_i(1) * error_i(1) + k_d(1) * error_d(1), 1);
            double vz = bound(k_p(2) * error(2) + k_i(2) * error_i(2) + k_d(2) * error_d(2), 1);
            double r  = bound(k_p(3) * velocity_d(3), 1);

            //yaw = 0; // FOR DEMO

            velocity_msg.linear.x = cos(yaw) * vx + sin(yaw) * vy;
            velocity_msg.linear.y = -sin(yaw) * vx + cos(yaw) * vy;
            velocity_msg.linear.z = vz;
            velocity_msg.angular.x = 0.0;
            velocity_msg.angular.y = 0.0;
            velocity_msg.angular.z = r;
        }
        else{
            velocity_msg.linear.x = 0.0;
            velocity_msg.linear.y = 0.0;
            velocity_msg.linear.z = 0.0;
            velocity_msg.angular.x = 0.0;
            velocity_msg.angular.y = 0.0;
            velocity_msg.angular.z = 0.0;
        }
        if(!land)
            velocity_publisher.publish(velocity_msg);
    }
}

int main(int argc, char** argv){
    cout << "[SafeBebop] SafeBebop is running..." << endl;

    SafeBebop* bebop = new SafeBebop(argc, argv);

    bebop->run();
}
