#include "controllers/safeY6.h"

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

void dynamicReconfigureCallback(controllers::setSafeY6Config &config, uint32_t level){
    controller = config.controller;
}

void optitrackCallback(const geometry_msgs::PoseStamped& optitrack_msg){
    Duration dt = optitrack_msg.header.stamp - time_old;
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.seq = optitrack_msg.header.seq;
    odometry_msg.header.stamp = optitrack_msg.header.stamp;
    odometry_msg.header.frame_id = optitrack_msg.header.frame_id;
    odometry_msg.child_frame_id = "Y6";

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

    //cout << "[SafeBebop] yaw = " << yaw << endl;
    /*tf::Quaternion q1(odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    m1.getEulerZYX(z, y, x);
    cout << "[SafeBebop]:" << endl;
    cout << "x = " << odometry_msg.pose.pose.position.x << ",\t y =  " << odometry_msg.pose.pose.position.y << ",\t z = " << odometry_msg.pose.pose.position.z << endl;
    cout << "roll = " << x << ",\t pitch =  " << y << ",\t yaw = " << z << endl;*/
    //cout << "vx = " << odometry_msg.twist.twist.linear.x << ",\t vy =  " << odometry_msg.twist.twist.linear.y << ",\t vz = " << odometry_msg.twist.twist.linear.z << endl;
    //cout << "p = " << odometry_msg.twist.twist.angular.x << ",\t q =  " << odometry_msg.twist.twist.angular.y << ",\t r = " << odometry_msg.twist.twist.angular.z << endl;
}

float bound(float v, float b){
    if(v < -b)
        return -b;
    if(v > b)
        return b;
    return v;
}

void commandCallback(const std_msgs::Int8& command_msg){
    geometry_msgs::TwistStamped velocity_msg;
	velocity_msg.header.stamp = ros::Time::now();;
    switch(command_msg.data){
    case 1: // hower

        break;
    case 2: // arm
    	offb_set_mode.request.custom_mode = "OFFBOARD";
		set_mode_client.call(offb_set_mode);
   		arm_cmd.request.value = true;
		arming_client.call(arm_cmd);
        stop = false;
        break;
    case 3: // land
    	offb_set_mode.request.custom_mode = "OFFBOARD";
		set_mode_client.call(offb_set_mode);
        land_cmd.request.yaw = yaw;
        land_cmd.request.latitude = position_d(0);
        land_cmd.request.longitude = position_d(1);
        land_cmd.request.altitude = 0;
		land_client.call(land_cmd);
        stop = true;
        break;
    case 4: // disarm
		offb_set_mode.request.custom_mode = "OFFBOARD";
		set_mode_client.call(offb_set_mode);
   		arm_cmd.request.value = false;
		arming_client.call(arm_cmd);
		stop = true;
    }
}

void commandPositionCallback(const geometry_msgs::QuaternionStamped& command_msg){
    float x = bound(command_msg.quaternion.x, MAX_X);
    float y = bound(command_msg.quaternion.y, MAX_Y);
    float z = bound(command_msg.quaternion.z, MAX_Z);
    float psi = bound(command_msg.quaternion.w, M_PI);

    position_d << x, y, z, psi;
}

void commandVelocityCallback(const geometry_msgs::Quaternion& command_msg){
    float vx = bound(command_msg.x, MAX_V);
    float vy = bound(command_msg.y, MAX_V);
    float vz = bound(command_msg.z, MAX_V);
    float r = bound(command_msg.w, MAX_V);

    velocity_d << vx, vy, vz, r;
}

void commandAttitudeCallback(const geometry_msgs::Quaternion& command_msg){
    /*float roll = bound(command_msg.x, MAX_RP);
    float pitch = bound(command_msg.y, MAX_RP);
    float yaw = bound(command_msg.z, M_PI);
    float thrust = bound(command_msg.w, 1);*/

    float roll = bound(command_msg.y, MAX_RP);
    float pitch = bound(command_msg.x, MAX_RP);
    float yaw = bound(command_msg.w, M_PI);
    float thrust = bound(command_msg.z, 1);

    attitude_d << roll, pitch, yaw, thrust;
}

void noiseCallback(const nav_msgs::OdometryConstPtr& noise_msg){
    noise = *noise_msg;
}

void batteryCallback(const sensor_msgs::BatteryStateConstPtr& battery_msg){
    if(battery_msg->percentage < 0.1){
        stop = true;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        set_mode_client.call(offb_set_mode);
        land_cmd.request.yaw = yaw;
        land_cmd.request.latitude = position_d(0);
        land_cmd.request.longitude = position_d(1);
        land_cmd.request.altitude = 0;
        land_client.call(land_cmd);
    }
}

// Constructor
SafeY6::SafeY6(int argc, char** argv){
    ros::init(argc, argv, "safeY6");
    ros::NodeHandle node_handle;

    optitrack_subscriber = node_handle.subscribe("/optitrack/odometry", 1, optitrackCallback);
    command_subscriber = node_handle.subscribe("/y6/command", 1, commandCallback);
    command_position_subscriber = node_handle.subscribe("/y6/command_position", 1, commandPositionCallback);
    command_velocity_subscriber = node_handle.subscribe("/y6/command_velocity", 1, commandVelocityCallback);
    command_attitude_subscriber = node_handle.subscribe("/y6/command_attitude", 1, commandAttitudeCallback);
    noise_subscriber = node_handle.subscribe("/y6/noise", 1, noiseCallback);
    battery_subscriber = node_handle.subscribe("/mavros/battery", 1, batteryCallback);

    position_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
    velocity_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
    attitude_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 1);
    throttle_publisher = node_handle.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 1);
    odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/y6/ground_truth/odometry", 1);
    noisy_odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/y6/odometry", 1);
	
    set_mode_client = node_handle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    arming_client = node_handle.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    land_client = node_handle.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    position.x = 0;
    position.y = 0;
    position.z = 0;

    position_d << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;
    attitude_d << 0, 0, 0, 0;

    yaw = 0;

    stop = true;

    controller = 0;

    Twist t;
    velocities.clear();
    for(int i = 0; i < 10; ++i)
        velocities.push_back(t);
}

// Destructor
SafeY6::~SafeY6(){
    stop = true;

    ros::Rate rate(10);
    rate.sleep();

    ros::shutdown();
    exit(0);
}

void SafeY6::run(){
    dynamic_reconfigure::Server<controllers::setSafeY6Config> server;
    dynamic_reconfigure::Server<controllers::setSafeY6Config>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/100;
    ros::Rate rate(100);
    long count = 0;

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        geometry_msgs::PoseStamped position_msg;
        geometry_msgs::TwistStamped velocity_msg;
        geometry_msgs::PoseStamped attitude_msg;
        std_msgs::Float64 throttle_msg;
        if(controller == 0){
            position_msg.header.stamp = ros::Time::now();
            position_msg.header.seq = count;
            position_msg.header.frame_id = "local_origin";
            if(position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > -MAX_Z && !stop){

                double x = position_d(0);
                double y = position_d(1);
                double z = position_d(2);
                double psi  = position_d(3);

                position_msg.pose.position.x = x;
                position_msg.pose.position.y = y;
                position_msg.pose.position.z = z;
                tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, psi);
                quaternionTFToMsg(q, position_msg.pose.orientation);
            }
            else{
                position_msg.pose.position.x = 0;
                position_msg.pose.position.y = 0;
                position_msg.pose.position.z = 0;
                tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
                quaternionTFToMsg(q, position_msg.pose.orientation);
            }

            position_publisher.publish(position_msg);
        }
        else if(controller == 1){
            velocity_msg.header.stamp = ros::Time::now();
            velocity_msg.header.seq = count;
            velocity_msg.header.frame_id = "local_origin";
            if(position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > -MAX_Z && !stop){

                double vx = velocity_d(0);
                double vy = velocity_d(1);
                double vz = velocity_d(2);
                double r  = velocity_d(3);

                velocity_msg.twist.linear.x = vx;
                velocity_msg.twist.linear.y = vy;
                velocity_msg.twist.linear.z = vz;
            }
            else{
                velocity_msg.twist.linear.x = 0;
                velocity_msg.twist.linear.y = 0;
                velocity_msg.twist.linear.z = 0;
            }

            velocity_publisher.publish(velocity_msg);
        }
        else if(controller == 2){
            attitude_msg.header.stamp = ros::Time::now();
            attitude_msg.header.seq = count;
            attitude_msg.header.frame_id = "local_origin";
            if(position.x < MAX_X && position.x > -MAX_X && position.y < MAX_Y && position.y > -MAX_Y && position.z < MAX_Z && position.z > -MAX_Z && !stop){

                double roll = attitude_d(0);
                double pitch = attitude_d(1);
                double yaw = attitude_d(2);
                double thrust = attitude_d(3);

                tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
                //quaternionTFToMsg(q, attitude_msg.pose.orientation);

                throttle_msg.data = thrust;
            }
            else{
                tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
                //quaternionTFToMsg(q, attitude_msg.pose.orientation);

                throttle_msg.data = 0.1;
            }

            attitude_publisher.publish(attitude_msg);
            throttle_publisher.publish(throttle_msg);
        }
        ++count;
    }
}

int main(int argc, char** argv){
    cout << "[SafeY6] SafeY6 is running..." << endl;

    SafeY6* bebop = new SafeY6(argc, argv);

    bebop->run();
}
