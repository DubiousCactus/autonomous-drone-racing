#include "controllers/logger.h"

void trajectoryCallback(const geometry_msgs::QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
}

void trajectoryVelocityCallback(const geometry_msgs::QuaternionStamped& trajectory_velocity_msg){
    velocity_d << trajectory_velocity_msg.quaternion.x, trajectory_velocity_msg.quaternion.y, trajectory_velocity_msg.quaternion.z;
}

void mocapCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x, odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z, odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    position_mocap << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z;
    orientation_mocap << roll / M_PI * 180, pitch / M_PI * 180, yaw / M_PI * 180;
    velocity_mocap << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z;
    rate_mocap << odometry_msg->twist.twist.angular.x, odometry_msg->twist.twist.angular.y, odometry_msg->twist.twist.angular.z;
}

void bebopCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x, odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z, odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    position_bebop << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z;
    orientation_bebop << roll / M_PI * 180, pitch / M_PI * 180, yaw / M_PI * 180;
    velocity_bebop << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z;
    rate_bebop << odometry_msg->twist.twist.angular.x, odometry_msg->twist.twist.angular.y, odometry_msg->twist.twist.angular.z;
}

void visionCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x, odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z, odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    position_vision << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z;
    orientation_vision << roll / M_PI * 180, pitch / M_PI * 180, yaw / M_PI * 180;
    velocity_vision << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z;
    rate_vision << odometry_msg->twist.twist.angular.x, odometry_msg->twist.twist.angular.y, odometry_msg->twist.twist.angular.z;
}

void controllerCommandCallback(const geometry_msgs::Quaternion& command_msg){
    command_controller << command_msg.x, command_msg.y, command_msg.z, command_msg.w;
}

void bebopCommandCallback(const geometry_msgs::Twist& command_msg){
    command_bebop << command_msg.linear.x, command_msg.linear.y, command_msg.linear.z, command_msg.angular.z;
}

void teleopCallback(const std_msgs::Int8& command_msg){
    switch(command_msg.data){
    case 2:
        flying = true;
        break;
    case 3:
        flying = false;
        break;
    }
}

// Constructor
Logger::Logger(int argc, char** argv){
    ros::init(argc, argv, "Logger");
    ros::NodeHandle node_handle;

    flying = true;  //false

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H:%M:%S", timeinfo);
    const char* str(buffer);

    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;

    char file[100];
    strcpy(file, homedir);
    strcat(file, "/Results/");
    strcat(file, str);
    strcat(file, "__");
    strcat(file, argv[1]);
    strcat(file, ".txt");
    results.open(file);

    cout << "*****[Logger]: file = " << file << endl;

    trajectory_subscriber = node_handle.subscribe("/bebop/trajectory", 1, trajectoryCallback);
    trajectory_velocity_subscriber = node_handle.subscribe("/bebop/trajectory_velocity", 1, trajectoryVelocityCallback);
    mocap_subscriber = node_handle.subscribe("/bebop/odometry", 1, mocapCallback);
    bebop_subscriber = node_handle.subscribe("/bebop/odom", 1, bebopCallback);
    vision_subscriber = node_handle.subscribe("/bebop/odometry_noisy", 1, visionCallback);
    //vision_quality_subscriber = node_handle.subscribe("/vslam/info", 1, visionQualityCallback);
    controller_subscriber = node_handle.subscribe("/bebop/command_velocity", 1, controllerCommandCallback);
    command_subscriber = node_handle.subscribe("/bebop/cmd_vel", 1, bebopCommandCallback);
    teleop_subscriber = node_handle.subscribe("/bebop/command", 1, teleopCallback);
}

// Destructor
Logger::~Logger(){
    ros::shutdown();
    exit(0);
}

void Logger::run(){
    ros::Rate rate(100);

    results << "time" << "\t"                                                                                                                           // 1
            << "pose_d_x" << "\t" << "pose_d_y" << "\t" << "pose_d_z" << "\t" << "pose_d_yaw" << "\t"                                                   // 2, 3, 4, 5
            << "velocity_d_x" << "\t" << "velocity_d_y" << "\t" << "velocity_d_z" << "\t"                                                               // 6, 7, 8
            << "position_mocap_x" << "\t" << "position_mocap_y" << "\t" << "position_mocap_z" << "\t"                                                   // 9, 10, 11
            << "orientation_mocap_x" << "\t" << "orientation_mocap_y" << "\t" << "orientation_mocap_z" << "\t"                                          // 12, 13, 14
            << "velocity_mocap_x" << "\t" << "velocity_mocap_y" << "\t" << "velocity_mocap_z" << "\t"                                                   // 15, 16, 17
            << "rate_mocap_x" << "\t" << "rate_mocap_y" << "\t" << "rate_mocap_z" << "\t"                                                               // 18, 19, 20
            << "position_bebop_x" << "\t" << "position_bebop_y" << "\t" << "position_bebop_z" << "\t"                                                   // 21, 22, 23
            << "orientation_bebop_x" << "\t" << "orientation_bebop_y" << "\t" << "orientation_bebop_z" << "\t"                                          // 25, 26, 27
            << "velocity_bebop_x" << "\t" << "velocity_bebop_y" << "\t" << "velocity_bebop_z" << "\t"                                                   // 28, 29, 30
            << "rate_bebop_x" << "\t" << "rate_bebop_y" << "\t" << "rate_bebop_z" << "\t"                                                               // 31, 32, 33
            << "position_vision_x" << "\t" << "position_vision_y" << "\t" << "position_vision_z" << "\t"                                                // 34, 35, 36
            << "orientation_vision_x" << "\t" << "orientation_vision_y" << "\t" << "orientation_vision_z" << "\t"                                       // 37, 38 ,39
            << "velocity_vision_x" << "\t" << "velocity_vision_y" << "\t" << "velocity_vision_z" << "\t"                                                // 40, 41, 42
            << "rate_vision_x" << "\t" << "rate_vision_y" << "\t" << "rate_vision_z" << "\t"                                                            // 43, 44, 45
            //<< "tracking_quality" << "\t"                                                                                                               // 46
            << "command_controller_x" << "\t" << "command_controller_y" << "\t" << "command_controller_z" << "\t" << "command_controller_yaw" << "\t"   // 47, 48, 49, 50
            << "command_bebop_x" << "\t" << "command_bebop_y" << "\t" << "command_bebop_z" << "\t" << "command_bebop_yaw"                               // 51, 52, 53, 54
            << endl;

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        if(pose_d(2) > -10 && flying){
            if(beginning.isZero())
                beginning = ros::Time::now();

            results << (ros::Time::now() - beginning) << "\t"                                                                                             // 1
                    << pose_d(0) << "\t" << pose_d(1) << "\t" << pose_d(2) << "\t" << pose_d(3) << "\t"                                                   // 2, 3, 4, 5
                    << velocity_d(0) << "\t" << velocity_d(1) << "\t" << velocity_d(2) << "\t"                                                            // 6, 7, 8
                    << position_mocap(0) << "\t" << position_mocap(1) << "\t" << position_mocap(2) << "\t"                                                // 9, 10, 11
                    << orientation_mocap(0) << "\t" << orientation_mocap(1) << "\t" << orientation_mocap(2) << "\t"                                       // 12, 13, 14
                    << velocity_mocap(0) << "\t" << velocity_mocap(1) << "\t" << velocity_mocap(2) << "\t"                                                // 15, 16, 17
                    << rate_mocap(0) << "\t" << rate_mocap(1) << "\t" << rate_mocap(2) << "\t"                                                            // 18, 19, 20
                    << position_bebop(0) << "\t" << position_bebop(1) << "\t" << position_bebop(2) << "\t"                                                // 21, 22, 23
                    << orientation_bebop(0) << "\t" << orientation_bebop(1) << "\t" << orientation_bebop(2) << "\t"                                       // 25, 26, 27
                    << velocity_bebop(0) << "\t" << velocity_bebop(1) << "\t" << velocity_bebop(2) << "\t"                                                // 28, 29, 30
                    << rate_bebop(0) << "\t" << rate_bebop(1) << "\t" << rate_bebop(2) << "\t"                                                            // 31, 32, 33
                    << position_vision(0) << "\t" << position_vision(1) << "\t" << position_vision(2) << "\t"                                             // 34, 35, 36
                    << orientation_vision(0) << "\t" << orientation_vision(1) << "\t" << orientation_vision(2) << "\t"                                    // 37, 38 ,39
                    << velocity_vision(0) << "\t" << velocity_vision(1) << "\t" << velocity_vision(2) << "\t"                                             // 40, 41, 42
                    << rate_vision(0) << "\t" << rate_vision(1) << "\t" << rate_vision(2) << "\t"                                                         // 43, 44, 45
                    //<< tracking_quality << "\t"                                                                                                           // 46
                    << command_controller(0) << "\t" << command_controller(1) << "\t" << command_controller(2) << "\t" << command_controller(3) << "\t"   // 47, 48, 49, 50
                    << command_bebop(0) << "\t" << command_bebop(1) << "\t" << command_bebop(2) << "\t" << command_bebop(3)                               // 51, 52, 53, 54
                    << endl;
        }
    }
}

int main(int argc, char** argv){
    cout << "[logger] Logger is running..." << endl;

    Logger* logger = new Logger(argc, argv);

    logger->run();
}
