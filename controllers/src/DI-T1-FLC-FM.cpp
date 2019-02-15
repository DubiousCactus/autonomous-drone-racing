#include "controllers/DI-T1-FLC-FM.h"

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x, odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z, odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double x, y, z;
    m.getEulerZYX(z, y, x);
    pose << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z, z;
    velocity << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z, odometry_msg->twist.twist.angular.z;
    //cout << "[DI_T1_FLC_FM] yaw = " << (180 * z / M_PI) << endl;

    new_odometry = true;
}

void trajectoryCallback(const QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
    //cout << "[DI_T1_FLC_FM] position_d: " << pose_d.transpose() << endl;
}

void trajectoryVelocityCallback(const QuaternionStamped& velocity_msg){
    velocity_d << velocity_msg.quaternion.x, velocity_msg.quaternion.y, velocity_msg.quaternion.z, velocity_msg.quaternion.w;
    //cout << "[DI_T1_FLC_FM] velocity_d: " << velocity_d.transpose() << endl;
}

void dynamicReconfigureCallback(controllers::setDIT1FLCConfig &config, uint32_t level){
    k_p = config.k_p;
    k_d = config.k_d;
    k_a = config.k_a;
    k_b = config.k_b;
}

// Constructor
DI_T1_FLC_FM::DI_T1_FLC_FM(int argc, char** argv){
    ros::init(argc, argv, "DI_T1_FLC_FM");
    ros::NodeHandle node_handle;

    odometry_subscriber = node_handle.subscribe("/uav/odometry", 1, odometryCallback);
    trajectory_subscriber = node_handle.subscribe("/uav/trajectory", 1, trajectoryCallback);
    trajectory_velocity_subscriber = node_handle.subscribe("/uav/trajectory_velocity", 1, trajectoryVelocityCallback);

    velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/uav/command_velocity", 1);

    pose << 0, 0, 0, 0;
    pose_d << 0, 0, 0, 0;
    velocity << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;

    phi_i << 0, 0, 0, 0;

    if(argc > 1){
        k_p = atof(argv[1]);
        k_d = atof(argv[2]);
        k_a = atof(argv[3]);
        k_b = atof(argv[4]);
    }
    else{
        k_p = 1.0;
        k_d = 0.004;
        k_a = 0.077;
        k_b = 7.336;
    }

    new_odometry = false;
}

// Destructor
DI_T1_FLC_FM::~DI_T1_FLC_FM(){
    ros::shutdown();
    exit(0);
}

double DI_T1_FLC_FM::denormalizeAngle(double a1, double a2){
    if(abs(a2 - a1) > M_PI){
        if(a2 < a1)
            a1 -= 2 * M_PI;
        else
            a1 += 2 * M_PI;
    }
    return a1;
}

double DI_T1_FLC_FM::phi(double sigma1, double sigma2){
    return sigma1 + sigma2 - (abs(sigma1) * sigma2 + sigma1 * abs(sigma2)) / 2;
}

double DI_T1_FLC_FM::bound(double n){
    return n <= -1 ? -1 : n >= 1 ? 1 : n;
}

void DI_T1_FLC_FM::run(){
    dynamic_reconfigure::Server<controllers::setDIT1FLCConfig> server;
    dynamic_reconfigure::Server<controllers::setDIT1FLCConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/100;
    ros::Rate rate(100);
    double time = 0;
    int c = 0;
    geometry_msgs::Quaternion velocity_msg;

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        if(pose_d(2) > -10 && new_odometry){ // command
            pose(3) = denormalizeAngle(pose(3), pose_d(3));

            ros::Time begin = ros::Time::now();

            error = pose_d - pose;
            error_d = velocity_d - velocity;

            sigma1 << bound(k_p * error(0)), bound(k_p * error(1)), bound(k_p * error(2)), bound(k_p * error(3));
            sigma2 << bound(k_d * error_d(0)), bound(k_d * error_d(1)), bound(k_d * error_d(2)), bound(k_d * error_d(3));

            phi_p << phi(sigma1(0), sigma2(0)), phi(sigma1(1), sigma2(1)), phi(sigma1(2), sigma2(2)), phi(sigma1(3), sigma2(3));
            phi_i += phi_p * dt;

            velocity_msg.x = k_a * phi_p(0) + k_b * phi_i(0);
            velocity_msg.y = k_a * phi_p(1) + k_b * phi_i(1);
            velocity_msg.z = k_a * phi_p(2) + k_b * phi_i(2);
            velocity_msg.w = phi_p(3);

            time += (ros::Time::now() - begin).toSec() * 1000;
            c++;

            velocity_publisher.publish(velocity_msg);

            //cout << "[DI_T1_FLC_FM]: time = " << (time/c) << endl;
        }

        new_odometry = false;
    }
}

int main(int argc, char** argv){
    cout << "[DI_T1_FLC_FM] DI_T1_FLC_FM position controller is running..." << endl;

    DI_T1_FLC_FM* controller = new DI_T1_FLC_FM(argc, argv);

    /*VectorXd sigma1(1);
    VectorXd sigma2(1);
    VectorXd phi(0);
    for(sigma1(0) = -1; sigma1(0) <= 1; sigma1(0)+=0.5)
        for(sigma2(0) = -1; sigma2(0) <= 1; sigma2(0)+=0.5){
            phi = sigma1 + sigma2 - (sigma1.cwiseAbs().cwiseProduct(sigma2) + sigma1.cwiseProduct(sigma2.cwiseAbs()))/2;
            cout << "[DI_T1_FLC_FM] phi(" << sigma1(0) << ", " << sigma2(0) << ") = " << phi << endl;}*/

    controller->run();
}
