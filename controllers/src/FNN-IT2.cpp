#include "controllers/FNN-IT2.h"

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x, odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z, odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double x, y, z;
    m.getEulerZYX(z, y, x);
    pose << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z, z;
    velocity << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z, odometry_msg->twist.twist.angular.z;
    //cout << "[FNN_IT2] yaw = " << (180 * z / M_PI) << endl;

    new_odometry = true;
}

void trajectoryCallback(const QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
    //cout << "[FNN_IT2] position: " << pose_d.transpose() << endl;
}

void trajectoryVelocityCallback(const QuaternionStamped& velocity_msg){
    velocity_d << velocity_msg.quaternion.x, velocity_msg.quaternion.y, velocity_msg.quaternion.z, velocity_msg.quaternion.w;
    //cout << "[FNN_IT2] velocity_d: " << velocity_d.transpose() << endl;
}

void pdCallback(const geometry_msgs::Quaternion& pd_msg){
    u_c << pd_msg.x, pd_msg.y, pd_msg.z, pd_msg.w;
    //cout << "[FNN_IT2] tau_c: " << tau_c.transpose() << endl;
}

void dynamicReconfigureCallback(controllers::setFNN_IT2Config &config, uint32_t level){
    mf_type = config.mf;
    alpha << config.alpha, config.alpha, config.alpha, config.alpha;
    gamma1 << config.gamma, config.gamma, config.gamma, config.gamma;
}

// Constructor
FNN_IT2::FNN_IT2(int argc, char** argv){
    ros::init(argc, argv, "FNN_IT2");
    ros::NodeHandle node_handle;

    odometry_subscriber = node_handle.subscribe("/uav/odometry", 1, odometryCallback);
    trajectory_subscriber = node_handle.subscribe("/uav/trajectory", 1, trajectoryCallback);
    trajectory_velocity_subscriber = node_handle.subscribe("/uav/trajectory_velocity", 1, trajectoryVelocityCallback);
    pd_subscriber = node_handle.subscribe("/uav/command_velocity_pd", 1, pdCallback);

    velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/uav/command_velocity_fnn", 1);
    fnn_velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/fnn/command_velocity", 1);

    pose << 0, 0, 0, 0;
    pose_d << 0, 0, 0, 0;
    velocity << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;

    if(argc > 1){
        alpha << atof(argv[1]), atof(argv[1]), atof(argv[1]), atof(argv[1]);
        gamma1 << atof(argv[2]), atof(argv[2]), atof(argv[2]), atof(argv[2]);
    }
    else{
        alpha << ALPHA, ALPHA, ALPHA, ALPHA;
        gamma1 << GAMMA, GAMMA, GAMMA, GAMMA;
    }

    mf_type = 1;

    a11 << A1, A1, A1, A1;
    a12 << A2, A2, A2, A2;
    a21 << A1, A1, A1, A1;
    a22 << A2, A2, A2, A2;
    c1 << C, C, C, C;
    c2 << C, C, C, C;
    d1 << D, D, D, D;
    d2 << D, D, D, D;

    sigma11 << SIGMA1, SIGMA1, SIGMA1, SIGMA1;
    sigma12 << SIGMA2, SIGMA2, SIGMA2, SIGMA2;
    sigma21 << SIGMA1, SIGMA1, SIGMA1, SIGMA1;
    sigma22 << SIGMA2, SIGMA2, SIGMA2, SIGMA2;

    q << 0.5, 0.5, 0.5, 0.5;

    for(int i = 0; i < 9; ++i)
        f.col(i) << ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000;

    cout << "***** [FNN_IT2] alpha = " << alpha(0) << ", gamma1 = " << gamma1(0) << endl;
}

// Destructor
FNN_IT2::~FNN_IT2(){
    ros::shutdown();
    exit(0);
}

void FNN_IT2::run(){
    dynamic_reconfigure::Server<controllers::setFNN_IT2Config> server;
    dynamic_reconfigure::Server<controllers::setFNN_IT2Config>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/200;
    ros::Rate rate(200);
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        if(pose_d(2) > 0 && new_odometry){ // if there is a command
            e = pose_d - pose;
            e_d = velocity_d - velocity;

            u_f(0) = update(0, e(0), e_d(0), u_c(0), dt);
            u_f(1) = update(1, e(1), e_d(1), u_c(1), dt);
            u_f(2) = update(2, e(2), e_d(2), u_c(2), dt);
            u_f(3) = update(3, e(3), e_d(3), u_c(3), dt);

            //cout << "[FNN_IT2] u_f: " << u_f.transpose() << endl;

            geometry_msgs::Quaternion velocity_msg;
            velocity_msg.x = u_c(0) - u_f(0);
            velocity_msg.y = u_c(1) - u_f(1);
            velocity_msg.z = u_c(2) - u_f(2);
            velocity_msg.w = u_c(3);
            velocity_publisher.publish(velocity_msg);

            velocity_msg.x = -u_f(0);
            velocity_msg.y = -u_f(1);
            velocity_msg.z = -u_f(2);
            velocity_msg.w = -u_f(3);
            fnn_velocity_publisher.publish(velocity_msg);
        }

        new_odometry = false;
    }
}

double FNN_IT2::update(short axis, double x1, double x2, double u_c, double dt){
    x1 = bound(x1, -1, 1);
    x2 = bound(x2, -1, 1);

    for(int i = 0; i < 3; ++i){
        if(mf_type == 0){
            mu1_upper(i) = Gaussian(c1(axis, i), sigma11(axis, i), x1);
            mu1_lower(i) = Gaussian(c1(axis, i), sigma12(axis, i), x1);
            mu2_upper(i) = Gaussian(c2(axis, i), sigma21(axis, i), x2);
            mu2_lower(i) = Gaussian(c2(axis, i), sigma22(axis, i), x2);
        }
        if(mf_type == 1){
            mu1_upper(i) = elliptic(a11(axis, i), c1(axis, i), d1(axis, i), x1);
            mu1_lower(i) = elliptic(a12(axis, i), c1(axis, i), d1(axis, i), x1);
            mu2_upper(i) = elliptic(a21(axis, i), c2(axis, i), d2(axis, i), x2);
            mu2_lower(i) = elliptic(a22(axis, i), c2(axis, i), d2(axis, i), x2);
        }
    }

    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j){
            W_upper(3 * i + j) = mu1_upper(i) * mu2_upper(j);
            W_lower(3 * i + j) = mu1_lower(i) * mu2_lower(j);
        }

    W_upper_bar = W_upper / W_upper.sum();
    W_lower_bar = W_lower / W_lower.sum();

    Pi = q(axis) * W_lower_bar + (1 - q(axis)) * W_upper_bar;
    f_d = -Pi/(Pi.transpose() * Pi) * alpha(axis) * sign(u_c);
    alpha_d = 2 * gamma1(axis) * abs(u_c);
    q_d = -1.0/(f(axis) * (W_lower_bar - W_upper_bar).transpose())(0) * alpha(axis) * abs(u_c);

    f.row(axis) += f_d * dt;
    //alpha(axis) += alpha_d * dt;
    q(axis) = 0.5;//+= q_d * dt;

    f.row(axis) = bound(f.row(axis), -10, 10);
    alpha(axis) = bound(alpha(axis), 0, 1);
    q(axis) = bound(q(axis), 0, 1);

    //cout << "[FNN_IT2] FNN output: " << (q(axis) * f.row(axis) * W_lower_bar + (1 - q(axis)) * f.row(axis) * W_upper_bar) << endl;

    return (q(axis) * f.row(axis) * W_lower_bar + (1 - q(axis)) * f.row(axis) * W_upper_bar)(0);
}

short FNN_IT2::sign(double x){
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

double FNN_IT2::Gaussian(double c, double sigma, double x){
    return exp(-pow(x - c, 2)/pow(sigma, 2));
}

double FNN_IT2::elliptic(double a, double c, double d, double x){
    if(c - d < x && x < c + d)
        return pow(1 - pow(abs((x - c) / d), a), 1.0/a);
    return 0;
}

double FNN_IT2::bound(double v, double min, double max){
    return (v < min) ? min : ((v > max) ? max : v);
}

VectorXd FNN_IT2::bound(VectorXd v, double min, double max){
    for(int i = 0; i < v.size(); ++i)
        v(i) = bound(v(i), min, max);
    return v;
}

int main(int argc, char** argv){
    cout << "[FNN_IT2] FNN_IT2 position controller is running..." << endl;

    FNN_IT2* controller = new FNN_IT2(argc, argv);

    controller->run();
}
