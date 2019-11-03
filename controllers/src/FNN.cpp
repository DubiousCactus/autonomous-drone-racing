#include "controllers/FNN.h"

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x, odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z, odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double x, y, z;
    m.getEulerZYX(z, y, x);
    pose << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z, z;
    velocity << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z, odometry_msg->twist.twist.angular.z;
    //cout << "[PID] yaw = " << (180 * z / M_PI) << endl;

    new_odometry = true;
}

void trajectoryCallback(const QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
    //cout << "[FNN] position: " << pose_d.transpose() << endl;
}

void trajectoryVelocityCallback(const QuaternionStamped& velocity_msg){
    velocity_d << velocity_msg.quaternion.x, velocity_msg.quaternion.y, velocity_msg.quaternion.z, velocity_msg.quaternion.w;
    //cout << "[FNN] velocity_d: " << velocity_d.transpose() << endl;
}

void pdCallback(const geometry_msgs::Quaternion& pd_msg){
    tau_c << pd_msg.x, pd_msg.y, pd_msg.z, pd_msg.w;
    //cout << "[FNN] tau_c: " << tau_c.transpose() << endl;
}

void dynamicReconfigureCallback(controllers::setFNNConfig &config, uint32_t level){
    fnn_type = config.fnn_type;
    alpha << config.alpha, config.alpha, config.alpha, config.alpha;
    gamma1 << config.gamma, config.gamma, config.gamma, config.gamma;
    nu << config.nu, config.nu, config.nu, config.nu;
}

// Constructor
FNN::FNN(int argc, char** argv){
    ros::init(argc, argv, "SI_T2_FPID");
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

    e_d_old = velocity_d - velocity;

    if(argc > 1){
        alpha << atof(argv[1]), atof(argv[1]), atof(argv[1]), atof(argv[1]);
        gamma1 << atof(argv[2]), atof(argv[2]), atof(argv[2]), atof(argv[2]);
        nu << atof(argv[3]), atof(argv[3]), atof(argv[3]), atof(argv[3]);
    }
    else{
        alpha << ALPHA, ALPHA, ALPHA, ALPHA;
        gamma1 << GAMMA, GAMMA, GAMMA, GAMMA;
        nu << GAMMA, GAMMA, GAMMA, GAMMA;
    }

    fnn_type = 1;

    c1 << CENTER_E, CENTER_E, CENTER_E, CENTER_E;
    c2 << CENTER_E_D, CENTER_E_D, CENTER_E_D, CENTER_E_D;
    sigma1 << VARIANCE_E, VARIANCE_E, VARIANCE_E, VARIANCE_E;
    sigma2 << VARIANCE_E_D, VARIANCE_E_D, VARIANCE_E_D, VARIANCE_E_D;

    for(int i = 0; i < 9; ++i)
        f.col(i) << ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000, ((double)rand() / RAND_MAX) / 1000;

    //cout << "***** [FNN] alpha = " << alpha(0) << ", gamma1 = " << gamma1(0) << ", nu = " << nu(0) << endl;
}

// Destructor
FNN::~FNN(){
    ros::shutdown();
    exit(0);
}

void FNN::run(){
    dynamic_reconfigure::Server<controllers::setFNNConfig> server;
    dynamic_reconfigure::Server<controllers::setFNNConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/1000;
    ros::Rate rate(1000);
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        if(pose_d(2) > 0 && new_odometry){ // if there is a command
            e = pose_d - pose;
            e_d = velocity_d - velocity;
            e_dd = (e_d - e_d_old) / dt;
            e_d_old = e_d;

            //cout << "[FNN] e = " << e(0) << ", e_d = " << e_d(0) << ", e_dd = " << e_dd(0) << ", tau_c = " << tau_c(0) << ", dt = " << dt << endl;

            if(fnn_type == 1){
                tau_f(0) = SMC(0, e(0), e_d(0), e_d(0), e_dd(0), tau_c(0), dt);
                tau_f(1) = SMC(1, e(1), e_d(1), e_d(1), e_dd(1), tau_c(1), dt);
                tau_f(2) = SMC(2, e(2), e_d(2), e_d(2), e_dd(2), tau_c(2), dt);
                tau_f(3) = SMC(3, e(3), e_d(3), e_d(3), e_dd(3), tau_c(3), dt);
            }
            else if(fnn_type == 2){
                tau_f(0) = LM(0, e(0), e_d(0), e_d(0), e_dd(0), tau_c(0), dt);
                tau_f(1) = LM(1, e(1), e_d(1), e_d(1), e_dd(1), tau_c(1), dt);
                tau_f(2) = LM(2, e(2), e_d(2), e_d(2), e_dd(2), tau_c(2), dt);
                tau_f(3) = LM(3, e(3), e_d(3), e_d(3), e_dd(3), tau_c(3), dt);
            }
            else
                tau_f << 0, 0, 0, 0;

            //cout << "[FNN] tau_f: " << tau_f.transpose() << endl;

            geometry_msgs::Quaternion velocity_msg;
            velocity_msg.x = tau_c(0) - tau_f(0);
            velocity_msg.y = tau_c(1) - tau_f(1);
            velocity_msg.z = tau_c(2) - tau_f(2);
            velocity_msg.w = tau_c(3);
            velocity_publisher.publish(velocity_msg);

            velocity_msg.x = -tau_f(0);
            velocity_msg.y = -tau_f(1);
            velocity_msg.z = -tau_f(2);
            velocity_msg.w = 0;
            fnn_velocity_publisher.publish(velocity_msg);
        }

        new_odometry = false;
    }
}

double FNN::SMC(short axis, double x1, double x1_d, double x2, double x2_d, double tau_c, double dt){
    x1 = bound(x1, -1, 1);
    x1_d = bound(x1_d, -1, 1);
    x2 = bound(x2, -1, 1);
    x2_d = bound(x2_d, -1, 1);

    //cout << "[FNN] x1 = " << x1 << ", x1_d = " << x1_d << endl;
    //cout << "[FNN] x2 = " << x2 << ", x2_d = " << x2_d << endl;

    //cout << "[FNN] c1 = " << c1.row(axis) << endl;
    //cout << "[FNN] c2 = " << c2.row(axis) << endl;

    //cout << "[FNN] sigma1 = " << sigma1.row(axis) << endl;
    //cout << "[FNN] sigma2 = " << sigma2.row(axis) << endl;

    for(int i = 0; i < 3; ++i){
        mu1(i) = Gaussian(c1(axis, i), sigma1(axis, i), x1);
        mu2(i) = Gaussian(c2(axis, i), sigma2(axis, i), x2);
    }

    //cout << "[FNN] mu1 = " << mu1.transpose() << endl;
    //cout << "[FNN] mu2 = " << mu2.transpose() << endl;

    //cout << "[FNN] ***** sigma2 = " << sigma2(0,1) << endl;

    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
            W(3 * i + j) = mu1(i) * mu2(j);

    //cout << "[FNN] W = " << W.transpose() << endl;

    W_bar = W / W.sum();

    c1_d << x1_d, x1_d, x1_d;
    c2_d << x2_d, x2_d, x2_d;

    for(int i = 0; i < 3; ++i){
            sigma1_d(i) = -pow(sigma1(axis, i), 3)/(pow(x1 - c1(axis, i), 2) + 0.001) * alpha(axis) * sign(tau_c);
            sigma2_d(i) = -pow(sigma2(axis, i), 3)/(pow(x2 - c2(axis, i), 2) + 0.001) * alpha(axis) * sign(tau_c);
    }
    f_d = -W_bar/(W_bar.transpose() * W_bar) * alpha(axis) * sign(tau_c);
    alpha_d = gamma1(axis) * abs(tau_c) - gamma1(axis) * nu(axis) * alpha(axis);

    c1.row(axis) += c1_d * dt;
    c2.row(axis) += c2_d * dt;
    sigma1.row(axis) += sigma1_d * dt;
    sigma2.row(axis) += sigma2_d * dt;
    f.row(axis) += f_d * dt;
    alpha(axis) += alpha_d * dt;

    c1.row(axis) = bound(c1.row(axis), -1, 1);
    c2.row(axis) = bound(c2.row(axis), -1, 1);
    sigma1.row(axis) = bound(sigma1.row(axis), 0.001, 1);
    sigma2.row(axis) = bound(sigma2.row(axis), 0.001, 1);
    f.row(axis) = bound(f.row(axis), -10, 10);
    alpha(axis) = bound(alpha(axis), 0, 10);

    //cout << "[FNN] axis = " << axis << endl;
    //cout << "[FNN] f = " << f.row(axis) << endl;
    //cout << "[FNN] W_bar = " << W_bar.transpose() << endl;
    //cout << "[FNN] alpha = " << alpha(axis) << endl;

    return (f.row(axis) * W_bar)(0);
}

double FNN::LM(short axis, double x1, double d_x1, double x2, double d_x2, double tau_c, double dt){
    /*c1_d = x1_d;
    c2_d = x2_d;
    for(int i = 0; i < 3; ++i){
        sigma1_d(i) = -pow(sigma1(i), 3)/pow(x1 - c1(i)) * alpha * sign(tau_c);
        sigma2_d(i) = -pow(sigma2(i), 3)/pow(x2 - c2(i)) * alpha * sign(tau_c);
    }
    delta = max(W_bar.transpose() * W_bar, alpha_bar);
    f_d = -gamma1 * (W_bar * W_bar.transpose() + delta * MatrixXd::Identity(3*3, 3*3)).inverse() * W_bar * sign(tau_c);

    c1.row(axis) += c1_d * dt;
    c2.row(axis) += c2_d * dt;
    sigma1.row(axis) += sigma_1 * dt;
    sigma2.row(axis) += sigma_2 * dt;
    f.row(axis) << f_d * dt;
    alpha.row(axis) += alpha_d * dt;*/
}

short FNN::sign(double x){
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

double FNN::Gaussian(double c, double sigma, double x){
    return exp(-pow(x - c, 2)/pow(sigma, 2));
}

double FNN::elliptic(double a, double c, double d, double x){
    if(c - d < x && x < c + d)
        return pow(1 - pow(abs((x - c) / d), a), 1.0/a);
    return 0;
}

double FNN::bound(double v, double min, double max){
    return (v < min) ? min : ((v > max) ? max : v);
}

VectorXd FNN::bound(VectorXd v, double min, double max){
    for(int i = 0; i < v.size(); ++i)
        v(i) = bound(v(i), min, max);
    return v;
}

int main(int argc, char** argv){
    cout << "[FNN] FNN position controller is running..." << endl;

    FNN* controller = new FNN(argc, argv);

    controller->run();
}
