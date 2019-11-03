#include "controllers/SE3.h"

SE3::SE3(int argc, char** argv){
    ros::init(argc, argv, "se3");
    ros::NodeHandle node_handle;

    /*ros::NodeHandle nh("~");

    nh.getParam("initial_x", position.x);
    nh.getParam("initial_y", position.y);
    nh.getParam("initial_z", position.z);*/

    velocity_subscriber = node_handle.subscribe("/vtol/velocity", 1, &SE3::velocityCallback, this);
    attitude_subscriber = node_handle.subscribe("/vtol/attitude", 1, &SE3::attitudeCallback, this);
    angular_velocity_subscriber = node_handle.subscribe("/vtol/angular_velocity", 1, &SE3::angularVelocityCallback, this);
    desired_velocity_subscriber = node_handle.subscribe("/vtol/command_velocity", 1, &SE3::desiredVelocityCallback, this);

    velocity_publisher = node_handle.advertise<QuaternionStamped>("/vtol/input", 1);

    ros::param::get("vtol_m", m);
    ros::param::get("vtol_Ix", I(0,0));
    ros::param::get("vtol_Iy", I(1,1));
    ros::param::get("vtol_Iz", I(2,2));

    m = 1;
    I << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    g = 9.8;

    e3 << 0, 0, 1;

    kv = 10;
    kR = 10;
    kOmega = 10;

    v << 0, 0, 0;
    R = getRotationMatrix(0, 0, 0);
    Omega << 0, 0, 0;
    vd << 0, 0, 0;
    x_vec << 1, 0, 0;
}

void SE3::run() {
    ros::Rate rate(1000);
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        vd << 1, 0, 0;

        ev = v - vd;

        if(vd(0) != 0 || vd(1) != 0)
            x_vec << vd(0), vd(1), 0;
        if((kv * ev + m * g * e3).norm() != 0)
            z_vec = (-kv * ev - m * g * e3) / (-kv * ev - m * g * e3).norm();
        if((z_vec.cross(x_vec)).norm() != 0)
            y_vec = (z_vec.cross(x_vec)) / (z_vec.cross(x_vec)).norm();

        cout << "x_vec: " << x_vec.transpose() << ", y_vec: " << y_vec.transpose() << ", z_vec: " << z_vec.transpose() << endl;

        Rd.col(0) = y_vec.cross(z_vec);
        Rd.col(1) = y_vec;
        Rd.col(2) = z_vec;

        cout << "Rd:\n" << Rd << endl;

        eR = (double)1/2 * veeMap(Rd.transpose() * R - R.transpose() * Rd);

        cout << "R*R:\n" << Rd.transpose() * R - R.transpose() * Rd << endl;
        cout << "eR:" << eR.transpose() << endl;

        T = -(-kv * ev - m * g * e3).transpose() * R * e3;
        torque = -kR * eR - kOmega * Omega + Omega.cross(I * Omega);

        cout << "T = " << T << endl;
        cout << "torque: " << torque.transpose() << endl;

        msg_command.header.stamp = ros::Time::now();
        msg_command.quaternion.x = torque(0);
        msg_command.quaternion.y = -torque(1);
        msg_command.quaternion.z = -torque(2);
        msg_command.quaternion.w = T;
        velocity_publisher.publish(msg_command);
    }
}

void SE3::velocityCallback(const Vector3Stamped& msg){
    v << msg.vector.x, msg.vector.y, msg.vector.z;
}

void SE3::attitudeCallback(const Vector3Stamped& msg){
    R = getRotationMatrix(msg.vector.x, msg.vector.y, msg.vector.z);
}

void SE3::angularVelocityCallback(const Vector3Stamped& msg){
    Omega << msg.vector.x, msg.vector.y, msg.vector.z;
}

void SE3::desiredVelocityCallback(const Vector3Stamped& msg){
    vd << msg.vector.x, -msg.vector.y, -msg.vector.z;
}

void SE3::setGains(double kx, double kv, double kR, double kOmega) {
    kx = kx;
    kv = kv;
    kR = kR;
    kOmega = kOmega;
}

Matrix3d SE3::getRotationMatrix(double phi, double theta, double psi) {
    Matrix3d R;
    R << cos(psi) * cos(theta), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi), sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta),
            cos(theta) * sin(psi), cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta), cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi),
            -sin(theta), cos(theta) * sin(phi), cos(phi) * cos(theta);
    return R;
}

Vector3d SE3::veeMap(Matrix3d M){
    Vector3d v;
    v << M(2, 1), M(0, 2), M(1, 0);
    return v;
}

int main(int argc, char** argv){
    cout << "[SE3] SE3 is running..." << endl;

    SE3* se = new SE3(argc, argv);

    se->run();
}
