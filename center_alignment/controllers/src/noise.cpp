#include "controllers/noise.h"

void dynamicReconfigureCallback(controllers::setNoiseConfig &config, uint32_t level){
    noise_position = config.noise_position;
    noise_orientation = config.noise_attitude;
    noise_velocity = config.noise_velocity;
    noise_angular_velocity = config.noise_angular_velocity;
}

// Constructor
Noise::Noise(int argc, char** argv){
    ros::init(argc, argv, "Noise");
    ros::NodeHandle node_handle;

    noise_publisher = node_handle.advertise<nav_msgs::Odometry>("/uav/noise", 1);

    if(argc == 2)
        noise_position = atof(argv[1]);
    if(argc == 5){
        noise_position = atof(argv[1]);
        noise_orientation = atof(argv[2]) / 180 * M_PI;
        noise_velocity = atof(argv[3]);
        noise_angular_velocity = atof(argv[4]);
    }
}

// Destructor
Noise::~Noise(){
    ros::shutdown();
    exit(0);
}

void Noise::run(){
    dynamic_reconfigure::Server<controllers::setNoiseConfig> server;
    dynamic_reconfigure::Server<controllers::setNoiseConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    long sequence = 0;
    ros::Rate rate(100);
    while(ros::ok()){
      rate.sleep();
      ros::spinOnce();

      tf::Quaternion q = tf::createQuaternionFromRPY(generateGaussianNoise(noise_orientation), generateGaussianNoise(noise_orientation), generateGaussianNoise(noise_orientation));

      nav_msgs::Odometry noise_msg;
      noise_msg.header.seq = sequence++;
      noise_msg.header.stamp = Time::now();
      noise_msg.pose.pose.position.x = generateGaussianNoise(noise_position);
      noise_msg.pose.pose.position.y = generateGaussianNoise(noise_position);
      noise_msg.pose.pose.position.z = generateGaussianNoise(noise_position);
      noise_msg.pose.pose.orientation.w = q.getW();
      noise_msg.pose.pose.orientation.x = q.getX();
      noise_msg.pose.pose.orientation.y = q.getY();
      noise_msg.pose.pose.orientation.z = q.getZ();
      noise_msg.twist.twist.linear.x = generateGaussianNoise(noise_velocity);
      noise_msg.twist.twist.linear.y = generateGaussianNoise(noise_velocity);
      noise_msg.twist.twist.linear.z = generateGaussianNoise(noise_velocity);
      noise_msg.twist.twist.angular.x = generateGaussianNoise(noise_angular_velocity);
      noise_msg.twist.twist.angular.y = generateGaussianNoise(noise_angular_velocity);
      noise_msg.twist.twist.angular.z = generateGaussianNoise(noise_angular_velocity);
      noise_publisher.publish(noise_msg);
    }
}

double Noise::generateGaussianNoise(double sigma){
    double mu = 0;
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0 * M_PI;

    static double z0, z1;
    static bool generate;
    generate = !generate;

    if(!generate)
       return z1 * sigma + mu;

    double u1, u2;
    do{
       u1 = rand() * (1.0 / RAND_MAX);
       u2 = rand() * (1.0 / RAND_MAX);
     }while(u1 <= epsilon);

    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}

int main(int argc, char** argv){
    cout << "[Noise] Noise generator is running..." << endl;

    Noise* controller = new Noise(argc, argv);

    controller->run();
}
