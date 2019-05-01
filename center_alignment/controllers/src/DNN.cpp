#include "controllers/DNN.h"

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    position << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z;
    tf::Quaternion q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    orientation << roll, pitch, yaw;
    velocity << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z;
    angular_velocity << odometry_msg->twist.twist.angular.x, odometry_msg->twist.twist.angular.y, odometry_msg->twist.twist.angular.z;

    new_odometry = true;
}

void trajectoryCallback(const QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
}

void trajectoryVelocityCallback(const QuaternionStamped& velocity_msg){
    velocity_d << velocity_msg.quaternion.x, velocity_msg.quaternion.y, velocity_msg.quaternion.z;
}

void thrustCallback(const mavros_msgs::Thrust& thrust_msg){
    thrust_old = thrust;
    thrust = thrust_msg.thrust;
}

void controllerTypeCallback(const dynamic_reconfigure::Config& config_msg){
    controller_type = config_msg.ints[0].value;
}

void dynamicReconfigureCallback(controllers::setDNNConfig &config, uint32_t level){
    alpha = config.alpha;
}

// Constructor
DNN::DNN(int argc, char** argv){
    ros::init(argc, argv, "DNN");
    ros::NodeHandle node_handle;

    odometry_subscriber = node_handle.subscribe("/uav/true_odometry", 1, odometryCallback);
    trajectory_subscriber = node_handle.subscribe("/uav/trajectory", 1, trajectoryCallback);
    trajectory_velocity_subscriber = node_handle.subscribe("/uav/trajectory_velocity", 1, trajectoryVelocityCallback);
    thrust_subscriber = node_handle.subscribe("/mavros/setpoint_attitude/thrust", 1, thrustCallback);
    controller_type_subscriber = node_handle.subscribe("/safe_y6/parameter_updates", 1, controllerTypeCallback);

    velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/uav/command_velocity", 1);

    error << 0, 0, 0, 0;
    error_old << 0, 0, 0, 0;
    error_old_old << 0, 0, 0, 0;
    error_d << 0, 0, 0;
    error_d_old << 0, 0, 0;
    error_d_old_old << 0, 0, 0;

    alpha = 0.01;

    new_odometry = false;
}

// Destructor
DNN::~DNN(){
    ros::shutdown();
    exit(0);
}

double DNN::denormalizeAngle(double a1, double a2){
    if(abs(a2 - a1) > M_PI){
        if(a2 < a1)
            a1 -= 2 * M_PI;
        else
            a1 += 2 * M_PI;
    }
    return a1;
}

void DNN::run(){
    dynamic_reconfigure::Server<controllers::setDNNConfig> server;
    dynamic_reconfigure::Server<controllers::setDNNConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    std::cout << "OpenNN. Test DNN." << std::endl;

    // Neural network
    NeuralNetwork neural_networks[3];
    const std::string neural_network_file_name_x = "/home/andriy/catkin_ws/src/controllers/data/x/neural_network_bebop.xml";
    NeuralNetwork neural_network_x(neural_network_file_name_x);
    const std::string neural_network_file_name_y = "/home/andriy/catkin_ws/src/controllers/data/y/neural_network_bebop.xml";
    NeuralNetwork neural_network_y(neural_network_file_name_y);
    const std::string neural_network_file_name_z = "/home/andriy/catkin_ws/src/controllers/data/z/neural_network_bebop.xml";
    NeuralNetwork neural_network_z(neural_network_file_name_z);
    neural_networks[0].set(neural_network_x);
    neural_networks[1].set(neural_network_y);
    neural_networks[2].set(neural_network_z);

    Vector<double> inputs(neural_network_x.get_inputs_number());
    Vector<double> outputs(neural_network_x.get_outputs_number());
    Vector<double> data(neural_network_x.get_inputs_number() + neural_network_x.get_outputs_number());

    double update;
    Vector3d control;

    double dt = (double)1/100;
    ros::Rate rate(100);
    geometry_msgs::Quaternion velocity_msg;

    double time = 0;
    int c = 0;

    int controller_type = 1;

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        //ros::param::get("/safe_y6/controller", controller_type);

        if(controller_type != 0 && pose_d(2) > -10 && new_odometry){ // command

            orientation(2) = denormalizeAngle(orientation(2), pose_d(3));

            ros::Time begin = ros::Time::now();

            pose << position, orientation(2);

            error_old_old = error_old;
            error_old = error;
            error_d_old_old = error_d_old;
            error_d_old = error_d;

            error = pose_d - pose;
            error_d = velocity_d - velocity;//(error - error_old) / dt;

            for(int axis = 0; axis <= 2; ++axis){
                inputs[0] = error(axis);
                inputs[1] = error_old(axis);
                inputs[2] = error_old_old(axis);
                inputs[3] = error_d(axis);
                inputs[4] = error_d_old(axis);
                inputs[5] = error_d_old_old(axis);
                /*if(axis == 0){
                    inputs[6] = orientation(1);
                    inputs[7] = angular_velocity(1);
                }else if(axis == 1){
                    inputs[6] = orientation(0);
                    inputs[7] = angular_velocity(0);
                }else{
                    inputs[6] = thrust;
                    inputs[7] = thrust_old;
                }*/
                cout << "***** Inputs: " << inputs << endl;
                neural_networks[axis].get_scaling_layer_pointer()->set_scaling_method(ScalingLayer::MinimumMaximum);
                neural_networks[axis].get_unscaling_layer_pointer()->set_unscaling_method(UnscalingLayer::MinimumMaximum);
                outputs = neural_networks[axis].calculate_outputs(inputs);
                cout << "***** Old output: " << outputs << endl;

                update = alpha * (error(axis)/2 + error_d(axis)/2);
                update = alpha * (error(axis));
                /*if(axis == 1 || axis == 2)
                    update = -update;*/
                cout << "***** Update: " << update << endl;
                outputs[0] = outputs[0] + update;
                cout << "***** Updated output: " << outputs << endl;

                DataSet data_set(1, neural_network_x.get_inputs_number(), neural_network_x.get_outputs_number());
                inputs = neural_networks[axis].get_scaling_layer_pointer()->calculate_minimum_maximum_outputs(inputs);
                for(int i = 0; i < neural_networks[axis].get_inputs_number(); ++i)
                    data[i] = inputs[i];
                Vector<Statistics<double>> statistics = neural_networks[axis].get_unscaling_layer_pointer()->get_statistics();
                outputs[0] = 2 * (outputs[0] - statistics[0].minimum) / (statistics[0].maximum - statistics[0].minimum) - 1;
                for(int i = neural_network_x.get_inputs_number(); i < neural_network_x.get_inputs_number() + neural_network_x.get_outputs_number(); ++i)
                    data[i] = outputs[0];
                data_set.add_instance(data);

                //cout << "***** Target: " << data_set.get_instance(1) << endl;

                neural_networks[axis].get_scaling_layer_pointer()->set_scaling_method(ScalingLayer::NoScaling);
                neural_networks[axis].get_unscaling_layer_pointer()->set_unscaling_method(UnscalingLayer::NoUnscaling);
                LossIndex loss_index;
                loss_index.set_regularization_type(LossIndex::NEURAL_PARAMETERS_NORM);
                loss_index.set_data_set_pointer(&data_set);
                loss_index.set_neural_network_pointer(neural_networks + axis);

                TrainingStrategy training_strategy;
                training_strategy.set(&loss_index);
                training_strategy.set_main_type("GRADIENT_DESCENT");
                GradientDescent* gdp = training_strategy.get_gradient_descent_pointer();
                gdp->set_maximum_time(0.001);
                gdp->set_maximum_iterations_number(10);
                gdp->set_display_period(10);
                gdp->set_minimum_loss_increase(1.0e-3);
                gdp->set_reserve_loss_history(true);
                training_strategy.set_display(false);
                training_strategy.perform_training();

                inputs[0] = error(axis);
                inputs[1] = error_old(axis);
                inputs[2] = error_old_old(axis);
                inputs[3] = error_d(axis);
                inputs[4] = error_d_old(axis);
                inputs[5] = error_d_old_old(axis);
                /*if(axis == 0){
                    inputs[6] = orientation(1);
                    inputs[7] = angular_velocity(1);
                }else if(axis == 1){
                    inputs[6] = orientation(0);
                    inputs[7] = angular_velocity(0);
                }else{
                    inputs[6] = thrust;
                    inputs[7] = thrust_old;
                }*/
                neural_networks[axis].get_scaling_layer_pointer()->set_scaling_method(ScalingLayer::MinimumMaximum);
                neural_networks[axis].get_unscaling_layer_pointer()->set_unscaling_method(UnscalingLayer::MinimumMaximum);
                outputs = neural_networks[axis].calculate_outputs(inputs);
                cout << "***** New output: " << outputs << endl << endl;

                control(axis) = outputs[0];
            }

            velocity_msg.x = control(0);
            velocity_msg.y = control(1);
            velocity_msg.z = control(2);
            velocity_msg.w = 0; //k_p * error(3);

            time += (ros::Time::now() - begin).toSec() * 1000;
            c++;

            velocity_publisher.publish(velocity_msg);

            cout << "[DNN]: time = " << (time/c) << endl;
        }

        //new_odometry = false;
    }
}

short DNN::sign(double x){
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

int main(int argc, char** argv){
    cout << "[DNN] DNN position controller is running..." << endl;

    DNN* controller = new DNN(argc, argv);

    controller->run();
}
