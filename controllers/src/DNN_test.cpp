#include <iostream>
#include <math.h>
#include <time.h>
#include <Eigen/Dense>
#include <opennn.h>

using namespace OpenNN;
using namespace std;
using namespace Eigen;

int main(int argc, char** argv){
    cout << "[DNN] DNN position controller is running..." << endl;

    Vector4d error;
    Vector4d error_old;
    Vector4d error_old_old;
    Vector3d error_d;
    Vector3d error_d_old;
    Vector3d error_d_old_old;
    Vector3d orientation;
    Vector3d angular_velocity;

    double update;

    // Neural network
    NeuralNetwork neural_networks[3];
    const std::string neural_network_file_name_x = "/home/andriy/catkin_ws/src/controllers/data/x/neural_network1.xml";
    NeuralNetwork neural_network_x(neural_network_file_name_x);
    const std::string neural_network_file_name_y = "/home/andriy/catkin_ws/src/controllers/data/y/neural_network1.xml";
    NeuralNetwork neural_network_y(neural_network_file_name_y);
    const std::string neural_network_file_name_z = "/home/andriy/catkin_ws/src/controllers/data/z/neural_network1.xml";
    NeuralNetwork neural_network_z(neural_network_file_name_z);
    neural_networks[0].set(neural_network_x);
    neural_networks[1].set(neural_network_y);
    neural_networks[2].set(neural_network_z);

    Vector<double> inputs(neural_network_x.get_inputs_number());
    Vector<double> outputs(neural_network_x.get_outputs_number());
    Vector<double> data(neural_network_x.get_inputs_number() + neural_network_x.get_outputs_number());

    float elapsed = 0;
    int c = 0;
    time_t begin, end, diff1, diff2;

    bool debug = false, performance = false;

    for(int loop = 0; loop < 10000; ++loop){
        begin = clock();

        error << 1, -1, 1, 0;
        error_old << 1, -1, 1, 0;
        error_old_old << 1, -1, 1, 0;
        error_d << -1, 0, 1;
        error_d_old << -1, 0, 1;
        error_d_old_old << -1, 0, 1;
        orientation << 0, 0, 0;
        angular_velocity << 0, 0, 0;

        for(int axis = 0; axis <= 2; ++axis){
            diff1 = clock();
            inputs[0] = error(axis);
            inputs[1] = error_old(axis);
            inputs[2] = error_old_old(axis);
            inputs[3] = error_d(axis);
            inputs[4] = error_d_old(axis);
            inputs[5] = error_d_old_old(axis);
            //inputs[6] = orientation(axis);
            //inputs[7] = angular_velocity(axis);
            neural_networks[axis].get_scaling_layer_pointer()->set_scaling_method(ScalingLayer::MinimumMaximum);
            neural_networks[axis].get_unscaling_layer_pointer()->set_unscaling_method(UnscalingLayer::MinimumMaximum);
            outputs = neural_networks[axis].calculate_outputs(inputs);
            if(debug) cout << "***** Inputs: " << inputs << endl;
            if(debug) cout << "***** Old output: " << outputs << endl;
            diff2 = clock(); if(performance) cout << "***** Time1: " << ((float)(diff2 - diff1)/CLOCKS_PER_SEC) << endl;

            diff1 = clock();
            update = abs(error(axis)) - abs(error_d(axis))/2 - 3*abs(error(axis))*abs(error_d(axis))/4 - 3*error(axis)*error_d(axis)/4;
            if(debug) cout << "***** Update: " << update << endl;
            outputs[0] = outputs[0] * (1 + update/10);
            if(debug) cout << "***** Update output: " << outputs << endl;
            diff2 = clock(); if(performance) cout << "***** Time2: " << ((float)(diff2 - diff1)/CLOCKS_PER_SEC) << endl;

            diff1 = clock();
            DataSet data_set(1, neural_network_x.get_inputs_number(), neural_network_x.get_outputs_number());
            inputs = neural_networks[axis].get_scaling_layer_pointer()->calculate_minimum_maximum_outputs(inputs);
            for(int i = 0; i < neural_networks[axis].get_inputs_number(); ++i)
                data[i] = inputs[i];
            Vector<Statistics<double>> statistics = neural_networks[axis].get_unscaling_layer_pointer()->get_statistics();
            outputs[0] = 2 * (outputs[0] - statistics[0].minimum) / (statistics[0].maximum - statistics[0].minimum) - 1;
            for(int i = neural_network_x.get_inputs_number(); i < neural_network_x.get_inputs_number() + neural_network_x.get_outputs_number(); ++i)
                data[i] = outputs[0];
            data_set.add_instance(data);
            diff2 = clock(); if(performance) cout << "***** Time3: " << ((float)(diff2 - diff1)/CLOCKS_PER_SEC) << endl;

            if(debug) cout << "***** Target: " << data_set.get_instance(1) << endl;

            diff1 = clock();
            neural_networks[axis].get_scaling_layer_pointer()->set_scaling_method(ScalingLayer::NoScaling);
            neural_networks[axis].get_unscaling_layer_pointer()->set_unscaling_method(UnscalingLayer::NoUnscaling);
            LossIndex loss_index;
            loss_index.set_regularization_type(LossIndex::NEURAL_PARAMETERS_NORM);
            loss_index.set_data_set_pointer(&data_set);
            loss_index.set_neural_network_pointer(neural_networks + axis);
            diff2 = clock(); if(performance) cout << "***** Time4: " << ((float)(diff2 - diff1)/CLOCKS_PER_SEC) << endl;

            diff1 = clock();
            TrainingStrategy training_strategy;
            training_strategy.set(&loss_index);

            training_strategy.set_main_type("GRADIENT_DESCENT");
            GradientDescent* gdp = training_strategy.get_gradient_descent_pointer();
            gdp->set_maximum_time(0.001);
            gdp->set_maximum_iterations_number(10);
            gdp->set_display_period(10);
            gdp->set_minimum_loss_increase(1.0e-3);
            gdp->set_reserve_loss_history(true);

            /*QuasiNewtonMethod* quasi_Newton_method_pointer = training_strategy.get_quasi_Newton_method_pointer();
            quasi_Newton_method_pointer->set_maximum_time(0.001);
            quasi_Newton_method_pointer->set_maximum_iterations_number(10);
            quasi_Newton_method_pointer->set_display_period(10);
            quasi_Newton_method_pointer->set_minimum_loss_increase(1.0e-3);
            quasi_Newton_method_pointer->set_reserve_loss_history(true);*/
            training_strategy.set_display(false);
            training_strategy.perform_training();
            diff2 = clock(); if(performance) cout << "***** Time5: " << ((float)(diff2 - diff1)/CLOCKS_PER_SEC) << endl;

            diff1 = clock();
            inputs[0] = error(axis);
            inputs[1] = error_old(axis);
            inputs[2] = error_old_old(axis);
            inputs[3] = error_d(axis);
            inputs[4] = error_d_old(axis);
            inputs[5] = error_d_old_old(axis);
            //inputs[6] = orientation(axis);
            //inputs[7] = angular_velocity(axis);
            neural_networks[axis].get_scaling_layer_pointer()->set_scaling_method(ScalingLayer::MinimumMaximum);
            neural_networks[axis].get_unscaling_layer_pointer()->set_unscaling_method(UnscalingLayer::MinimumMaximum);
            outputs = neural_networks[axis].calculate_outputs(inputs);
            if(debug) cout << "***** New output: " << outputs << endl << endl;
            diff2 = clock(); if(performance) cout << "***** Time6: " << ((float)(diff2 - diff1)/CLOCKS_PER_SEC) << endl;
        }

        end = clock();
        elapsed += ((float)(end - begin)/CLOCKS_PER_SEC);
        c++;
        cout << "[DNN]: time = " << (elapsed/c) << endl;
    }
}
