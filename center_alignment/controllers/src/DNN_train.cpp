#include <iostream>
#include <string>
#include <math.h>
#include <time.h>
#include <Eigen/Dense>
#include <tensorflow/c/c_api.h>
#include <opennn.h>

using namespace OpenNN;

int main(int argc, char** argv){
    printf("Hello from TensorFlow C library version %s\n", TF_Version());

    std::string axis("x");

    try{
        std::cout << "OpenNN. Train DNN." << std::endl;

        srand((unsigned)time(NULL));

        // Global variables
        DataSet data_set;
        NeuralNetwork neural_network;
        LossIndex loss_index;
        TrainingStrategy training_strategy;

        // Data set
        data_set.set_data_file_name("/home/andriy/catkin_ws/src/controllers/data/pid_bebop_" + axis + ".dat");
        data_set.set_separator("Space");
        data_set.load_data();

        // Variables
        Variables* variables_pointer = data_set.get_variables_pointer();
        Vector< Variables::Item > variables_items(9);
        variables_items[0].name = "error_" + axis;
        variables_items[0].units = "meters";
        variables_items[0].use = Variables::Input;
        variables_items[1].name = "error_" + axis + "_old";
        variables_items[1].units = "meters";
        variables_items[1].use = Variables::Input;
        variables_items[2].name = "error_" + axis + "_old_old";
        variables_items[2].units = "meters";
        variables_items[2].use = Variables::Input;
        variables_items[3].name = "d_error_" + axis;
        variables_items[3].units = "meters per second";
        variables_items[3].use = Variables::Input;
        variables_items[4].name = "d_error_" + axis + "old";
        variables_items[4].units = "meters per second";
        variables_items[4].use = Variables::Input;
        variables_items[5].name = "d_error_" + axis + "_old_old";
        variables_items[5].units = "meters per second";
        variables_items[5].use = Variables::Input;
        if(!strcmp(axis.c_str(), "x")){
            variables_items[6].name = "pitch";
            variables_items[6].units = "radiants";
            variables_items[6].use = Variables::Unused;
            variables_items[7].name = "pitch_rate";
            variables_items[7].units = "radiants per second";
            variables_items[7].use = Variables::Unused;
            variables_items[8].name = "command_pitch";
            variables_items[8].units = "radiants";
            variables_items[8].use = Variables::Target;
        }else if(!strcmp(axis.c_str(), "y")){
            /*variables_items[6].name = "roll";
            variables_items[6].units = "radiants";
            variables_items[6].use = Variables::Unused;
            variables_items[7].name = "roll_rate";
            variables_items[7].units = "radiants per second";
            variables_items[7].use = Variables::Unused;*/
            variables_items[6].name = "command_roll";
            variables_items[6].units = "radiants";
            variables_items[6].use = Variables::Target;
        }else{
            /*variables_items[6].name = "thrust_old";
            variables_items[6].units = "percents";
            variables_items[6].use = Variables::Unused;
            variables_items[7].name = "thrust_old_old";
            variables_items[7].units = "percents";
            variables_items[7].use = Variables::Unused;*/
            variables_items[6].name = "command_thrust";
            variables_items[6].units = "percents";
            variables_items[6].use = Variables::Target;
        }
        variables_pointer->set_items(variables_items);

        const OpenNN::Matrix<std::string> inputs_information = variables_pointer->arrange_inputs_information();
        const OpenNN::Matrix<std::string> targets_information = variables_pointer->arrange_targets_information();

        // Instances
        Instances* instances_pointer = data_set.get_instances_pointer();
        instances_pointer->split_random_indices();

        const Vector< Statistics<double> > inputs_statistics = data_set.scale_inputs_minimum_maximum();
        const Vector< Statistics<double> > targets_statistics = data_set.scale_targets_minimum_maximum();

        // Neural network
        Vector<size_t> architecture(4);
        architecture[0] = variables_pointer->count_inputs_number();
        architecture[1] = 64;
        architecture[2] = 64;
        architecture[3] = variables_pointer->count_targets_number();
        neural_network.set(architecture);

        Inputs* inputs = neural_network.get_inputs_pointer();
        inputs->set_information(inputs_information);

        Outputs* outputs = neural_network.get_outputs_pointer();
        outputs->set_information(targets_information);

        neural_network.construct_scaling_layer();
        ScalingLayer* scaling_layer_pointer = neural_network.get_scaling_layer_pointer();
        scaling_layer_pointer->set_statistics(inputs_statistics);
        scaling_layer_pointer->set_scaling_method(ScalingLayer::NoScaling);

        neural_network.construct_unscaling_layer();
        UnscalingLayer* unscaling_layer_pointer = neural_network.get_unscaling_layer_pointer();
        unscaling_layer_pointer->set_statistics(targets_statistics);
        unscaling_layer_pointer->set_unscaling_method(UnscalingLayer::NoUnscaling);

        // Loss index
        loss_index.set_data_set_pointer(&data_set);
        loss_index.set_neural_network_pointer(&neural_network);
        loss_index.set_regularization_type(LossIndex::NEURAL_PARAMETERS_NORM);

        // Training strategy object
        training_strategy.set(&loss_index);
        QuasiNewtonMethod* quasi_Newton_method_pointer = training_strategy.get_quasi_Newton_method_pointer();
        quasi_Newton_method_pointer->set_maximum_time(1000);
        quasi_Newton_method_pointer->set_maximum_iterations_number(1000);
        quasi_Newton_method_pointer->set_display_period(10);
        quasi_Newton_method_pointer->set_minimum_loss_increase(1.0e-6);
        quasi_Newton_method_pointer->set_reserve_loss_history(true);

        TrainingStrategy::Results training_strategy_results = training_strategy.perform_training();

        // Testing analysis
        TestingAnalysis testing_analysis(&neural_network, &data_set);
        TestingAnalysis::LinearRegressionResults linear_regression_results = testing_analysis.perform_linear_regression_analysis();

        // Save results
        scaling_layer_pointer = neural_network.get_scaling_layer_pointer();
        unscaling_layer_pointer = neural_network.get_unscaling_layer_pointer();
        scaling_layer_pointer->set_scaling_method(ScalingLayer::MinimumMaximum);
        unscaling_layer_pointer->set_unscaling_method(UnscalingLayer::MinimumMaximum);

        data_set.save("/home/andriy/catkin_ws/src/controllers/data/" + axis + "/data_set_bebop.xml");
        neural_network.save("/home/andriy/catkin_ws/src/controllers/data/" + axis + "/neural_network_bebop.xml");
        neural_network.save_expression("/home/andriy/catkin_ws/src/controllers/data/" + axis + "/expression_bebop.txt");
        loss_index.save("/home/andriy/catkin_ws/src/controllers/data/" + axis + "/loss_index_bebop.xml");
        training_strategy.save("/home/andriy/catkin_ws/src/controllers/data/" + axis + "/training_strategy_bebop.xml");
        training_strategy_results.save("/home/andriy/catkin_ws/src/controllers/data/" + axis + "/training_strategy_results_bebop.dat");
        linear_regression_results.save("/home/andriy/catkin_ws/src/controllers/data/" + axis + "/linear_regression_analysis_results_bebop.dat");

        return(0);
    }
    catch(std::exception& e){
        std::cerr << e.what() << std::endl;
        return(1);
    }
}
