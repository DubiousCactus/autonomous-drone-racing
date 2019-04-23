/*
 * Controller.cpp
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#include "Controller.h"
#include <iostream>


Controller::Controller(gain_param k_x, gain_param k_y, float z_velocity)
{
	this->rate = 100;
	this->PIDBoy = new PID(k_x, k_y, z_velocity, rate);
	this->state = AIMING;
	this->gate_region = 0;
}

Controller::~Controller()
{
	delete this->PIDBoy;
}

void Controller::GatePredictionCallback(const GatePredictionMessagePtr &msg)
{
	//this->gate_region = msg.region;
}

void Controller::CurrentVelocityCallback(const Vector3Ptr &msg)
{
	//this->current_velocity = msg; // TODO: Maybe convert msg to vector
}

void Controller::Run()
{
	ros::Rate rate(this->rate);
	Eigen::Vector3d gate_center;

	while (ros::ok()) {
		rate.sleep();
		ros::spinOnce();

		switch (this->state) {
			case AIMING:
				{
					while (this->gate_region == 0)
						// TODO: Yaw around
					// gate_center = ...; TODO: Compute region center
					break;
				}
			case FLYING:
				{
					/* Compute the gate error */
					Eigen::Vector3d gate_err; // TODO

					/* Compute the velocity from the PID controller */
					auto velocity = this->PIDBoy->Compute(gate_err, this->current_velocity);

					/* Apply the velocity or send it to the drone */
					// TODO
					break;
				}
			case CROSSING:
				break;
		}
	}
}

int main(int argc, char **argv)
{
	std::cout << "[*] Running the controller..." << std::endl;
	ros::init(argc, argv, "Controller");
	/*Controller controller(k_x, k_y, z_velocity); // TODO: create maps from config
	controller.Run();*/
	ros::shutdown();


	return 0;
}
