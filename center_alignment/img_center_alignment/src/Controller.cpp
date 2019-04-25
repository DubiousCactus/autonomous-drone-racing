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
	this->state = LANDED;
	this->gate_region = 0;
	this->altitude = .0;
	this->subVelocity = this->handle.subscribe("~local_position/velocity", 1000,
			&Controller::CurrentVelocityCallback, this);
	this->pubVelocity =
		this->handle.advertise<geometry_msgs::TwistStamped>("/uav/command_velocity",
				100);
}

Controller::~Controller()
{
	delete this->PIDBoy;
}

void Controller::GatePredictionCallback(const GatePredictionMessagePtr &msg)
{
	// TODO: Compute pixel coordinates in the image frame (remember the scaling!)
	// TODO: Substract the image center from each axis
	//this->gate_region = msg.region;
}

void Controller::CurrentVelocityCallback(geometry_msgs::TwistStampedConstPtr msg)
{
	this->current_velocity << msg->twist.linear.x, msg->twist.linear.y,
		msg->twist.linear.z;
}

void Controller::HeightSensorCallback(const Vector3Ptr &msg)
{
	//this->altitude = msg;
}

void Controller::PublishVelocity(Eigen::Vector3d velocity)
{
	geometry_msgs::TwistStampedPtr twistStamped;
	twistStamped->twist.linear.x = velocity.x();
	twistStamped->twist.linear.y = velocity.y();
	twistStamped->twist.linear.z = velocity.z();
	this->pubVelocity.publish(twistStamped);
}

void Controller::PublishVelocity(float yawVelocity)
{
	geometry_msgs::TwistStamped twistStamped;
	twistStamped.twist.angular.z = yawVelocity;
	this->pubVelocity.publish(twistStamped);
}

Eigen::Vector3d Controller::ComputeGateCenter()
{
	int window_size = sqrt(NB_WINDOWS);
	int window_width = IMG_WIDTH/window_size;
	int window_height = IMG_HEIGHT/window_size;
	int window_xindex = this->gate_region % window_size;
	if (window_xindex == 0)
		window_xindex = window_size;
	int window_x = (window_xindex - 1) * window_width;
	int window_y = window_height * (this->gate_region/window_size);
	return Eigen::Vector3d(
			window_x + (window_width/2),
			window_y + (window_height/2),
			0);
}

void Controller::Run()
{
	int tick = 0;
	ros::Rate rate(this->rate);
	Eigen::Vector3d gate_center;
	Eigen::Vector3d origin(IMG_WIDTH/2, IMG_HEIGHT/2);

	while (ros::ok()) {
		rate.sleep();
		ros::spinOnce();

		switch (this->state) {
			case LANDED:
				{
					/* Take off */
					while (this->altitude < 200) {
						Eigen::Vector3d velocity(0, 0, 0.1);
						this->PublishVelocity(velocity);
					}
					this->state = AIMING;
				}
			case AIMING:
				{
					while (this->gate_region == 0) {
						// Yaw velocity of 0.1
						this->PublishVelocity(0.05);
					}
					gate_center = this->ComputeGateCenter();
					this->state = FLYING;
					break;
				}
			case FLYING:
				{
					/* Compute the gate error */
					Eigen::Vector3d gate_err = gate_center - origin;

					/* Compute the velocity from the PID controller */
					auto velocity = this->PIDBoy->Compute(gate_err,
							this->current_velocity);

					/* Apply the velocity or send it to the drone */
					this->PublishVelocity(velocity);

					if (tick >= DETECTION_RATE) {
						tick = 0;
						this->state = AIMING;
					}
					//this->state = CROSSING;
					break;
				}
			case CROSSING:
				{
					// TODO: If the height sensor indicates we're in a gate
					this->state = LEAVING;
				}
				break;
			case LEAVING:
				// TODO: fly straight ahead for 1 meter
				break;
		}
		tick++;
	}
}

int main(int argc, char **argv)
{
	std::cout << "[*] Running the controller..." << std::endl;
	ros::init(argc, argv, "Controller");

	// TODO: Read from config
	gain_param k_x, k_y;
	k_x.at("p") = 0.5;
	k_x.at("i") = 0.5;
	k_x.at("d") = 0.5;

	k_y.at("p") = 0.5;
	k_y.at("i") = 0.5;
	k_y.at("d") = 0.5;

	Controller controller(k_x, k_y, 0.05); // TODO: create maps from config
	controller.Run();

	ros::shutdown();


	return 0;
}
