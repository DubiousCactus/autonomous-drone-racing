/*
 * Controller.cpp
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#include "Controller.h"
#include <iostream>
#include <algorithm>


Controller::Controller(gain_param k_x, gain_param k_y, float z_velocity, int
		filter_window_size)
{
	this->rate = 100;
	this->PIDBoy = new PID(k_x, k_y, z_velocity, rate);
	this->state = LANDED;
	this->gate_region = 0;
	this->altitude = .0;
	if (filter_window_size % 2) {
		filter_window_size--;
	}
	this->filter_window.resize(filter_window_size);
	this->subVelocity = this->handle.subscribe("/local_position/velocity", 1000,
			&Controller::CurrentVelocityCallback, this);
	this->pubVelocity =
		this->handle.advertise<geometry_msgs::TwistStamped>("/uav/command_velocity",
				100);
	this->subPredictor = this->handle.subscribe("/predictor", 100,
			&Controller::GatePredictionCallback, this);

	this->dynRcfgServer.setCallback(
			boost::bind(&Controller::DynamicReconfigureCallback, this, _1, _2));
}


Controller::~Controller()
{
	delete this->PIDBoy;
}

/* Simple Median Filter implementation */
int Controller::FilterPrediction(int prediction)
{
	if (this->filter_window.size() < this->filter_window.max_size())
		return prediction;

	if (this->filter_window.size() == this->filter_window.max_size())
		this->filter_window.pop_back();

	std::deque<int> sortedWindow = this->filter_window;
	sortedWindow.resize(sortedWindow.max_size() + 1);
	sortedWindow.push_front(prediction);
	std::sort(std::begin(sortedWindow), std::end(sortedWindow));
	this->filter_window.push_front(prediction);

	return sortedWindow.at(sortedWindow.size()/2);
}

void Controller::DynamicReconfigureCallback(PIDConfig &cfg, uint32_t level)
{
	gain_param gain_z, gain_y;
	gain_z.insert(std::pair<std::string, float>("p", cfg.k_p_z));
	gain_z.insert(std::pair<std::string, float>("i", cfg.k_i_z));
	gain_z.insert(std::pair<std::string, float>("d", cfg.k_d_z));

	gain_y.insert(std::pair<std::string, float>("p", cfg.k_p_y));
	gain_y.insert(std::pair<std::string, float>("i", cfg.k_i_y));
	gain_y.insert(std::pair<std::string, float>("d", cfg.k_d_y));

	this->PIDBoy->SetGainParameters(gain_z, gain_y, cfg.x_vel);
}

void Controller::GatePredictionCallback(const GatePredictionMessage &msg)
{
	this->gate_region = this->FilterPrediction(msg.window);
}

void Controller::CurrentVelocityCallback(geometry_msgs::TwistStampedConstPtr msg)
{
	this->current_velocity = Vector3d(msg->twist.linear.x, msg->twist.linear.y,
			msg->twist.linear.z);
}


void Controller::HeightSensorCallback(const Vector3Ptr &msg)
{
	//this->altitude = msg;
}

void Controller::PublishVelocity(Vector3d velocity)
{
	geometry_msgs::TwistStamped twistStamped;
	twistStamped.twist.linear.x = velocity.x;
	twistStamped.twist.linear.y = velocity.y;
	twistStamped.twist.linear.z = velocity.z;
	twistStamped.header.stamp = ros::Time::now();
	this->pubVelocity.publish(twistStamped);
}

void Controller::PublishVelocity(float yawVelocity)
{
	geometry_msgs::TwistStamped twistStamped;
	twistStamped.twist.angular.z = yawVelocity;
	twistStamped.header.stamp = ros::Time::now();
	this->pubVelocity.publish(twistStamped);
}

Vector3d Controller::ComputeGateCenter()
{
	int window_size = sqrt(NB_WINDOWS);
	int window_width = IMG_WIDTH/window_size;
	int window_height = IMG_HEIGHT/window_size;
	int window_xindex = this->gate_region % window_size;
	if (window_xindex == 0)
		window_xindex = window_size;
	int window_x = (window_xindex - 1) * window_width;
	int window_y = window_height * (this->gate_region/window_size);
	return Vector3d(
			window_x + (window_width/2),
			window_y + (window_height/2),
			0);
}

void Controller::Run()
{
	int tick = 0;
	ros::Rate rate(this->rate);
	Vector3d gate_center;
	Vector3d origin(IMG_WIDTH/2, IMG_HEIGHT/2);
	ros::Time startLeavingTime;

	while (ros::ok()) {
		rate.sleep();
		ros::spinOnce();

		switch (this->state) {
			case LANDED:
				{
					std::cout << "[*] Press <ENTER> to start flying" << std::endl;
					if (std::cin.get()) {
						std::cout << "[*] Taking off!" << std::endl;
						this->state = TAKEOFF;
					}
				}
			case TAKEOFF:
				{
					/* Take off */
					if (this->altitude < 200) {
						Vector3d velocity(0, 0, 0.1);
						this->PublishVelocity(velocity);
					} else {
						std::cout << "[*] Aiming" << std::endl;
						this->state = AIMING;
					}
					break;
				}
			case AIMING:
				{
					if (this->gate_region == 0) {
						// Yaw velocity
						this->PublishVelocity(0.05);
					} else {
						gate_center = this->ComputeGateCenter();
						std::cout << "[*] Flying towards target" << std::endl;
						this->state = FLYING;
					}
					break;
				}
			case REFINING:
				{
					if (this->gate_region == 0) {
						std::cout << "[*] Crossing the gate, watch out !" << std::endl;
						this->state = CROSSING;
					} else {
						gate_center = this->ComputeGateCenter();
						this->state = FLYING;
					}
					break;
				}
			case FLYING:
				{
					/* Compute the gate error */
					Vector3d gate_err = origin - gate_center;

					/* Compute the velocity from the PID controller */
					auto velocity = this->PIDBoy->Compute(gate_err,
							this->current_velocity);

					/* Apply the velocity or send it to the drone */
					this->PublishVelocity(velocity);

					if (tick >= DETECTION_RATE) {
						tick = 0;
						std::cout << "[*] Correcting course..." << std::endl;
						this->state = REFINING;
					}
					break;
				}
			case CROSSING:
				{
					if (this->altitude > MAX_GATE_HEIGHT) {
						this->PublishVelocity(Vector3d(0.1, 0, 0));
					} else {
						std::cout << "[*] Leaving the gate" << std::endl;
						startLeavingTime = ros::Time::now();
						this->state = LEAVING;
					}
					break;
				}
			case LEAVING:
				{
					ros::Duration timeElapsed = ros::Time::now() - startLeavingTime;
					if (timeElapsed.toSec() < CROSSING_TIME) {
						this->PublishVelocity(Vector3d(0.1, 0, 0));
					} else {
						std::cout << "[*] Targetting new gate" << std::endl;
						this->state = AIMING;
					}
					break;
				}
			case LANDING:
				{
					// TODO: Land
					 break;
				}
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
	k_x.insert(std::pair<std::string, float>("p", 0.5));
	k_x.insert(std::pair<std::string, float>("i", 0.5));
	k_x.insert(std::pair<std::string, float>("d", 0.5));

	k_y.insert(std::pair<std::string, float>("p", 0.5));
	k_y.insert(std::pair<std::string, float>("i", 0.5));
	k_y.insert(std::pair<std::string, float>("d", 0.5));

	Controller controller(k_x, k_y, 0.05, 30);
	controller.Run();

	ros::shutdown();


	return 0;
}
