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
	this->state = AIMING;
	this->gate_region = 0;
	this->altitude = .0;
	if (filter_window_size % 2) {
		filter_window_size--;
	}
	this->filter_window_size = filter_window_size;
	this->subVelocity = this->handle.subscribe("/mavros/local_position/velocity", 1000,
			&Controller::CurrentVelocityCallback, this);
	this->pubVelocity =
		this->handle.advertise<geometry_msgs::Quaternion>("/IntelDrone/command_velocity_body",
				100);
	this->pubFilteredWindow =
		this->handle.advertise<geometry_msgs::TwistStamped>("/predictor/filtered", 100);
	this->subPredictor = this->handle.subscribe("/predictor/raw", 100,
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
	if (this->filter_window.size() < this->filter_window_size) {
		this->filter_window.push_front(prediction);
		return prediction;
	}

	std::vector<int>
		sortedWindow(this->filter_window.begin(), this->filter_window.end());
	sortedWindow.push_back(prediction);
	std::sort(std::begin(sortedWindow), std::end(sortedWindow));

	if (this->filter_window.size() == this->filter_window_size)
		this->filter_window.pop_back();

	this->filter_window.push_front(prediction);

	int filtered_pred = sortedWindow.at(sortedWindow.size()/2);
	geometry_msgs::TwistStamped stupid;
	stupid.header.stamp = ros::Time::now();
	stupid.twist.linear.x = filtered_pred;
	this->pubFilteredWindow.publish(stupid);

	return filtered_pred;
}

void Controller::DynamicReconfigureCallback(PIDConfig &cfg, uint32_t level)
{
	gain_param gain_z, gain_yaw;
	gain_z.insert(std::pair<std::string, float>("p", cfg.k_p_z));
	gain_z.insert(std::pair<std::string, float>("i", cfg.k_i_z));
	gain_z.insert(std::pair<std::string, float>("d", cfg.k_d_z));

	gain_yaw.insert(std::pair<std::string, float>("p", cfg.k_p_y));
	gain_yaw.insert(std::pair<std::string, float>("i", cfg.k_i_y));
	gain_yaw.insert(std::pair<std::string, float>("d", cfg.k_d_y));

	this->PIDBoy->SetGainParameters(gain_z, gain_yaw, cfg.x_vel);
}

void Controller::GatePredictionCallback(const GatePredictionMessage &msg)
{
	this->gate_region = this->FilterPrediction(msg.window);
}

void Controller::CurrentVelocityCallback(geometry_msgs::TwistStampedConstPtr msg)
{
	this->current_velocity.linear = Vector3d(msg->twist.linear.x, msg->twist.linear.y,
			msg->twist.linear.z);
	this->current_velocity.yaw = msg->twist.angular.z;
}


void Controller::HeightSensorCallback(const Vector3Ptr &msg)
{
	//this->altitude = msg;
}

void Controller::PublishVelocity(Vector3d velocity)
{
	geometry_msgs::Quaternion quat;
	quat.x = velocity.x;
	quat.y = velocity.y;
	quat.z = velocity.z;
	/*geometry_msgs::TwistStamped twistStamped;
	twistStamped.twist.linear.x = velocity.x;
	twistStamped.twist.linear.y = velocity.y;
	twistStamped.twist.linear.z = velocity.z;
	twistStamped.header.stamp = ros::Time::now();*/
	this->pubVelocity.publish(quat);
}

void Controller::PublishVelocity(Velocity velocity)
{
	geometry_msgs::Quaternion quat;
	quat.x = velocity.linear.x;
	quat.y = velocity.linear.y;
	quat.z = velocity.linear.z;
	quat.w = velocity.yaw;
	/*geometry_msgs::TwistStamped twistStamped;
	twistStamped.twist.linear.x = velocity.x;
	twistStamped.twist.linear.y = velocity.y;
	twistStamped.twist.linear.z = velocity.z;
	twistStamped.header.stamp = ros::Time::now();*/
	this->pubVelocity.publish(quat);
}

void Controller::PublishVelocity(float yawVelocity)
{
	geometry_msgs::Quaternion quat;
	quat.w = yawVelocity;
	/*geometry_msgs::TwistStamped twistStamped;
	twistStamped.twist.angular.z = yawVelocity;
	twistStamped.header.stamp = ros::Time::now();*/
	this->pubVelocity.publish(quat);
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
	ros::Time startLeavingTime, startTakeOffTime, startCrossingTime;

	while (ros::ok()) {
		rate.sleep();
		ros::spinOnce();

		switch (this->state) {
			case LANDED:
				{
					this->PublishVelocity(Vector3d(0, 0, 0));
					std::cout << "[*] Press <ENTER> to start flying" << std::endl;
					if (std::cin.get()) {
						std::cout << "[*] Taking off!" << std::endl;
						startTakeOffTime = ros::Time::now();
						this->state = TAKEOFF;
					}
				}
			case TAKEOFF:
				{
					/* Take off */
					//if (this->altitude < 200) {
					ros::Duration d = ros::Time::now() - startTakeOffTime;
					if (d.toSec() < 5) {
						Vector3d velocity(0, 0, 0.5);
						this->PublishVelocity(velocity);
					} else {
						std::cout << "[*] Aiming" << std::endl;
						this->PublishVelocity(Vector3d(0, 0, 0));
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
						this->PublishVelocity(0);
						gate_center = this->ComputeGateCenter();
						std::cout << "[*] Flying towards gate center: " <<
							gate_center.x << "," << gate_center.y << "]" <<
							std::endl;
						this->state = FLYING;
					}
					break;
				}
			case REFINING:
				{
					bool crossing = true;
					for (auto windowIt = this->filter_window.begin(); windowIt
							!= this->filter_window.end(); windowIt++ ) {
						if (*windowIt != 13) {
							crossing = false;
							break;
						}
					}
					if (crossing) {
						std::cout << "[*] Crossing the gate, watch out !" << std::endl;
						startCrossingTime = ros::Time::now();
						this->state = CROSSING;
					} else if (this->gate_region != 0) {
						this->PublishVelocity(Vector3d(0, 0, 0));
						gate_center = this->ComputeGateCenter();
						std::cout << "[*] Flying towards window " <<
							this->gate_region << std::endl;
						this->state = FLYING;
					}
					break;
				}
						std::cout << "[*] Aiming" << std::endl;
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
						this->PublishVelocity(Vector3d(0, 0, 0));
						std::cout << "[*] Correcting course..." << std::endl;
						this->state = REFINING;
						//std::cout << "[*] Aiming" << std::endl;
						//this->state = AIMING;
					}
					break;
				}
			case CROSSING:
				{
					//if (this->altitude > MAX_GATE_HEIGHT) {
					ros::Duration timeElapsed = ros::Time::now() - startCrossingTime;
					if (timeElapsed.toSec() < CROSSING_TIME) {
						this->PublishVelocity(Vector3d(0.3, 0, 0));
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
