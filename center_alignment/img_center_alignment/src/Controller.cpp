/*
 * Controller.cpp
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#include <iostream>
#include <assert.h>
#include <algorithm>

#include "Controller.h"

Controller::Controller(gain_param k_x, gain_param k_y, float z_velocity)
{
	this->ready = false;
	this->rate = CONTROLLER_RATE;
	this->PIDBoy = new PID(k_x, k_y, z_velocity, rate);
	this->state = LANDED;
	this->gate_center.locked = false;
	this->altitude = .0;
	this->subVelocity = this->handle.subscribe("/mavros/local_position/velocity", 1000,
			&Controller::CurrentVelocityCallback, this);
	this->pubVelocity =
		this->handle.advertise<geometry_msgs::Quaternion>(
				"/IntelDrone/command_velocity_body", 100);
	this->subPredictor = this->handle.subscribe("/predictor/raw", 1000,
			&Controller::GatePredictionCallback, this);

	this->dynRcfgServer.setCallback(
			boost::bind(&Controller::DynamicReconfigureCallback, this, _1, _2));
}


Controller::~Controller()
{
	delete this->PIDBoy;
}

void Controller::DynamicReconfigureCallback(PIDConfig &cfg, uint32_t level)
{
	gain_param gain_z, gain_yaw;
	gain_z.insert(std::pair<std::string, float>("p", cfg.k_p_z));
	gain_z.insert(std::pair<std::string, float>("i", cfg.k_i_z));
	gain_z.insert(std::pair<std::string, float>("d", cfg.k_d_z));

	gain_yaw.insert(std::pair<std::string, float>("p", cfg.k_p_yaw));
	gain_yaw.insert(std::pair<std::string, float>("i", cfg.k_i_yaw));
	gain_yaw.insert(std::pair<std::string, float>("d", cfg.k_d_yaw));

	this->PIDBoy->SetGainParameters(gain_z, gain_yaw, cfg.x_vel);
}

void Controller::GatePredictionCallback(const perception::GatePrediction::ConstPtr &msg)
{
	this->gate_center.locked = msg->locked;
	this->gate_center.bbox = *msg;
	if (!msg->locked) {
		this->gate_center.x = 0;
		this->gate_center.y = 0;
	} else {
		this->gate_center.x = (msg->bbox.maxX + msg->bbox.minX)/2;
		this->gate_center.x = (this->gate_center.x * CAM_WIDTH)/IMG_WIDTH;
		this->gate_center.y = (msg->bbox.maxY + msg->bbox.minY)/2;
		this->gate_center.y = (this->gate_center.y * CAM_HEIGHT)/IMG_HEIGHT;
	}
	if (this->previous_predictions.size() >= PREVIOUS_PREDICTIONS_CNT) {
		this->previous_predictions.pop_back();
	}
	this->previous_predictions.push_front(msg->bbox);
	this->ready = true;
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
	this->pubVelocity.publish(quat);
}

void Controller::PublishVelocity(Velocity velocity)
{
	geometry_msgs::Quaternion quat;
	quat.x = velocity.linear.x;
	quat.y = velocity.linear.y;
	quat.z = velocity.linear.z;
	quat.w = velocity.yaw;
	this->pubVelocity.publish(quat);
}

void Controller::PublishVelocity(float yawVelocity)
{
	geometry_msgs::Quaternion quat;
	quat.w = yawVelocity;
	this->pubVelocity.publish(quat);
}


void Controller::ClearCalibration()
{
	std::queue<float> empty;
	std::swap(this->previous_ratios, empty);
	this->ref_gate.ratio = 0;
	this->ref_gate.width = 0;
	this->ref_gate.height = 0;
}


bool Controller::CrossingCondition()
{
	int prevArea;
	bool crossing = false;

	if (this->previous_predictions.size() >= PREVIOUS_PREDICTIONS_CNT
			&& !this->gate_center.locked) {
		crossing = true;
		/* The first one is the current prediction, which we don't need */
		this->previous_predictions.pop_front();
		for (auto bboxIt = this->previous_predictions.begin(); bboxIt
				!= this->previous_predictions.end(); ++bboxIt) {
			int area = (bboxIt->maxX - bboxIt->minX)*(bboxIt->maxY - bboxIt->minY);
			if (area < prevArea && bboxIt != this->previous_predictions.begin()) {
				crossing = false;
				break;
			}
			prevArea = area;
		}
		this->previous_predictions.clear();
	}

	return crossing;
}


void Controller::Run()
{
	int tick = 0;
	ros::Rate rate(this->rate);
	Vector3d origin(CAM_WIDTH/2, CAM_HEIGHT/2);
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
					if (!this->gate_center.locked) {
						// Yaw velocity
						this->PublishVelocity(0.05);
					} else {
						/*std::cout << "[*] Flying towards gate center: " <<
							this->gate_center.x << "," << this->gate_center.y << "]" <<
							std::endl;*/
						this->ClearCalibration();
						this->state = CALIBRATING;
					}
					break;
				}
			case CALIBRATING:
				{
					/* Init the list */
					int width = this->gate_center.bbox.maxX -
						this->gate_center.bbox.minX;
					int height = this->gate_center.bbox.maxY -
						this->gate_center.bbox.minY;
					float ratio = width / height;
					int meanRatio, meanWidth, meanHeight;
					bool valid = true;
					meanRatio = meanWidth = meanHeight = 0;

					if (this->ref_gate_buffer.empty()) {
						this->ref_gate_buffer.push(Gate(ratio, height, width));
						break;
					}

					for (auto prevA = this->ref_gate_buffer.begin();
							prevA != this->ref_gate_buffer.end(); ++prevA) {
						if (abs(prevA->ratio/ratio) > CALIBRATION_ERROR_THRESHOLD) {
							this->ref_gate_buffer.pop();
							valid = false;
							break;
						}
						meanRatio += prevA->ratio;
						meanWidth += prevA->width;
						meanHeight += prevA->height;
					}

					meanRatio /= CALIBRATION_QUEUE_SIZE;
					meanWidth /= CALIBRATION_QUEUE_SIZE;
					meanHeight /= CALIBRATION_QUEUE_SIZE;

					if (valid) {
						this->ref_gate_buffer.push(Gate(ratio, height, width));
					}

					if (this->ref_gate_buffer.size() == CALIBRATION_QUEUE_SIZE) {
						std::cout << "[*] Target gate calibrated !" << std::endl;
						this->ref_gate.ratio = meanRatio;
						this->ref_gate.width = meanWidth;
						this->ref_gate.height = meanHeight;
						this->state = FLYING;
					}
					break;
				}
			case REFINING:
				{
					/*if (!this->ready) {
						this->state = WAITING;
						break;
					}*/

					this->ready = false;
					bool crossing = this->CrossingCondition();
					if (crossing) {
						std::cout << "[*] Crossing the gate, watch out !" << std::endl;
						startCrossingTime = ros::Time::now();
						this->state = CROSSING;
					} else if (this->gate_center.locked) {
						std::cout << "[*] Flying towards coords [" <<
							this->gate_center.x << "," << this->gate_center.y <<
							"]" << std::endl;
						this->state = FLYING;
					} else {
						std::cout << "[*] Gate lost! Re-targetting..." << std::endl;
						this->state = AIMING;
					}
					break;
				}
			case FLYING:
				{
					assert(this->ref_gate.ratio != 0);
					assert(this->ref_gate.width != 0);
					assert(this->ref_gate.height != 0);
					/* Compute the gate error */
					float ratio = GET_RATIO();

					Vector3d center(this->gate_center.x, this->gate_center.y, 0);
					if (abs(ratio / this->ref_gate.ratio) > CALIBRATION_ERROR_THRESHOLD) {
						/* Compensate for the shift */
						center.x -= GET_X_SHIFT();
						center.y -= GET_Y_SHIFT();
					}
					Vector3d gate_err = origin - center;

					/* Compute the velocity from the PID controller */
					auto velocity = this->PIDBoy->Compute(gate_err,
							this->current_velocity);

					/* TODO: Maybe it's better to compute the velocity until
 					 * the error is 0, and then switch to the WAITING state ?
					 * */
					/* Apply the velocity or send it to the drone */
					this->PublishVelocity(velocity);
					std::cout << "[*] Aligning" << std::endl;

					if (++tick >= DETECTION_RATE) {
						tick = 0;
						std::cout << "[*] Correcting course..." << std::endl;
						this->state = REFINING;
					}
					break;
				}
			case CROSSING:
				{
					//if (this->altitude > MAX_GATE_HEIGHT) {
					ros::Duration timeElapsed = ros::Time::now() - startCrossingTime;
					if (timeElapsed.toSec() < CROSSING_TIME) {
						this->PublishVelocity(Vector3d(0.5, 0, 0));
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
					if (timeElapsed.toSec() < LEAVING_TIME) {
						this->PublishVelocity(Vector3d(0.5, 0, 0));
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
			case WAITING:
				{
					this->PublishVelocity(Vector3d(0, 0, 0));
					if (this->ready)
						this->state = REFINING;
				}
		}
	}
}

int main(int argc, char **argv)
{
	std::cout << "[*] Running the controller..." << std::endl;
	ros::init(argc, argv, "Controller");

	// TODO: Read from config
	gain_param k_x, k_y;
	k_x.insert(std::pair<std::string, float>("p", 0.5));
	k_x.insert(std::pair<std::string, float>("i", 0.1));
	k_x.insert(std::pair<std::string, float>("d", 0.2));

	k_y.insert(std::pair<std::string, float>("p", 0.2));
	k_y.insert(std::pair<std::string, float>("i", 0.0));
	k_y.insert(std::pair<std::string, float>("d", 0.0));

	Controller controller(k_x, k_y, 0.5);
	controller.Run();

	ros::shutdown();


	return 0;
}
