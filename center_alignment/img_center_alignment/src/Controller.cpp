/*
 * Controller.cpp
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#include <iostream>
#include <assert.h>
#include <algorithm>

#include "img_center_alignment/Controller.h"

Controller::Controller(gain_param k_x, gain_param k_y, float z_velocity)
{
	this->gate_losses = 0;
	this->ready = false;
	this->detectionReady = false;
	this->rate = CONTROLLER_RATE;
	this->PIDBoy = new PID(k_x, k_y, z_velocity, rate);
	this->state = LANDED;
	this->gate_center.locked = false;
	this->altitude = .0;
	this->ClearCalibration();
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
	this->gate_center.bbox = msg->bbox;
	if (!msg->locked) {
		this->gate_center.x = 0;
		this->gate_center.y = 0;
	} else {
		this->gate_center.x = (msg->bbox.maxX + msg->bbox.minX)/2;
		this->gate_center.x = (this->gate_center.x * CAM_WIDTH)/IMG_WIDTH;
		this->gate_center.y = (msg->bbox.maxY + msg->bbox.minY)/2;
		this->gate_center.y = (this->gate_center.y * CAM_HEIGHT)/IMG_HEIGHT;
		this->gate_center.distance = msg->distance;
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
	this->ref_gate_buffer.clear();
	this->ref_gate.ratio = 0;
	this->ref_gate.width = 0;
	this->ref_gate.height = 0;
}


void Controller::SetState(State state)
{
	this->state = state;
}


void Controller::SetGateCenter(Prediction center)
{
	this->gate_center = center;
}


Vector3d Controller::GetAlignmentError()
{
	return this->alignment_error;
}


State Controller::GetState()
{
	return this->state;
}


Gate Controller::GetRefGate()
{
	return this->ref_gate;
}


std::list<Gate> Controller::GetRefGateBuffer()
{
	return this->ref_gate_buffer;
}


void Controller::Calibrate()
{
	/* Init the list */
	int width = this->gate_center.bbox.maxX -
		this->gate_center.bbox.minX;
	int height = this->gate_center.bbox.maxY -
		this->gate_center.bbox.minY;
	float ratio = (float)width / (float)height;
	assert(ratio != 0 && "target ratio is 0!");
	float meanRatio;
	int meanWidth, meanHeight;
	bool valid = true;
	meanRatio = meanWidth = meanHeight = 0;

	if (!this->gate_center.locked) {
		return;
	}

	if (this->ref_gate_buffer.empty()) {
		Gate gate = { ratio, width, height };
		this->ref_gate_buffer.push_front(gate);
		return;
	}

	for (auto prevA = this->ref_gate_buffer.begin();
			prevA != this->ref_gate_buffer.end(); ++prevA) {
		if (abs(1-(prevA->ratio/ratio)) >
				CALIBRATION_ERROR_THRESHOLD) {
			if (this->ref_gate_buffer.size() > 1) {
				this->ref_gate_buffer.pop_back();
			}
			valid = false;
			break;
		}
		assert(prevA->ratio != 0 && "prevA ratio is 0!");
		meanWidth += prevA->width;
		meanHeight += prevA->height;
	}

	if (valid) {
		Gate gate = { ratio, width, height };
		this->ref_gate_buffer.push_front(gate);
		meanWidth += width;
		meanHeight += height;
	} else {
		this->ClearCalibration();
		return;
	}

	if (valid && this->ref_gate_buffer.size() ==
			CALIBRATION_QUEUE_SIZE) {
		meanWidth /= CALIBRATION_QUEUE_SIZE;
		meanHeight /= CALIBRATION_QUEUE_SIZE;
		meanRatio = (float)meanWidth/(float)meanHeight;

		std::cout << "[*] Target gate calibrated ! Mean ratio: "
			<< meanRatio << " - Mean width: " << meanWidth
			<< " - Mean height: " << meanHeight << std::endl;
		std::cout << "[*] Flying towards gate center: " <<
			this->gate_center.x << "," << this->gate_center.y
			<< "]" << std::endl;

		this->ref_gate.ratio = meanRatio;
		this->ref_gate.width = meanWidth;
		this->ref_gate.height = meanHeight;
		this->state = FLYING;
	}
}


void Controller::DetectionRate()
{
	ros::Rate rate(DETECTION_RATE);
	while(ros::ok()) {
		rate.sleep();
		this->mutex.lock();
		this->detectionReady = true;
		this->mutex.unlock();
	}
}


void Controller::VelocityPublisher()
{
	ros::Rate rate(VELOCITY_RATE);
	while(ros::ok()) {
		rate.sleep();
		this->PublishVelocity(this->desiredVelocity);
	}
}


void Controller::Step()
{
	Vector3d origin(CAM_WIDTH/2, CAM_HEIGHT/2);
	switch (this->state) {
		case LANDED:
			{
				this->PublishVelocity(Vector3d(0, 0, 0));
				std::cout << "[*] Press <ENTER> to start flying" << std::endl;
				if (std::cin.get()) {
					std::cout << "[*] Taking off!" << std::endl;
					this->startTakeOffTime = ros::Time::now();
					this->state = TAKEOFF;
				}
			}
		case TAKEOFF:
			{
				/* Take off */
				//if (this->altitude < 200) {
				ros::Duration d = ros::Time::now() - this->startTakeOffTime;
				if (d.toSec() < 4) {
					Velocity v;
					v.linear = Vector3d(0, 0, 0.5);
					this->desiredVelocity = v;
				} else {
					std::cout << "[*] Aiming" << std::endl;
					Velocity v;
					v.linear = Vector3d(0, 0, 0);
					v.yaw = 0.1;
					this->desiredVelocity = v;
					this->state = AIMING;
				}
				break;
			}
		case AIMING:
			{
				if (!this->gate_center.locked) {
					Velocity v;
					v.linear = Vector3d(0, 0, 0);
					v.yaw = 0.15;
					this->desiredVelocity = v;
				} else {
					this->ClearCalibration();
					this->state = CALIBRATING;
				}
				break;
			}
		case CALIBRATING:
			{
				this->Calibrate();
				break;
			}
		case REFINING:
			{
				if (!this->ready) {
					this->state = WAITING;
					break;
				}

				this->ready = false;
				//bool crossing = this->CrossingCondition();
				bool crossing = this->gate_center.distance <= CROSSING_DISTANCE;
				if (crossing) {
					std::cout << "[*] Crossing the gate, watch out !" <<
						std::endl;
					this->startCrossingTime = ros::Time::now();
					this->state = CROSSING;
				} else if (this->gate_center.locked) {
					/*std::cout << "[*] Flying towards coords [" <<
					  this->gate_center.x << "," << this->gate_center.y <<
					  "]" << std::endl;*/
					this->state = FLYING;
				} else if (this->gate_losses < LOSSES_BEFORE_REAIM) {
					this->gate_losses++;
				} else {
					this->gate_losses = 0;
					std::cout << "[*] Gate lost! Re-targetting..." << std::endl;
					this->state = AIMING;
				}
				break;
			}
		case FLYING:
			{
				assert(this->ref_gate.ratio != 0 && "0 ratio!");
				assert(this->ref_gate.width != 0 && "0 width!");
				assert(this->ref_gate.height != 0 && "0 height!");
				/* Compute the gate error */
				float ratio = GET_RATIO();

				Vector3d center(this->gate_center.x, this->gate_center.y, 0);
				if (std::abs(1.0 - (ratio / this->ref_gate.ratio)) >
						CALIBRATION_ERROR_THRESHOLD) {
					//std::cout << "[!] Compensating!" << std::endl;
					if (ratio > this->ref_gate.ratio) {
						/* Correct the height of the bbox */
						int correctedHeight = (this->gate_center.bbox.maxX -
								this->gate_center.bbox.minX) / this->ref_gate.ratio;
						center.y = (center.y > CAM_HEIGHT/2) ? 
							 GET_Y_COORD(correctedHeight) :
							 GET_Y_COORD(-correctedHeight);
					} else if (ratio < this->ref_gate.ratio) {
						/* Correct the width of the bbox */
						int correctedWidth = (this->gate_center.bbox.maxY -
								this->gate_center.bbox.minY) * this->ref_gate.ratio;
						center.x = (center.x > CAM_WIDTH/2) ? 
							GET_X_COORD(correctedWidth) :
							GET_X_COORD(-correctedWidth);
					}
				}
				Vector3d gate_err = origin - center;
				this->alignment_error = gate_err;

				/* Compute the velocity from the PID controller */
				auto velocity = this->PIDBoy->Compute(gate_err,
						this->current_velocity);

				/* TODO: Maybe it's better to compute the velocity until
				 * the error is 0, and then switch to the WAITING state ?
				 * */
				/* Apply the velocity or send it to the drone */
				this->desiredVelocity = velocity;
				//std::cout << "[*] Aligning" << std::endl;
				if(this->detectionReady) {
					this->mutex.lock();
					this->detectionReady = false;
					this->mutex.unlock();
					//std::cout << "[*] Correcting course..." << std::endl;
					this->state = REFINING;
				}
/*
				if (++(this->tick) >= DETECTION_RATE) {
					this->tick = 0;
					std::cout << "[*] Correcting course..." << std::endl;
					this->state = REFINING;
				}*/
				break;
			}
		case CROSSING:
			{
				//if (this->altitude > MAX_GATE_HEIGHT) {
				ros::Duration timeElapsed = ros::Time::now() - this->startCrossingTime;
				if (timeElapsed.toSec() < CROSSING_TIME) {
					Velocity v;
					v.linear = Vector3d(0.5, 0, 0);
					this->desiredVelocity = v;
				} else {
					std::cout << "[*] Leaving the gate" << std::endl;
					this->startLeavingTime = ros::Time::now();
					this->state = LEAVING;
				}
				break;
			}
		case LEAVING:
			{
				ros::Duration timeElapsed = ros::Time::now() - this->startLeavingTime;
				if (timeElapsed.toSec() < LEAVING_TIME) {
					Velocity v;
					v.linear = Vector3d(0.5, 0, 0);
					this->desiredVelocity = v;
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
				Velocity v;
				v.linear = Vector3d(0.15, 0, 0);
				this->desiredVelocity = v;
				if (this->ready)
					this->state = REFINING;
			}
	}
}


void Controller::SetPreviousPredictions(std::list<perception::Bbox> previous_predictions)
{
	this->previous_predictions = previous_predictions;
}


bool Controller::CrossingCondition()
{
	int prevArea, sameSize, smaller;
	bool crossing = false;

	if (this->previous_predictions.size() >= (PREVIOUS_PREDICTIONS_CNT)
			&& !this->gate_center.locked) {
		crossing = true;
		sameSize = smaller = 0;
		/* The first one is the current prediction, which we don't need */
		this->previous_predictions.pop_front();
		std::cout << "Area (has " << this->previous_predictions.size() << "): ";
		for (auto bboxIt = this->previous_predictions.begin(); bboxIt
				!= this->previous_predictions.end(); ++bboxIt) {
			int area = (bboxIt->maxX - bboxIt->minX)*(bboxIt->maxY - bboxIt->minY);
			std::cout  << area << ", ";
			if (area > prevArea && bboxIt != this->previous_predictions.begin()) {
				std::cout << " *bip* ";
				smaller++;
				//crossing = false;
				//break;
			} else if (area == prevArea && bboxIt !=
					this->previous_predictions.begin()) {
				sameSize++;
			}
			prevArea = area;
		}
		std::cout << std::endl;
		if (smaller > 2) {
			crossing = false;
		}
/*
		if (!crossing) {
			//this->previous_predictions.clear();
		} else if (sameSize >= PREVIOUS_PREDICTIONS_CNT/2) {
			crossing = false;
		}*/
	} else if (this->previous_predictions.size() < (PREVIOUS_PREDICTIONS_CNT)) {std::cout << " not enough predictions" << std::endl;}

	return crossing;
}


void Controller::Run()
{
	this->tick = 0;
	ros::Rate rate(this->rate);
	std::thread tDetection(&Controller::DetectionRate, this);
	std::thread tVelocity(&Controller::VelocityPublisher, this);

	while (ros::ok()) {
		rate.sleep();
		ros::spinOnce();
		this->Step();
	}

	tDetection.join();
	tVelocity.join();
}

int main(int argc, char **argv)
{
	std::cout << "[*] Running the controller..." << std::endl;
	ros::init(argc, argv, "Controller");

	// TODO: Read from config
	gain_param k_x, k_y;
	k_x.insert(std::pair<std::string, float>("p", 0.0));
	k_x.insert(std::pair<std::string, float>("i", 0.0));
	k_x.insert(std::pair<std::string, float>("d", 0.0));

	k_y.insert(std::pair<std::string, float>("p", 0.0));
	k_y.insert(std::pair<std::string, float>("i", 0.0));
	k_y.insert(std::pair<std::string, float>("d", 0.0));

	Controller controller(k_x, k_y, 0.25);
	controller.Run();

	ros::shutdown();


	return 0;
}
