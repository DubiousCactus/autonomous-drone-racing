/*
 * Controller.h
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen3/Eigen/Dense>

#include "PID.h"


typedef enum {
	LANDED,
	AIMING,
	FLYING,
	CROSSING
} State;

typedef int* GatePredictionMessagePtr;
typedef int* Vector3Ptr;

class Controller {
	public:
		Controller(gain_param k_x, gain_param k_y, float z_velocity);
		~Controller();
		void Run();
	private:
		PID *PIDBoy;
		State state;
		Eigen::Vector3d current_velocity;
		ros::NodeHandle handle;
		ros::Subscriber subHeightSensor;
		ros::Subscriber subPredictor;
		ros::Subscriber subVelocity;
		float altitude;
		int gate_region;
		int rate;
		void HeightSensorCallback(const Vector3Ptr &msg);
		void GatePredictionCallback(const GatePredictionMessagePtr &msg);
		void CurrentVelocityCallback(geometry_msgs::TwistStampedConstPtr msg);
};

#endif /* !CONTROLLER_H */
