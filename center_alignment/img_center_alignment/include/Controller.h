/*
 * Controller.h
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include "PID.h"


typedef enum State {
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
		int gate_region;
		int rate;
		void GatePredictionCallback(const GatePredictionMessagePtr &msg);
		void CurrentVelocityCallback(const Vector3Ptr &msg);
};

#endif /* !CONTROLLER_H */
