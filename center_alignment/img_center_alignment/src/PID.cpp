/*
 * PID.cpp
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#include "PID.h"


PID::PID(k_param gain_z, k_param gain_y, float x_velocity, int rate)
{
	/* PID parameter maps("proportional", "integral", "derivative") for x, y
 	 * and z */
	this->gain_z = gain_z;
	this->gain_y = gain_y;
	this->x_velocity = x_velocity;
	this->rate = rate;
}

Eigen::Vector3d PID::Compute(Eigen::Vector3d err, Eigen::Vector3d current_velocity)
{
	this->err_integral += err * (1./this->rate);
	Eigen::Vector3d err_derivative = -current_velocity;
	Eigen::Vector3d velocity;
	velocity.z = this->gain_z.at("p") * err(0)
		+ this->gain_z.at("i") * this->err_integral(0)
		+ this->gain_z.at("d") * err_derivative(0);

	velocity.y = this->gain_y.at("p") * err(1)
		+ this->gain_y.at("i") * this->err_integral(1)
		+ this->gain_y.at("d") * err_derivative(1);

	velocity.x = this->x_velocity;

	return velocity;
}
