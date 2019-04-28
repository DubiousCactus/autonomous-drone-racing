/*
 * PID.cpp
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#include "PID.h"


PID::PID(gain_param gain_z, gain_param gain_y, double x_velocity, int rate)
{
	/* PID parameter maps("proportional", "integral", "derivative") for x, y
 	 * and z */
	this->gain_z = gain_z;
	this->gain_y = gain_y;
	this->x_velocity = x_velocity;
	this->rate = rate;
}

Vector3d PID::Compute(Vector3d err, Vector3d current_velocity)
{
	this->err_integral += err * (1./this->rate);
	Vector3d err_derivative = -current_velocity;
	double z = this->gain_z.at("p") * err.y
		+ this->gain_z.at("i") * this->err_integral.y
		+ this->gain_z.at("d") * err_derivative.y;

	double y = this->gain_y.at("p") * err.x
		+ this->gain_y.at("i") * this->err_integral.x
		+ this->gain_y.at("d") * err_derivative.x;

	Vector3d velocity(this->x_velocity, y, z);

	return velocity;
}
