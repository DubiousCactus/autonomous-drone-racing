/*
 * PID.cpp
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#include "PID.h"


PID::PID(gain_param gain_z, gain_param gain_yaw, double x_velocity, int rate)
{
	/* PID parameter maps("proportional", "integral", "derivative") for x, y
 	 * and z */
	this->SetGainParameters(gain_z, gain_yaw, x_velocity);
	this->rate = rate;
}

void PID::SetGainParameters(gain_param z, gain_param y, double x_velocity)
{
	this->gain_z = z;
	this->gain_yaw = y;
	this->x_velocity = x_velocity;
}

Velocity PID::Compute(Vector3d err, Velocity current_velocity)
{
	this->err_integral += err * (1./this->rate);
	Vector3d err_derivative = -current_velocity.linear;
	double z = this->gain_z.at("p") * err.y
		+ this->gain_z.at("i") * this->err_integral.y
		+ this->gain_z.at("d") * err_derivative.z;

	double yaw = this->gain_yaw.at("p") * err.x
		+ this->gain_yaw.at("i") * this->err_integral.x
		+ this->gain_yaw.at("d") * -current_velocity.yaw;

	Vector3d linVel(this->x_velocity, 0, z);
	Velocity vel;
	vel.linear = linVel;
	vel.yaw = yaw;

	return vel;
}
