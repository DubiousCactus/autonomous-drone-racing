/*
 * PID.h
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef PID_H
#define PID_H

#include <eigen3/Eigen/Dense>
#include <string>
#include <map>

typedef std::map<std::string, float> gain_param;

class PID {
	public:
		PID(gain_param gain_z, gain_param gain_y, float x_velocity, int rate);
		Eigen::Vector3d Compute(Eigen::Vector3d gate_err, Eigen::Vector3d current_velocity);
	private:
		gain_param gain_z, gain_y;
		float x_velocity;
		int rate;
		Eigen::Vector3d err_integral;
};

#endif /* !PID_H */
