/*
 * PID.h
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef PID_H
#define PID_H

#include <Eigen>

typedef std::map<char, float> k_param;

class PID {
	public:
		PID(k_param gain_z, k_param gain_y, float x_velocity, int rate);
	private:
		std::map gain_z, gain_y;
		float x_velocity;
		int rate;
		Eigen::Vector3d err_integral;
		Eigen::Vector3d Compute(Eigen::Vector3d gate_err, Eigen::Vector3d current_velocity);
}

#endif /* !PID_H */
