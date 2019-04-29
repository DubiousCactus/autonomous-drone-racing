/*
 * PID.h
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef PID_H
#define PID_H

#include <string>
#include <map>

typedef std::map<std::string, double> gain_param;

typedef struct Vector3d {
	double x, y, z;
	Vector3d(double x, double y, double z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vector3d(double x, double y)
	{
		this->x = x;
		this->y = y;
		this->z = 0;
	}

	Vector3d()
	{
		x = y = z = 0;
	}

	Vector3d& operator+=(const Vector3d& base)
	{
		this->x += base.x;
		this->y += base.y;
		this->z += base.z;

		return *this;
	}

	Vector3d operator*(const double value)
	{
		x *= value;
		y *= value;
		z *= value;
	}

	Vector3d operator-()
	{
		Vector3d tmp(*this);
		tmp.x = -tmp.x;
		tmp.y = -tmp.y;
		tmp.z = -tmp.z;

		return tmp;
	}

	Vector3d operator-(Vector3d base)
	{
		Vector3d tmp(*this);
		tmp.x -= base.x;
		tmp.y -= base.y;
		tmp.z -= base.z;

		return tmp;
	}
} Vector3d;


class PID {
	public:
		PID(gain_param gain_z, gain_param gain_y, double x_velocity, int rate);
		Vector3d Compute(Vector3d gate_err, Vector3d current_velocity);
		void SetGainParameters(gain_param z, gain_param y, double x_velocity);
	private:
		gain_param gain_z, gain_y;
		double x_velocity;
		int rate;
		Vector3d err_integral;
};

#endif /* !PID_H */
