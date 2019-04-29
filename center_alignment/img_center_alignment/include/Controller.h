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
#include <img_center_alignment/GatePredictionMessage.h>
#include <math.h>
#include <deque>

#include "PID.h"


// TODO: Read from config
#define DETECTION_RATE 100
#define IMG_WIDTH 340
#define IMG_HEIGHT 255
#define NB_WINDOWS 25
#define CROSSING_TIME 5
#define MAX_GATE_HEIGHT 100

typedef enum {
	LANDED,
	TAKEOFF,
	AIMING,
	REFINING,
	FLYING,
	CROSSING,
	LEAVING,
	LANDING
} State;

typedef int* Vector3Ptr;

using namespace img_center_alignment;

class Controller {
	public:
		Controller(gain_param k_x, gain_param k_y, float z_velocity,
				int filter_window_size);
		~Controller();
		void Run();
	private:
		PID *PIDBoy;
		State state;
		Vector3d current_velocity;
		ros::NodeHandle handle;
		ros::Subscriber subHeightSensor;
		ros::Subscriber subPredictor;
		ros::Subscriber subVelocity;
		ros::Publisher pubVelocity;
		float altitude;
		int gate_region;
		int rate;
		std::deque<int> filter_window;
		void HeightSensorCallback(const Vector3Ptr &msg);
		void GatePredictionCallback(const GatePredictionMessage &msg);
		void CurrentVelocityCallback(geometry_msgs::TwistStampedConstPtr msg);
		void PublishVelocity(Vector3d velocity);
		void PublishVelocity(float yawVelocity);
		int FilterPrediction(int prediction);
		Vector3d ComputeGateCenter();
};

#endif /* !CONTROLLER_H */
