/*
 * Controller.h
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/UInt32.h>
#include <img_center_alignment/PIDConfig.h>
#include <math.h>
#include <queue>
#include <list>

#include "perception/GatePrediction.h"
#include "perception/Bbox.h"
#include "PID.h"


// TODO: Read from config
#define CONTROLLER_RATE 100 // 15 Hz
#define DETECTION_RATE 3 // 100Hz / 3
#define IMG_WIDTH 300
#define IMG_HEIGHT 225
#define CAM_WIDTH 640
#define CAM_HEIGHT 480
#define CROSSING_TIME 7
#define LEAVING_TIME 1
#define MAX_GATE_HEIGHT 100
#define PREVIOUS_PREDICTIONS_CNT 5
#define CALIBRATION_QUEUE_SIZE 5
#define CALIBRATION_ERROR_THRESHOLD 0.05 // Diff percentage


// Macros
#define GET_X_SHIFT() this->ref_gate.width-\
					(this->gate_center.bbox.maxX-this->gate_center.bbox.minX)
#define GET_Y_SHIFT() this->ref_gate.height-\
					(this->gate_center.bbox.maxY-this->gate_center.bbox.minY)
#define GET_RATIO() (this->gate_center.bbox.maxX\
							- this->gate_center.bbox.minX)\
						/ (this->gate_center.bbox.maxY\
								- this->gate_center.bbox.minY)

typedef enum {
	LANDED,
	TAKEOFF,
	AIMING,
	CALIBRATING,
	REFINING,
	FLYING,
	CROSSING,
	LEAVING,
	LANDING,
	WAITING
} State;

typedef struct Gate {
	int ratio;
	int height;
	int width;

	Gate(int ratio, int height, int width)
	{
		this->ratio = ratio;
		this->height = height;
		this->width = width;
	}
} Gate;

typedef struct Prediction {
	perception::Bbox bbox;
	bool locked;
	int x;
	int y;
} Prediction;

typedef int* Vector3Ptr;

using namespace img_center_alignment;

class Controller {
	public:
		Controller(gain_param k_x, gain_param k_y, float z_velocity);
		~Controller();
		void Run();
	private:
		PID *PIDBoy;
		State state;
		Velocity current_velocity;
		ros::NodeHandle handle;
		ros::Subscriber subHeightSensor;
		ros::Subscriber subPredictor;
		ros::Subscriber subVelocity;
		ros::Publisher pubVelocity;
		dynamic_reconfigure::Server<PIDConfig> dynRcfgServer;
		bool ready;
		float altitude;
		Prediction gate_center;
		int rate;
		Gate ref_gate;
		std::list<perception::Bbox> previous_predictions;
		std::queue<Gate> ref_gate_buffer;
		void HeightSensorCallback(const Vector3Ptr &msg);
		void GatePredictionCallback(const perception::GatePrediction::ConstPtr &msg);
		void CurrentVelocityCallback(geometry_msgs::TwistStampedConstPtr msg);
		void PublishVelocity(Velocity velocity);
		void PublishVelocity(Vector3d velocity);
		void PublishVelocity(float yawVelocity);
		void DynamicReconfigureCallback(PIDConfig &cfg, uint32_t level);
		void ClearCalibration();
		bool CrossingCondition();
};

#endif /* !CONTROLLER_H */
