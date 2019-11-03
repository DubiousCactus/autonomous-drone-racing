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
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/UInt32.h>
#include <img_center_alignment/PIDConfig.h>
#include <cmath>
#include <thread>
#include <list>
#include <mutex>

#include "perception/GatePrediction.h"
#include "perception/Bbox.h"
#include "img_center_alignment/PID.h"


#define CONTROLLER_RATE 100
#define VELOCITY_RATE 100
#define DETECTION_RATE 20
#define IMG_WIDTH 300
#define IMG_HEIGHT 225
#define CAM_WIDTH 640
#define CAM_HEIGHT 480
#define LEAVING_TIME 1.0
#define PREVIOUS_PREDICTIONS_CNT 5
#define LOSSES_BEFORE_REAIM 8
#define CALIBRATION_QUEUE_SIZE 30
#define CALIBRATION_ERROR_THRESHOLD 0.10 // Accepted error percentage
#define CROSSING_DISTANCE 3.0
#define SPIKE_THRESHOLD 0.05
#define PID_TUNNING 0


// Macros
#define GET_RATIO() (float)(this->gate_center.bbox.maxX\
						- this->gate_center.bbox.minX)\
					/(float)(this->gate_center.bbox.maxY\
							- this->gate_center.bbox.minY)
#define GET_Y_COORD(offset) (this->gate_center.bbox.minY + offset +\
							this->gate_center.bbox.maxY)/2
#define GET_X_COORD(offset) (this->gate_center.bbox.minX + offset +\
							 this->gate_center.bbox.maxX)/2

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
	float ratio;
	int width;
	int height;
} Gate;

typedef struct Prediction {
	perception::Bbox bbox;
	bool locked;
	float distance;
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
		void Step();
		void SetState(State state);
		void SetGateCenter(Prediction center);
		bool CrossingCondition();
		void SetPreviousPredictions(std::list<perception::Bbox> previous_predictions);
		State GetState();
		Gate GetRefGate();
		Vector3d GetAlignmentError();
		std::list<Gate> GetRefGateBuffer();

	private:
		PID *PIDBoy;
		State state;
		Velocity current_velocity;
		Vector3d alignment_error;
		ros::NodeHandle handle;
		ros::Subscriber subHeightSensor;
		ros::Subscriber subPredictor;
		ros::Subscriber subVelocity;
		ros::Publisher pubVelocity;
		ros::Time startLeavingTime;
		ros::Time startTakeOffTime;
		ros::Time startCrossingTime;
		dynamic_reconfigure::Server<PIDConfig> dynRcfgServer;
		bool ready;
		bool detectionReady;
		bool altitude_spike;
		std::mutex mutex;
		float altitude;
		float spikeThreshold;
		Prediction gate_center;
		int rate;
		int tick;
		int gate_losses;
		Gate ref_gate;
		Velocity desiredVelocity;
		std::list<perception::Bbox> previous_predictions;
		std::list<Gate> ref_gate_buffer;
		void HeightSensorCallback(const sensor_msgs::RangeConstPtr msg);
		void GatePredictionCallback(const perception::GatePrediction::ConstPtr msg);
		void CurrentVelocityCallback(geometry_msgs::TwistStampedConstPtr msg);
		void PublishVelocity(Velocity velocity);
		void PublishVelocity(Vector3d velocity);
		void PublishVelocity(float yawVelocity);
		void DynamicReconfigureCallback(PIDConfig &cfg, uint32_t level);
		void ClearCalibration();
		void Calibrate();
		void DetectionRate();
		void VelocityPublisher();
};

#endif /* !CONTROLLER_H */
