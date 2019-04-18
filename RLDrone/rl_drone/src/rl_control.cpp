/*
 * rl_control.cc
 * Copyright (C) 2019 theomorales <theomorales@Theos-MacBook-Pro.local>
 *
 * Distributed under terms of the MIT license.
 */

#include "rl_control.hpp"


namespace gazebo
{
	RLControl::RLControl(): WorldPlugin()
	{
		ROS_INFO("[*] Created RLControl plugin");
	}

	RLControl::~RLControl() {}

	void RLControl::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
	{
		// Make sure the ROS node for Gazebo has already been initialized                                                                                    
		if (!ros::isInitialized())
		{
			ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
					<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			return;
		}

		ROS_INFO("[*] Loading RLControl plugin...");
		this->world = _parent;
		this->node = transport::NodePtr(new transport::Node());
		this->node->Init(this->world->GetName());
		this->pubVelocity = this->node->Advertise<msgs::Vector3d>("~/intel_aero/velocity");
		this->SpawnDrone(ignition::math::Vector3d(0, 0, 0));
		//std::string topicName = "~/intel_aero/crashed";
		//ROS_INFO("[*] Subscribing to topic ");
		//this->subCrashed = this->node->Subscribe(topicName, &RLControl::SetCrashed, this);
	}

	void RLControl::Train()
	{
		ROS_INFO("[*] Starting RL training...");
		//ROS_INFO("[*] Waiting for connection on ~/intel_aero/velocity");
		//this->pubVelocity->WaitForConnection();
		this->isCrashed = false;
		msgs::Vector3d velMsg;
		msgs::Set(&velMsg, ignition::math::Vector3d(.9, 0, .5));
		/*while (true) {*/
			//this->pubVelocity->Publish(velMsg);
			//common::Time::MSleep(500);
		/*}*/
		ROS_INFO("[*] End of training session");
	}

	void RLControl::SpawnDrone(const ignition::math::Vector3d location)
	{
		ROS_INFO("[*] Spawning Intel Aero");
		this->subSpawned = this->node->Subscribe("~/model/info", &RLControl::ModelSpawned, this);
		this->world->SetPaused(true);
		//ROS_INFO("[*] Spawning Intel Aero at " + location);
		// The filename must be in the GAZEBO_MODEL_PATH environment variable.
		//this->world->InsertModelFile("model://aero_control/models/aero");
		this->world->InsertModelFile("model://aero_control/models/aero_working_standalone");
		//this->world->InsertModelFile("model://Intel Aero");
	}

	void RLControl::ModelSpawned(ConstModelPtr &msg)
	{
		ROS_INFO("[*] Model spawned. Unpausing simulation.");
		this->world->SetPaused(false);
		common::Time::Sleep(5);
		this->Train();
	}
/*
	void RLControl::SetCrashed(ConstBoolPtr &msg)
	{
		this->isCrashed = msg;
	}*/
}
