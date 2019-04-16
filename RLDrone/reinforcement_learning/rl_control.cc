/*
 * rl_control.cc
 * Copyright (C) 2019 theomorales <theomorales@Theos-MacBook-Pro.local>
 *
 * Distributed under terms of the MIT license.
 */

#include "rl_control.hh"


namespace gazebo
{
	RLControl::RLControl(): WorldPlugin()
	{
		std::cout << "[*] Created RLControl plugin" << std::endl;
	}

	RLControl::~RLControl() {}

	void RLControl::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
	{
		std::cout << "[*] Loading RLControl plugin..." << std::endl;
		this->world = _parent;
		this->node = transport::NodePtr(new transport::Node());
		this->node->Init(this->world->Name());
		this->pubVelocity = this->node->Advertise<msgs::Vector3d>("~/intel_aero/velocity");
		this->SpawnDrone(ignition::math::Vector3d(0, 0, 0));
		std::string topicName = "~/intel_aero/crashed";
		std::cout  << "[*] Subscribing to topic " << topicName << std::endl;
		//this->subCrashed = this->node->Subscribe(topicName, &RLControl::SetCrashed, this);
		this->Train();
	}

	void RLControl::Train()
	{
		std::cout << "[*] Starting RL training..." << std::endl;
		std::cout << "[*] Waiting for connection on ~/intel_aero/velocity" << std::endl;
		//this->pubVelocity->WaitForConnection();
		this->isCrashed = false;
		msgs::Vector3d velMsg;
		msgs::Set(&velMsg, ignition::math::Vector3d(.9, 0, .5));
		this->pubVelocity->Publish(velMsg);
		std::cout << "[*] End of training session" << std::endl;
	}

	void RLControl::SpawnDrone(const ignition::math::Vector3d location)
	{
		std::cout << "[*] Spawning Intel Aero at " << location << std::endl;
		// The filename must be in the GAZEBO_MODEL_PATH environment variable.
		this->world->InsertModelFile("model://Intel Aero");
	}
/*
	void RLControl::SetCrashed(ConstBoolPtr &msg)
	{
		this->isCrashed = msg;
	}*/
}
