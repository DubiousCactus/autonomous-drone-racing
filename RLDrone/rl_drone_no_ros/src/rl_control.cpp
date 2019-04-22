/*
 * rl_control.cc
 * Copyright (C) 2019 theomorales <theomorales@Theos-MacBook-Pro.local>
 *
 * Distributed under terms of the MIT license.
 */

#include "rl_control.hpp"
#include <thread>


namespace gazebo
{
	void RLControl::ModelSpawned(ConstModelPtr &msg)
	{
		std::cout << "[*] Model spawned!" << std::endl;
		//std::cout << msg->DebugString();
		this->Train();
	}

	RLControl::RLControl()
	{
		std::cout << "[*] Starting RLControl" << std::endl;
		this->node = transport::NodePtr(new transport::Node());
		this->node->Init();
		this->pubVelocity = this->node->Advertise<msgs::Vector3d>("~/aero/velocity");
		this->pubFactory = this->node->Advertise<msgs::Factory>("~/factory");
		this->pubFactory->WaitForConnection();
		this->subSpawned = this->node->Subscribe("~/model/info", &RLControl::ModelSpawned, this);
	}

	RLControl::~RLControl() {
		/*delete this->node;
		delete this->pubVelocity;*/
	}

	void RLControl::SpawnDrone()
	{
		std::cout << "[*] Spawning aero_working_standalone" << std::endl;
		msgs::Factory msg;
		msg.set_sdf_filename("model://aero_working_standalone");
		// Pose to initialize the model to
		msgs::Set(msg.mutable_pose(),
				ignition::math::Pose3d(
					ignition::math::Vector3d(1, -2, 0),
					ignition::math::Quaterniond(0, 0, 0)));
		this->pubFactory->Publish(msg);
	}

	void RLControl::Train()
	{
		std::cout << "[*] Starting RL training..." << std::endl;
		std::cout << "[*] Waiting for connection on ~/aero/velocity" << std::endl;
		this->pubVelocity->WaitForConnection();
		std::cout << "[*] Connected!" << std::endl;
		this->isCrashed = false;
		msgs::Vector3d velMsg;
		msgs::Set(&velMsg, ignition::math::Vector3d(.9, .9, .9));
		this->pubVelocity->Publish(velMsg);
		this->pubVelocity->Publish(velMsg);
		this->pubVelocity->Publish(velMsg);
		this->pubVelocity->Publish(velMsg);
		this->pubVelocity->Publish(velMsg);
		this->pubVelocity->Publish(velMsg);
		this->pubVelocity->Publish(velMsg);
		/*while (true) {
			std::cout << "[*] Publishing velocity..." << std::endl;
			common::Time::Sleep(5);
		}*/
		std::cout << "[*] End of training session" << std::endl;
	}

/*
	void RLControl::SetCrashed(ConstBoolPtr &msg)
	{
		this->isCrashed = msg;
	}*/
}


int main(int argc, char **argv)
{
	/* GAZEBO MUST BE RUN BEFORE THE SUBSCRIPTION TIMES OUT! */
	gazebo::client::setup(argc, argv);
	gazebo::RLControl rlController;
	rlController.SpawnDrone();
	while(true)
		gazebo::common::Time::MSleep(100);
	gazebo::shutdown();

	return 0;
}
