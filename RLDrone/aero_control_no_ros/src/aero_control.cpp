/*
 * model_plugin.cc
 * Copyright (C) 2019 theomorales <theomorales@Theos-MacBook-Pro.local>
 *
 * Distributed under terms of the MIT license.
 */

#include "aero_control.hpp"


namespace gazebo
{
	AeroControl::AeroControl(): ModelPlugin()
	{
		std::cout << "[AERO] Created AeroControl plugin" << std::endl;
	}

	void AeroControl::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
		std::cout << "[AERO] Loading model " << std::endl;
		this->model = _parent;
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&AeroControl::OnUpdate, this));
		this->node = transport::NodePtr(new transport::Node());
		this->node->Init(this->model->GetWorld()->GetName());
		//transport::run();
		std::string topicName = "~/" + this->model->GetName() + "/velocity";
		std::cout << "[AERO] Subscribing to topic ";
		std::cout << topicName << std::endl;
		this->subVelocity = this->node->Subscribe("~/aero/velocity", &AeroControl::SetVelocity, this);
	}

	void AeroControl::OnUpdate()
	{
		// Example: apply a small linear velocity
		//this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
		//TODO: Publish to crashed
	}

	void AeroControl::SetVelocity(ConstVector3dPtr &msg)
	{
		std::cout << "[AERO] Got velocity command: " << std::endl;
		std::cout << msg->DebugString() << std::endl;
		this->model->SetLinearVel(msgs::ConvertIgn(*msg));
	}

	bool AeroControl::IsCrashed()
	{
		// TODO
		return false;
	}
}

