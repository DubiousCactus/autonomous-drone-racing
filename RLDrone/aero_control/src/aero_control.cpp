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
		ROS_INFO("[AERO] Created AeroControl plugin");
	}

	void AeroControl::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
		// Make sure the ROS node for Gazebo has already been initialized
		if (!ros::isInitialized())
		{
			ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
					<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			return;
		}

		ROS_INFO("[AERO] Loading model ");
		this->model = _parent;
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&AeroControl::OnUpdate, this));
		this->node = transport::NodePtr(new transport::Node());
		this->node->Init(this->model->GetWorld()->GetName());
		transport::run();
		std::string topicName = "~/" + this->model->GetName() + "/velocity";
		ROS_INFO("[AERO] Subscribing to topic ");
		std::cout << topicName << std::endl;
		this->subVelocity = this->node->Subscribe(topicName, &AeroControl::SetVelocity, this);
	}

	void AeroControl::OnUpdate()
	{
		// Example: apply a small linear velocity
		//this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
		//TODO: Publish to crashed
	}

	void AeroControl::SetVelocity(ConstVector3dPtr &msg)
	{
		ROS_INFO("[AERO] Got velocity command: ");
		std::cout << msg->DebugString() << std::endl;
		this->model->SetLinearVel(msgs::ConvertIgn(*msg));
	}

	bool AeroControl::IsCrashed()
	{
		// TODO
		return false;
	}
}

