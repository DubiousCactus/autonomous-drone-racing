/*
 * model_plugin.cc
 * Copyright (C) 2019 theomorales <theomorales@Theos-MacBook-Pro.local>
 *
 * Distributed under terms of the MIT license.
 */

#include "aero_control.hh"


namespace gazebo
{
	AeroControl::AeroControl(): ModelPlugin()
	{
		std::cout << "Hello world!" << std::endl;
	}

	void AeroControl::Load(physics::ModelPtr _parent, sdf::ElementPtr)
	{
		std::cout << "Loading model " << _parent->GetName() << std::endl;
		this->model = _parent;
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&AeroControl::OnUpdate, this));
	}

	void AeroControl::OnUpdate()
	{
		// Example: apply a small linear velocity
		this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
	}

	void AeroControl::SetVelocity(const ignition::math::Vector3d vel)
	{
		this->model->SetLinearVel(vel);
	}

	bool AeroControl::IsCrashed()
	{
		// TODO
		return false;
	}
}

