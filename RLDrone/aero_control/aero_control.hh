/*
 * aero_control.hh
 * Copyright (C) 2019 theomorales <theomorales@Theos-MacBook-Pro.local>
 *
 * Distributed under terms of the MIT license.
 */

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
	class AeroControl: public ModelPlugin
	{
		public:
			AeroControl(): ModelPlugin();
			void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
			void OnUpdate();
			void SetVelocity(const ignition::math::Vector3d vel);
		private:
			physics::ModelPtr model;
			event::ConnectionPtr updateConnection;
	};
	GZ_REGISTER_MODEL_PLUGIN(AeroControl)
}

