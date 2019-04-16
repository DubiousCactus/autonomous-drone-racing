/*
 * aero_control.hh
 * Copyright (C) 2019 theomorales <theomorales@Theos-MacBook-Pro.local>
 *
 * Distributed under terms of the MIT license.
 */

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
	class AeroControl: public ModelPlugin
	{
		public:
			AeroControl();
			void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
			void OnUpdate();
		private:
			physics::ModelPtr model;
			event::ConnectionPtr updateConnection;
			transport::NodePtr node;
			transport::SubscriberPtr subVelocity;
			transport::PublisherPtr pubCrashed;
			void SetVelocity(ConstVector3dPtr &msg);
			bool IsCrashed();
	};
	GZ_REGISTER_MODEL_PLUGIN(AeroControl)
}

