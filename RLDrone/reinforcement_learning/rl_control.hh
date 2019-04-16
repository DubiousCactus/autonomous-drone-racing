#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
	class RLControl: public WorldPlugin
	{
		public:
			RLControl();
			~RLControl();
			void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
		private:
			physics::WorldPtr world;
			transport::NodePtr node;
			transport::PublisherPtr pubVelocity;
			transport::SubscriberPtr subCrashed;
			bool isCrashed;
			void SpawnDrone(const ignition::math::Vector3d location);
			//void SetCrashed(ConstBoolPtr &msg);
			void Train();
	};

	GZ_REGISTER_WORLD_PLUGIN(RLControl)
}
