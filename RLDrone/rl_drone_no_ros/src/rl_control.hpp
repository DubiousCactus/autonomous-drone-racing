#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
	class RLControl
	{
		public:
			RLControl();
			~RLControl();
			void SpawnDrone();
		private:
			transport::NodePtr node;
			transport::PublisherPtr pubVelocity;
			transport::PublisherPtr pubFactory;
			transport::SubscriberPtr subCrashed;
			transport::SubscriberPtr subSpawned;
			bool isCrashed;
			void ModelSpawned(ConstModelPtr &msg);
			void Train();
			//void SetCrashed(ConstBoolPtr &msg);
	};
}
