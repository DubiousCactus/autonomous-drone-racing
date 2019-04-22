#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

namespace gazebo {
	class Controller
	{
		public:
			Controller();
			void Start();
		private:
			transport::NodePtr node;
			transport::SubscriberPtr subRlCtrl;
			transport::PublisherPtr pubVelocity;
			void PluginSpawnCallback(/*TODO*/);
			void PluginReadyCallback(/*TODO*/);
			void Train();
	};
}
