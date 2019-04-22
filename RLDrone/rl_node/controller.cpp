#include "controller.hh"
#include <iostream>

namespace gazebo {
  Controller::Controller() {
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init("controller");
  }

  void Controller::Start()
  {
    std::cout << "[*] Subscribing to ~/rl_control/ready" << std::endl;
    this->subRlCtrl = this->node->Subscribe("~/rl_control/ready", &Controller::PluginReadyCallback, this);
  }

  void Controller::PluginReadyCallback()
  {
    std::cout << _msg->DebugString();
    d::cout << "[*] Publishing topic ~/aero/velocity" << std::endl;
    this->pubVelocity = this->node->Advertise<msgs::Vector3d>("~/aero/velocity");
    this->Train();
  }

  void Controller::PluginSpawnCallback()
  {
    //std::cout << _msg->DebugString();
  }

  void Controller::Train()
  {
    std::cout << "[*] Starting RL training..." << std::endl;
    std::cout << "[*] Waiting for connection on ~/aero/velocity" << std::endl;
    this->pubVelocity->WaitForConnection();
    std::cout << "[*] Got connection!" << std::endl;
    msgs::Vector3d velMsg;
    msgs::Set(&velMsg, ignition::math::Vector3d(.9, 0, .5));
    while (true) {
      std::cout << "[*] Publishing velocity" << std::endl;
      this->pubVelocity->Publish(velMsg);
      common::Time::Sleep(5);
    }
    std::cout << "[*] End of training session" << std::endl;
  }
}


/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);
  gazebo::transport::init();
  gazebo::transport::run();

  std::cout << "[*] Starting the controller..." << std::endl;
  gazebo::Controller controller;
  controller.Start();

  // Busy wait loop...replace with your own code as needed.
  /*while (true)
    gazebo::common::Time::MSleep(10);*/

  gazebo::transport::fini();
  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
