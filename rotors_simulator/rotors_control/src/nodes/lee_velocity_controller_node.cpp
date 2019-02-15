#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "lee_velocity_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control
{

LeeVelocityControllerNode::LeeVelocityControllerNode()
{
  InitializeParams();

  ros::NodeHandle nh;

  cmd_pose_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_POSE, 1, &LeeVelocityControllerNode::CommandPoseCallback, this);

  //cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1, &LeeVelocityControllerNode::MultiDofJointTrajectoryCallback, this);

  velocity_sub_ = nh.subscribe("command_velocity", 1, &LeeVelocityControllerNode::VelocityCallback, this);

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &LeeVelocityControllerNode::OdometryCallback, this);

  //
  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  command_timer_ = nh.createTimer(ros::Duration(0), &LeeVelocityControllerNode::TimedCommandCallback, this, true, false);
}

LeeVelocityControllerNode::~LeeVelocityControllerNode() { }

// 12 parameters for the state
void LeeVelocityControllerNode::InitializeParams()
{
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.

  // For the position (3)
  lee_position_controller_.controller_parameters_.position_gain_.x() = 0;
  lee_position_controller_.controller_parameters_.position_gain_.y() = 0;
  lee_position_controller_.controller_parameters_.position_gain_.z() = 0;

  // FOr the velocity (3)
  GetRosParameter(pnh, "velocity_gain/x",
                  lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(pnh, "velocity_gain/y",
                  lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(pnh, "velocity_gain/z",
                  lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.z());

  // For the attitude (3)
  GetRosParameter(pnh, "attitude_gain/x",
                  lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.z());

  // For the angular rate (3)
  GetRosParameter(pnh, "angular_rate_gain/x",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(pnh, &lee_position_controller_.vehicle_parameters_);

  lee_position_controller_.InitializeParameters();
}

void LeeVelocityControllerNode::Publish()
{
  // Nothing to define
}

void LeeVelocityControllerNode::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void LeeVelocityControllerNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1)
  {
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i)
  {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1)
  {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void LeeVelocityControllerNode::VelocityCallback(const geometry_msgs::QuaternionConstPtr& msg){
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  eigen_reference.position_W = Eigen::Vector3d::Zero();
  eigen_reference.velocity_W << msg->x, msg->y, msg->z;
  eigen_reference.acceleration_W = Eigen::Vector3d::Zero();
  eigen_reference.jerk_W = Eigen::Vector3d::Zero();
  eigen_reference.snap_W = Eigen::Vector3d::Zero();
  eigen_reference.orientation_W_B = Eigen::Quaterniond::Identity();
  eigen_reference.angular_velocity_W << Eigen::Vector3d::Zero();

  commands_.push_front(eigen_reference);

  // We can trigger the first command immediately.
  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void LeeVelocityControllerNode::TimedCommandCallback(const ros::TimerEvent& e)
{
  if(commands_.empty())
  {
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty())
  {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void LeeVelocityControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("LeePositionController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  lee_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++){
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
    //std::cout << "[lee_position] " << ref_rotor_velocities[i] << std::endl;
  }
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);

}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lee_velocity_controller_node");

  rotors_control::LeeVelocityControllerNode lee_velocity_controller_node;

  ros::spin();

  return 0;
}
