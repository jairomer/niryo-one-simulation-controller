#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H
#include "simulation.hpp"
#include "joint_state_controller/joint_state_controller.h"

using namespace joint_state_controller;

bool JointStateController::init(hardware_interface::JointStateInterface* hw,
                                  ros::NodeHandle&                         root_nh,
                                  ros::NodeHandle&                         controller_nh)
  {
    return false;
  }

  void JointStateController::starting(const ros::Time& time)
  {
    // initialize time
    last_publish_time_ = time;
  }

  void JointStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {}

  void JointStateController::stopping(const ros::Time& /*time*/)
  {}

  void JointStateController::addExtraJoints(const ros::NodeHandle& nh, sensor_msgs::JointState& msg)
  {}


#endif