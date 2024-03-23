/**
 * @file TestController.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */

//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include "legged_controllers/TestController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbcSimple.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged
{
  bool TestController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
  {
    ROS_FATAL("TestController::init");
    return true;
  }

  void TestController::starting(const ros::Time &time)
  {
    ROS_FATAL("TestController::starting");
  }

  void TestController::update(const ros::Time &time, const ros::Duration &period)
  {
    ROS_FATAL("TestController::update");
  }

  TestController::~TestController()
  {
    ROS_FATAL("TestController::~TestController");
  }

} // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::TestController, controller_interface::ControllerBase)
