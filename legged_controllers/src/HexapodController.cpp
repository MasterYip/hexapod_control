/**
 * @file HexapodController.cpp
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

#include "legged_controllers/HexapodController.h"

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
  bool HexapodController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
  {
    // Initialize OCS2
    std::string urdfFile;
    std::string taskFile;
    std::string referenceFile;
    controller_nh.getParam("/urdfFile", urdfFile);
    controller_nh.getParam("/taskFile", taskFile);
    controller_nh.getParam("/referenceFile", referenceFile);
    bool verbose = false;
    loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

    setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
    // FIXME: MPC should not enabled
    setupMpc();
    setupMrt();

    // Visualization
    ros::NodeHandle nh;
    CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                        leggedInterface_->modelSettings().contactNames3DoF);
    robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                               leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
    selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                           leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

    // Hardware interface
    auto *hybridJointInterface = robot_hw->get<HybridJointInterface>();
    std::vector<std::string> joint_names{"RF_HAA", "RF_HFE", "RF_KFE",
                                         "RM_HAA", "RM_HFE", "RM_KFE",
                                         "RB_HAA", "RB_HFE", "RB_KFE",
                                         "LF_HAA", "LF_HFE", "LF_KFE",
                                         "LM_HAA", "LM_HFE", "LM_KFE",
                                         "LB_HAA", "LB_HFE", "LB_KFE"};
    for (const auto &joint_name : joint_names)
    {
      hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
    }
    auto *contactInterface = robot_hw->get<ContactSensorInterface>();
    for (const auto &name : leggedInterface_->modelSettings().contactNames3DoF)
    {
      contactHandles_.push_back(contactInterface->getHandle(name));
    }
    imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

    // State estimation
    setupStateEstimate(taskFile, verbose);

    // Whole body control
    wbc_ = std::make_shared<WeightedWbcSimple>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                               *eeKinematicsPtr_);
    wbc_->loadTasksSetting(taskFile, verbose);

    // Safety Checker
    safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

    ROS_WARN("HexapodController initialized.");
    ROS_WARN("State Num: %d", leggedInterface_->getCentroidalModelInfo().stateDim);
    ROS_WARN("Input Num: %d", leggedInterface_->getCentroidalModelInfo().inputDim);
    ROS_WARN("Actuated Dof Num: %d", leggedInterface_->getCentroidalModelInfo().actuatedDofNum);
    ROS_WARN("Generalized Dof Num: %d", leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum);
    ROS_WARN("3DOF Contact Num: %d", leggedInterface_->getCentroidalModelInfo().numThreeDofContacts);

    return true;
  }

  void HexapodController::starting(const ros::Time &time)
  {
    // Initial state
    currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
    updateStateEstimation(time, ros::Duration(0.002));
    currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
    currentObservation_.mode = ModeNumber::STANCE;

    TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

    // // Set the first observation and command and wait for optimization to finish
    // mpcMrtInterface_->setCurrentObservation(currentObservation_);
    // mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
    // ROS_INFO_STREAM("Waiting for the initial policy ...");
    // while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok())
    // {
    //   mpcMrtInterface_->advanceMpc();
    //   ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
    // }
    // ROS_INFO_STREAM("Initial policy has been received.");

    // mpcRunning_ = true;
    ROS_WARN("HexapodController started.");
  }

  void HexapodController::update(const ros::Time &time, const ros::Duration &period)
  {
    // State Estimate
    updateStateEstimation(time, period);
    // Update the current state of the system
    // mpcMrtInterface_->setCurrentObservation(currentObservation_);

    // Load the latest MPC policy
    // mpcMrtInterface_->updatePolicy();

    // Evaluate the current policy
    vector_t optimizedState, optimizedInput;
    optimizedState.resize(leggedInterface_->getCentroidalModelInfo().stateDim);
    optimizedInput.resize(leggedInterface_->getCentroidalModelInfo().inputDim);
    size_t plannedMode = 0; // The mode that is active at the time the policy is evaluated at.
    // mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

    // Whole body control (parameter definition can be found in the task file)
    currentObservation_.input = optimizedInput;

    // TODO: use heuristics to generate the desired state and input
    // wbcTimer_.startTimer();
    // vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
    // wbcTimer_.endTimer();

    // vector_t torque = x.tail(12);
    vector_t torque = vector_t::Zero(leggedInterface_->getCentroidalModelInfo().actuatedDofNum);

    vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

    // // Safety check, if failed, stop the controller
    // if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput))
    // {
    //   ROS_ERROR_STREAM("[Hexapod Controller] Safety check failed, stopping the controller.");
    //   stopRequest(time);
    // }

    for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j)
    {
      hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
    }

    // // Visualization
    // robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
    // selfCollisionVisualization_->update(currentObservation_);

    // // Publish the observation. Only needed for the command interface
    // observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
    ROS_WARN("HexapodController updated.");
  }

  void HexapodController::updateStateEstimation(const ros::Time &time, const ros::Duration &period)
  {
    vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
    contact_flag_t contacts;
    Eigen::Quaternion<scalar_t> quat;
    contact_flag_t contactFlag;
    vector3_t angularVel, linearAccel;
    matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

    for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
    {
      jointPos(i) = hybridJointHandles_[i].getPosition();
      jointVel(i) = hybridJointHandles_[i].getVelocity();
    }
    for (size_t i = 0; i < contacts.size(); ++i)
    {
      contactFlag[i] = contactHandles_[i].isContact();
    }
    for (size_t i = 0; i < 4; ++i)
    {
      quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
    }
    for (size_t i = 0; i < 3; ++i)
    {
      angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
      linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
    }
    for (size_t i = 0; i < 9; ++i)
    {
      orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
      angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
      linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
    }
    stateEstimate_->updateJointStates(jointPos, jointVel);
    stateEstimate_->updateContact(contactFlag);
    stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
    measuredRbdState_ = stateEstimate_->update(time, period);
    currentObservation_.time += period.toSec();
    scalar_t yawLast = currentObservation_.state(9);
    // FIXME: this caused program crashing
    std::cerr << "RBDState n:"<< measuredRbdState_.size() << std::endl;
    // measuredRbdState_: 24+24 q_gen + v_gen
    currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
    currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
    currentObservation_.mode = stateEstimate_->getMode();
  }

  HexapodController::~HexapodController()
  {
    controllerRunning_ = false;
    if (mpcThread_.joinable())
    {
      mpcThread_.join();
    }
    std::cerr << "########################################################################";
    std::cerr << "\n### MPC Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "########################################################################";
    std::cerr << "\n### WBC Benchmarking";
    std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
  }

  void HexapodController::setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                                               bool verbose)
  {
    leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
    leggedInterface_->modelSettings().jointNames = {"RF_HAA", "RF_HFE", "RF_KFE",
                                                    "RM_HAA", "RM_HFE", "RM_KFE",
                                                    "RB_HAA", "RB_HFE", "RB_KFE",
                                                    "LF_HAA", "LF_HFE", "LF_KFE",
                                                    "LM_HAA", "LM_HFE", "LM_KFE",
                                                    "LB_HAA", "LB_HFE", "LB_KFE"};
    leggedInterface_->modelSettings().contactNames3DoF = {"RF_FOOT", "RM_FOOT", "RB_FOOT",
                                                          "LF_FOOT", "LM_FOOT", "LB_FOOT"};
    leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
  }

  void HexapodController::setupMpc()
  {
    mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                    leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
    rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                      leggedInterface_->getCentroidalModelInfo());

    const std::string robotName = "legged_robot";
    ros::NodeHandle nh;
    // Gait receiver
    auto gaitReceiverPtr =
        std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
    // ROS ReferenceManager
    auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(nh);
    mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
    mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
  }

  void HexapodController::setupMrt()
  {
    mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
    mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
    mpcTimer_.reset();

    controllerRunning_ = true;
    mpcThread_ = std::thread([&]()
                             {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    } });
    setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
  }

  void HexapodController::setupStateEstimate(const std::string &taskFile, bool verbose)
  {
    stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
    dynamic_cast<KalmanFilterEstimate &>(*stateEstimate_).loadSettings(taskFile, verbose);
    currentObservation_.time = 0;
  }

  void HexapodCheaterController::setupStateEstimate(const std::string & /*taskFile*/, bool /*verbose*/)
  {
    stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                              leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  }

} // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::HexapodController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::HexapodCheaterController, controller_interface::ControllerBase)
