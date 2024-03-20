
//
// Created by qiayuan on 1/24/22.
//

#include "legged_hexapod_hw/HexapodHW.h"
#include <Eigen/Dense>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>

namespace legged
{
  bool HexapodHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
  {
    if (!LeggedHW::init(root_nh, robot_hw_nh))
    {
      return false;
    }

    setupJoints();
    setupImu();
    setupContactSensor(robot_hw_nh);

    std::string robot_type;
    root_nh.getParam("robot_type", robot_type);

    if (robot_type != "elspider_air")
    {
      ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
      return false;
    }

    jointStateSub_ = root_nh.subscribe("/hexapod/joint_state_fdb", 1, &HexapodHW::jointStateCallback, this);
    imuSub_ = root_nh.subscribe("/trunk_imu", 1, &HexapodHW::imuCallback, this);
    return true;
  }

  void HexapodHW::jointStateCallback(const sensor_msgs::JointState &msg)
  {
    jointStateMsg_ = msg;
  }

  void HexapodHW::imuCallback(const sensor_msgs::Imu &msg)
  {
    imuMsg_ = msg;
  }

  void HexapodHW::read(const ros::Time &time, const ros::Duration & /*period*/)
  {
    for (size_t i = 0; i < 18; ++i)
    {
      jointData_[i].pos_ = jointStateMsg_.position[i];
      jointData_[i].vel_ = jointStateMsg_.velocity[i];
      jointData_[i].tau_ = jointStateMsg_.effort[i];
    }

    imuData_.ori_[0] = imuMsg_.orientation.x;
    imuData_.ori_[1] = imuMsg_.orientation.y;
    imuData_.ori_[2] = imuMsg_.orientation.z;
    imuData_.ori_[3] = imuMsg_.orientation.w;
    imuData_.angularVel_[0] = imuMsg_.angular_velocity.x;
    imuData_.angularVel_[1] = imuMsg_.angular_velocity.y;
    imuData_.angularVel_[2] = imuMsg_.angular_velocity.z;
    imuData_.linearAcc_[0] = imuMsg_.linear_acceleration.x;
    imuData_.linearAcc_[1] = imuMsg_.linear_acceleration.y;
    imuData_.linearAcc_[2] = imuMsg_.linear_acceleration.z;

    // TODO: Update Contact State
    Eigen::Vector3d legtorque;
    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i)
    {
      legtorque << jointData_[i * 3].tau_, jointData_[i * 3 + 1].tau_, jointData_[i * 3 + 2].tau_;
      contactState_[i] = legtorque.norm() > contactThreshold_;
    }

    // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
    std::vector<std::string> names = hybridJointInterface_.getNames();
    for (const auto &name : names)
    {
      HybridJointHandle handle = hybridJointInterface_.getHandle(name);
      handle.setFeedforward(0.);
      handle.setVelocityDesired(0.);
      handle.setKd(3.);
    }
  }

  void HexapodHW::write(const ros::Time & /*time*/, const ros::Duration & /*period*/)
  {
    for (int i = 0; i < 18; ++i)
    {
      jointCmdMsg_.position[i] = jointData_[i].posDes_;
      jointCmdMsg_.velocity[i] = jointData_[i].velDes_;
      jointCmdMsg_.torque[i] = jointData_[i].ff_;
      jointCmdMsg_.kp[i] = jointData_[i].kp_;
      jointCmdMsg_.kd[i] = jointData_[i].kd_;
    }
    jointCmdMsg_.header.stamp = ros::Time::now();
    jointCmdPub_.publish(jointCmdMsg_);
  }

  bool HexapodHW::setupJoints()
  {
    for (int index = 0; index < JOINT_NAME.size(); index++)
    {
      hardware_interface::JointStateHandle state_handle(JOINT_NAME.at(index), &jointData_[index].pos_, &jointData_[index].vel_,
                                                        &jointData_[index].tau_);
      jointStateInterface_.registerHandle(state_handle);
      hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                             &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
    }
    return true;
  }

  bool HexapodHW::setupImu()
  {
    imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("base_imu", "base_imu", imuData_.ori_, imuData_.oriCov_,
                                                                           imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                           imuData_.linearAccCov_));
    imuData_.oriCov_[0] = 0.0012;
    imuData_.oriCov_[4] = 0.0012;
    imuData_.oriCov_[8] = 0.0012;

    imuData_.angularVelCov_[0] = 0.0004;
    imuData_.angularVelCov_[4] = 0.0004;
    imuData_.angularVelCov_[8] = 0.0004;

    return true;
  }

  bool HexapodHW::setupContactSensor(ros::NodeHandle &nh)
  {
    nh.getParam("contact_threshold", contactThreshold_);
    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i)
    {
      contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
    }
    return true;
  }

} // namespace legged
