/**
 * @file HexapodHW.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */
//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include <legged_hw/LeggedHW.h>
#include <legged_hexapod_hw/JointCmd.h>
#include <legged_hexapod_hw/FootState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
namespace legged
{

  // Define JOINT_STATE_NAME and FOOT_LINK_NAME constants
  const std::vector<std::string> JOINT_NAME = {"aRF_HAA", "aRF_HFE", "aRF_KFE",
                                               "bRM_HAA", "bRM_HFE", "bRM_KFE",
                                               "cRB_HAA", "cRB_HFE", "cRB_KFE",
                                               "dLF_HAA", "dLF_HFE", "dLF_KFE",
                                               "eLM_HAA", "eLM_HFE", "eLM_KFE",
                                               "fLB_HAA", "fLB_HFE", "fLB_KFE"};

  const std::vector<std::string> CONTACT_SENSOR_NAMES = {"aRF_FOOT", "bRM_FOOT", "cRB_FOOT",
                                                         "dLF_FOOT", "eLM_FOOT", "fLB_FOOT"};

  struct UnitreeMotorData
  {
    double pos_, vel_, tau_;                // state
    double posDes_, velDes_, kp_, kd_, ff_; // command
  };

  struct UnitreeImuData
  {
    double ori_[4];           // NOLINT(modernize-avoid-c-arrays)
    double oriCov_[9];        // NOLINT(modernize-avoid-c-arrays)
    double angularVel_[3];    // NOLINT(modernize-avoid-c-arrays)
    double angularVelCov_[9]; // NOLINT(modernize-avoid-c-arrays)
    double linearAcc_[3];     // NOLINT(modernize-avoid-c-arrays)
    double linearAccCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  };

  class HexapodHW : public LeggedHW
  {
  public:
    HexapodHW() = default;
    /** \brief Get necessary params from param server. Init hardware_interface.
     *
     * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
     * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
     *
     * @param root_nh Root node-handle of a ROS node.
     * @param robot_hw_nh Node-handle for robot hardware.
     * @return True when init successful, False when failed.
     */
    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

    /** \brief Communicate with hardware. Get data, status of robot.
     * @param time Current time
     * @param period Current time - last time
     */
    void read(const ros::Time &time, const ros::Duration &period) override;

    /** \brief Comunicate with hardware. Publish command to robot.
     * Propagate joint state to actuator state for the stored
     * transmission.
     * @param time Current time
     * @param period Current time - last time
     */
    void write(const ros::Time &time, const ros::Duration &period) override;

    void jointStateCallback(const sensor_msgs::JointState &msg);

    void imuCallback(const sensor_msgs::Imu &msg);

    void footStateCallback(const legged_hexapod_hw::FootState &msg);

  private:
    bool setupJoints();

    bool setupImu();

    bool setupContactSensor(ros::NodeHandle &nh);

    // Debug
    void printIMU();
    void printJointState();

    UnitreeMotorData jointData_[18]{}; // NOLINT(modernize-avoid-c-arrays)
    UnitreeImuData imuData_{};
    bool contactState_[6]{}; // NOLINT(modernize-avoid-c-arrays)

    int contactThreshold_{};

    ros::Publisher contactPublisher_;
    ros::Time lastContactPub_;

    // Feedback
    ros::Subscriber jointStateSub_;
    sensor_msgs::JointState jointStateMsg_;
    bool jointStateReceived_{false};

    ros::Subscriber imuSub_;
    sensor_msgs::Imu imuMsg_;
    bool imuReceived_{false};

    ros::Subscriber footStateSub_;
    legged_hexapod_hw::FootState footStateMsg_;
    bool footStateReceived_{false};

    // Command
    ros::Publisher jointCmdPub_;
    // legged_hexapod_hw::JointCmd jointCmdMsg_; // FIXME: This cause core dump
  };

} // namespace legged
