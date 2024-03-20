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
namespace legged
{

  // Define JOINT_STATE_NAME and FOOT_LINK_NAME constants
  const std::vector<std::string> JOINT_NAME = {"RF_HAA", "RF_HFE", "RF_KFE",
                                                     "RM_HAA", "RM_HFE", "RM_KFE",
                                                     "RB_HAA", "RB_HFE", "RB_KFE",
                                                     "LF_HAA", "LF_HFE", "LF_KFE",
                                                     "LM_HAA", "LM_HFE", "LM_KFE",
                                                     "LB_HAA", "LB_HFE", "LB_KFE"};

  const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "RM_FOOT", "RB_FOOT",
                                                         "LF_FOOT", "LM_FOOT", "LB_FOOT"};

  // const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

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
     *
     * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
     *
     * @param time Current time
     * @param period Current time - last time
     */
    void read(const ros::Time &time, const ros::Duration &period) override;

    /** \brief Comunicate with hardware. Publish command to robot.
     *
     * Propagate joint state to actuator state for the stored
     * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
     * current state.
     *
     * @param time Current time
     * @param period Current time - last time
     */
    void write(const ros::Time &time, const ros::Duration &period) override;

    void updateContact(const ros::Time &time);

  private:
    bool setupJoints();

    bool setupImu();

    bool setupContactSensor(ros::NodeHandle &nh);

    UnitreeMotorData jointData_[18]{}; // NOLINT(modernize-avoid-c-arrays)
    UnitreeImuData imuData_{};
    bool contactState_[6]{}; // NOLINT(modernize-avoid-c-arrays)

    int contactThreshold_{};

    ros::Publisher contactPublisher_;
    ros::Time lastContactPub_;

    // Feedback
    // JointState of Hexapod
    ros::Subscriber jointStateSub_;
    sensor_msgs::JointState jointStateMsg_;
    // IMU of Hexapod
    ros::Subscriber imuSub_;
    sensor_msgs::Imu imuMsg_;

    // Command
    ros::Publisher jointCmdPub_;
    legged_hexapod_hw::JointCmd jointCmdMsg_;
  };

} // namespace legged
