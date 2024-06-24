// Copyright 2023 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file mavlink_platform.hpp
 *
 * PixhawkPlatform class definition
 *
 * @author Miguel Fernández Cortizas
 *         Rafael Pérez Seguí
 *         Pedro Arias Pérez
 *         Javier Melero Deza
 */

#ifndef AS2_PLATFORM_MAVLINK__MAVLINK_PLATFORM_HPP_
#define AS2_PLATFORM_MAVLINK__MAVLINK_PLATFORM_HPP_

#include <Eigen/Dense>

#include <cmath>
#include <string>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <mavros_msgs/msg/command_code.hpp>
#include <mavros_msgs/msg/state.hpp>


#include "as2_core/aerial_platform.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"

namespace as2_platform_mavlink
{

class MavlinkPlatform : public as2::AerialPlatform
{
public:
  explicit MavlinkPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MavlinkPlatform() {}

public:
  void configureSensors() override;
  void publishSensorData();

  // TODO(miferco97): set ATTITUDE as default mode with yaw_speed = 0  and Thrust = 0 N
  void setDefaultControlMode() {}

  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  void sendCommand() override;
  bool ownSendCommand() override;
  void ownKillSwitch() override;
  void ownStopPlatform() override;

  void resetTrajectorySetpoint();
  void resetAttitudeSetpoint();
  void resetRatesSetpoint();

  bool getFlagSimulationMode();

private:
  bool has_mode_settled_ = false;

  std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>> battery_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_raw_estimation_ptr_;
  std::unique_ptr<as2::sensors::GPS> gps_sensor_ptr_;

  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr external_odometry_sub_;
  void externalOdomCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // mavlink_ subscribers
  // rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_imu_sub_;
  // rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odometry_sub_;
  // rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr px4_vehicle_control_mode_sub_;
  // rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr px4_timesync_sub_;
  // rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr px4_battery_sub_;
  // rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr px4_gps_sub_;

  // mavlink_ publishers
  // rclcpp::Publisher<px4_msgs::msg::ManualControlSwitches>::SharedPtr
  //   px4_manual_control_switches_pub_;
  // rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr px4_offboard_control_mode_pub_;
  // rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr px4_trajectory_setpoint_pub_;
  // rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr px4_vehicle_command_pub_;
  // rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
  //   px4_vehicle_attitude_setpoint_pub_;
  // rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr px4_vehicle_rates_setpoint_pub_;

  // mavlink_ Functions
  void mavlink_arm();
  void mavlink_disarm();
  void mavlink_publishOffboardControlMode();
  void mavlink_publishTrajectorySetpoint();
  void mavlink_publishAttitudeSetpoint();
  void mavlink_publishRatesSetpoint();
  void mavlink_publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
  void mavlink_publishVisualOdometry();

private:
  bool manual_from_operator_ = false;
  bool set_disarm_ = false;
  nav_msgs::msg::Odometry odometry_msg_;

  std::atomic<uint64_t> timestamp_;

  // px4_msgs::msg::OffboardControlMode px4_offboard_control_mode_;
  // px4_msgs::msg::TrajectorySetpoint px4_trajectory_setpoint_;
  // px4_msgs::msg::VehicleAttitudeSetpoint px4_attitude_setpoint_;
  // px4_msgs::msg::VehicleRatesSetpoint px4_rates_setpoint_;
  // px4_msgs::msg::VehicleOdometry px4_visual_odometry_msg_;

  float max_thrust_;
  float min_thrust_;
  bool simulation_mode_ = false;
  bool external_odom_ = true;
  std::string base_link_frame_id_;
  std::string odom_frame_id_;

  Eigen::Quaterniond q_ned_to_enu_;
  Eigen::Quaterniond q_enu_to_ned_;
  Eigen::Quaterniond q_aircraft_to_baselink_;
  Eigen::Quaterniond q_baselink_to_aircraft_;

private:
  // mavlink_ Callbacks
  // void mavlink_imuCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
  // void mavlink_odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  // void mavlink_VehicleControlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
  // void mavlink_BatteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
  // void mavlink_GpsCallback(const px4_msgs::msg::SensorGps::SharedPtr msg);
};

}  // namespace as2_platform_mavlink

#endif  // AS2_PLATFORM_MAVLINK__MAVLINK_PLATFORM_HPP_
