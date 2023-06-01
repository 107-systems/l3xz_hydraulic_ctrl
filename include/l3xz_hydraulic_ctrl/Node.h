/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_hydraulic_ctrl/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <tuple>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <ros2_heartbeat/Publisher.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * TYPEDEFS
 **************************************************************************************/

enum class HydraulicJoint { Femur, Tibia };
enum class Leg { LeftFront, LeftMiddle, LeftBack, RightFront, RightMiddle, RightBack };
typedef std::tuple<Leg, HydraulicJoint> HydraulicLegJointKey;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
  Node();


private:
  static std::chrono::milliseconds constexpr HEARTBEAT_LOOP_RATE{100};
  heartbeat::Publisher::SharedPtr _heartbeat_pub;
  void init_heartbeat();

  std::map<HydraulicLegJointKey, float> _angle_actual_rad_map;
  std::map<HydraulicLegJointKey,
           rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_actual_sub;
  std::map<HydraulicLegJointKey, float> _angle_target_rad_map;
  std::map<HydraulicLegJointKey,
           rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_target_sub;
  float _pressure_0_actual_pascal, _pressure_1_actual_pascal;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _pressure_0_sub, _pressure_1_sub;
  void init_sub();

  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _pump_readiness_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pump_rpm_setpoint_pub;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr _servo_pulse_width_pub;
  void init_pub();

  std::chrono::steady_clock::time_point _prev_ctrl_loop_timepoint;
  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void ctrl_loop();

  enum class State { Startup, Control };
  State _state;
  State handle_Startup();
  State handle_Control();


  static float constexpr STARTUP_PUMP_RAMP_START_RPM =  20.0f;
  static float constexpr STARTUP_PUMP_RAMP_STOP_RPM  = 200.0f;
  std::chrono::steady_clock::time_point _startup_prev_rpm_inc;
  float _pump_rpm_setpoint;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
