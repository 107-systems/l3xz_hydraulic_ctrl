/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_pump_ctrl/graphs/contributors.
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
#include <ros2_loop_rate_monitor/Monitor.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
  Node();
  ~Node();


private:
  static std::chrono::milliseconds constexpr HEARTBEAT_LOOP_RATE{100};
  heartbeat::Publisher::SharedPtr _heartbeat_pub;
  void init_heartbeat();

  float _pressure_0_actual_pascal, _pressure_1_actual_pascal;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _pressure_0_sub, _pressure_1_sub;
  void init_sub();

  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _pump_readiness_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pump_rpm_setpoint_pub;
  void init_pub();
  void pump_publish_readiness();
  void pump_publish_rpm_setpoint();

  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};
  loop_rate::Monitor::SharedPtr _ctrl_loop_rate_monitor;
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void ctrl_loop();

  float _pump_rpm_setpoint;

  enum class State { Startup, Control };
  State _state;
  std::chrono::steady_clock::time_point _startup_prev_rpm_inc;
  std::chrono::steady_clock::time_point _control_prev_no_pressure_error;
  State handle_Startup();
  State handle_Control();

  static float constexpr STARTUP_PUMP_RAMP_START_RPM =  20.0f;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
