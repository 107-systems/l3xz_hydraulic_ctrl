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

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int64.hpp>

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
  std::chrono::steady_clock::time_point const _node_start;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr _heartbeat_pub;
  static std::chrono::milliseconds constexpr HEARTBEAT_LOOP_RATE{100};
  rclcpp::TimerBase::SharedPtr _heartbeat_loop_timer;
  void init_heartbeat();

  std::map<HydraulicLegJointKey, float> _angle_actual_rad_map;
  std::map<HydraulicLegJointKey,
           rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_actual_sub;
  std::map<HydraulicLegJointKey, float> _angle_target_rad_map;
  std::map<HydraulicLegJointKey,
           rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_target_sub;
  void init_sub();

  void init_pub();

  std::chrono::steady_clock::time_point _prev_ctrl_loop_timepoint;
  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void ctrl_loop();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
