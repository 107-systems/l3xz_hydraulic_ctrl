/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_hydraulic_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_hydraulic_ctrl/Node.h>

#include <l3xz_hydraulic_ctrl/const/LegList.h>
#include <l3xz_hydraulic_ctrl/const/HydraulicJointList.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node()
: rclcpp::Node("l3xz_hydraulic_ctrl")
, _node_start{std::chrono::steady_clock::now()}
, _prev_ctrl_loop_timepoint{std::chrono::steady_clock::now()}
{
  init_heartbeat();
  init_sub();
  init_pub();

  _ctrl_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->ctrl_loop(); });

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_heartbeat()
{
  std::stringstream heartbeat_topic;
  heartbeat_topic << "/l3xz/" << get_name() << "/heartbeat";
  _heartbeat_pub = create_publisher<std_msgs::msg::UInt64>(heartbeat_topic.str(), 1);
  _heartbeat_loop_timer = create_wall_timer(HEARTBEAT_LOOP_RATE,
                                            [this]()
                                            {
                                              std_msgs::msg::UInt64 heartbeat_msg;
                                              heartbeat_msg.data = std::chrono::duration_cast<std::chrono::seconds>(
                                                std::chrono::steady_clock::now() - _node_start).count();
                                              _heartbeat_pub->publish(heartbeat_msg);
                                            });
}

void Node::init_sub()
{
  for (auto leg : LEG_LIST)
    for (auto joint : HYDRAULIC_JOINT_LIST)
    {
      std::stringstream angle_actual_sub_topic;
      angle_actual_sub_topic << "/l3xz/leg/" << LegToStr(leg) << "/" << JointToStr(joint) << "/angle/actual";

      _angle_actual_sub[make_key(leg, joint)] = create_subscription<std_msgs::msg::Float32>(
        angle_actual_sub_topic.str(),
        1,
        [this, leg, joint](std_msgs::msg::Float32::SharedPtr const /* msg */) { /* TODO _gait_ctrl_input.set_angle_deg(leg, joint, msg->data * 180.0f / M_PI); */ });
    }
}

void Node::init_pub()
{
  for (auto leg : LEG_LIST)
    for (auto joint : HYDRAULIC_JOINT_LIST)
    {
      std::stringstream angle_target_pub_topic;
      angle_target_pub_topic << "/l3xz/leg/" << LegToStr(leg) << "/" << JointToStr(joint) << "/angle/target";

      _angle_target_pub[make_key(leg, joint)] = create_publisher<std_msgs::msg::Float32>(angle_target_pub_topic.str(), 1);
    }
}

void Node::ctrl_loop()
{
  auto const now = std::chrono::steady_clock::now();
  auto const ctrl_loop_rate = (now - _prev_ctrl_loop_timepoint);
  if (ctrl_loop_rate > (CTRL_LOOP_RATE + std::chrono::milliseconds(1)))
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "ctrl_loop should be called every %ld ms, but is %ld ms instead",
                         CTRL_LOOP_RATE.count(),
                         std::chrono::duration_cast<std::chrono::milliseconds>(ctrl_loop_rate).count());
  _prev_ctrl_loop_timepoint = now;


//  for (auto leg : LEG_LIST)
//    for (auto joint : HYDRAULIC_JOINT_LIST)
//    {
//      /* Control hydraulic valves via servos and pump. */
//    }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
