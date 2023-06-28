/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_pump_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_pump_ctrl/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node()
: rclcpp::Node("l3xz_pump_ctrl")
, _pressure_0_actual_pascal{0.0f}
, _pressure_1_actual_pascal{0.0f}
, _pump_rpm_setpoint{STARTUP_PUMP_RAMP_START_RPM}
, _state{State::Startup}
, _startup_prev_rpm_inc{std::chrono::steady_clock::now()}
, _control_prev_no_pressure_error{std::chrono::steady_clock::now()}
{
  declare_parameter("pump_min_rpm", 800.0f);
  declare_parameter("pump_max_rpm", 1800.0f);
  declare_parameter("pump_timeout_sec", 60);

  init_heartbeat();
  init_sub();
  init_pub();

  _ctrl_loop_rate_monitor = loop_rate::Monitor::create(CTRL_LOOP_RATE, std::chrono::milliseconds(1));
  _ctrl_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->ctrl_loop(); });

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  RCLCPP_INFO(get_logger(), "%s shut down.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_heartbeat()
{
  std::stringstream heartbeat_topic;
  heartbeat_topic << "/l3xz/" << get_name() << "/heartbeat";

  _heartbeat_pub = heartbeat::Publisher::create(*this, heartbeat_topic.str());
}

void Node::init_sub()
{
  _pressure_0_sub = create_subscription<std_msgs::msg::Float32>(
    "/l3xz/pressure_0/actual",
    1,
    [this](std_msgs::msg::Float32::SharedPtr const msg)
    {
      _pressure_0_actual_pascal = msg->data;
    });

  _pressure_1_sub = create_subscription<std_msgs::msg::Float32>(
    "/l3xz/pressure_1/actual",
    1,
    [this](std_msgs::msg::Float32::SharedPtr const msg)
    {
      _pressure_1_actual_pascal = msg->data;
    });
}

void Node::init_pub()
{
  _pump_readiness_pub = create_publisher<std_msgs::msg::Int8>("/l3xz/pump/readiness/target", 1);
  _pump_rpm_setpoint_pub = create_publisher<std_msgs::msg::Float32>("/l3xz/pump/rpm/target", 1);
}

void Node::pump_publish_readiness()
{
  static int8_t const OREL20_READINESS_ENGAGED = 3;

  std_msgs::msg::Int8 msg;
  msg.data = OREL20_READINESS_ENGAGED;
  _pump_readiness_pub->publish(msg);
}

void Node::pump_publish_rpm_setpoint()
{
  std_msgs::msg::Float32 msg;
  msg.data = _pump_rpm_setpoint;
  _pump_rpm_setpoint_pub->publish(msg);
}

void Node::ctrl_loop()
{
  _ctrl_loop_rate_monitor->update();
  if (auto const [timeout, opt_timeout_duration] = _ctrl_loop_rate_monitor->isTimeout();
    timeout == loop_rate::Monitor::Timeout::Yes)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "ctrl_loop should be called every %ld ms, but is %ld ms instead",
                         CTRL_LOOP_RATE.count(),
                         opt_timeout_duration.value().count());
  }


  /* Perform state dependent actions. */
  switch (_state)
  {
    case State::Startup: _state = handle_Startup(); break;
    case State::Control: _state = handle_Control(); break;
    default: __builtin_unreachable(); break;
  }

  pump_publish_readiness();
  pump_publish_rpm_setpoint();
}

Node::State Node::handle_Startup()
{
  auto const now = std::chrono::steady_clock::now();
  auto const duration_since_last_incr = now - _startup_prev_rpm_inc;

  if (duration_since_last_incr > std::chrono::milliseconds(100))
  {
    _pump_rpm_setpoint += 20.0f;
    _startup_prev_rpm_inc = now;
  }

  /* State transition: */
  if (_pump_rpm_setpoint >= get_parameter("pump_min_rpm").as_double())
  {
    _pump_rpm_setpoint = get_parameter("pump_min_rpm").as_double();
    RCLCPP_INFO(get_logger(), "pump ramp up complete (RPM = %0.1f).", _pump_rpm_setpoint);
    return State::Control;
  }

  return State::Startup;
}

Node::State Node::handle_Control()
{
  bool const is_error_in_pressure_reading =
    (_pressure_0_actual_pascal < 0.0) || (_pressure_1_actual_pascal < 0.0);

  if (is_error_in_pressure_reading)
  {
    _pump_rpm_setpoint = get_parameter("pump_min_rpm").as_double();

    RCLCPP_ERROR_THROTTLE(get_logger(),
                          *get_clock(),
                          1000,
                          "P0 = %0.1f, P1 = %0.1f -> RPM = %0.1f : error in pressure sensing, defaulting to preset RPM.",
                          _pressure_0_actual_pascal / (100*1000.0f),
                          _pressure_1_actual_pascal / (100*1000.0f),
                          _pump_rpm_setpoint);

    return State::Control;
  }

  static float constexpr PRESSURE_TARGET_Pascal = 10.f * (100*1000.0f); /* 10 bar */
  float pressure_error_pascal = 0.0f;

  if (_pressure_0_actual_pascal < PRESSURE_TARGET_Pascal)
    pressure_error_pascal += (PRESSURE_TARGET_Pascal - _pressure_0_actual_pascal);
  if (_pressure_1_actual_pascal < PRESSURE_TARGET_Pascal)
    pressure_error_pascal += (PRESSURE_TARGET_Pascal - _pressure_1_actual_pascal);

  if (pressure_error_pascal <= 0.0f)
  {
    _pump_rpm_setpoint = get_parameter("pump_min_rpm").as_double();

    RCLCPP_INFO_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "P0 = %0.1f, P1 = %0.1f -> RPM = %0.1f : pressure control OFF.",
                         _pressure_0_actual_pascal / (100*1000.0f),
                         _pressure_1_actual_pascal / (100*1000.0f),
                         _pump_rpm_setpoint);

    _control_prev_no_pressure_error = std::chrono::steady_clock::now();
    return State::Control;
  }

  auto const pressure_error_duration = std::chrono::steady_clock::now() - _control_prev_no_pressure_error;
  if (pressure_error_duration > std::chrono::seconds(get_parameter("pump_timeout_sec").as_int()))
  {
    _pump_rpm_setpoint = static_cast<float>(get_parameter("pump_min_rpm").as_double());
    RCLCPP_ERROR_THROTTLE(get_logger(),
                          *get_clock(),
                          1000,
                          "P0 = %0.1f, P1 = %0.1f -> RPM = %0.1f : pressure control OFF (could not stabilise).",
                          _pressure_0_actual_pascal / (100*1000.0f),
                          _pressure_1_actual_pascal / (100*1000.0f),
                          _pump_rpm_setpoint);

    return State::Control;
  }

  static float constexpr k_PRESSURE_ERROR = 100.0f;
  float const pressure_error_bar = pressure_error_pascal / (100*1000.0f);
  float const pump_rpm_setpoint_increase = k_PRESSURE_ERROR * pressure_error_bar;

  _pump_rpm_setpoint = get_parameter("pump_min_rpm").as_double() + pump_rpm_setpoint_increase;
  _pump_rpm_setpoint = std::min(_pump_rpm_setpoint, static_cast<float>(get_parameter("pump_max_rpm").as_double()));

  RCLCPP_INFO_THROTTLE(get_logger(),
                       *get_clock(),
                       1000,
                       "P0 = %0.1f, P1 = %0.1f -> RPM = %0.1f : pressure control ON.",
                       _pressure_0_actual_pascal / (100*1000.0f),
                       _pressure_1_actual_pascal / (100*1000.0f),
                       _pump_rpm_setpoint);

  /* State transition: */
  return State::Control;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
