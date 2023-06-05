/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_hydraulic_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_hydraulic_ctrl/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * MODULE INTERNAL FUNCTIONS
 **************************************************************************************/

static std::string JointToStr(HydraulicJoint const joint)
{
  switch(joint)
  {
    case HydraulicJoint::Femur: return std::string("femur"); break;
    case HydraulicJoint::Tibia: return std::string("tibia"); break;
    default: __builtin_unreachable();
  }
}

static std::string LegToStr(Leg const leg)
{
  switch(leg)
  {
    case Leg::LeftFront:   return std::string("left_front");   break;
    case Leg::LeftMiddle:  return std::string("left_middle");  break;
    case Leg::LeftBack:    return std::string("left_back");    break;
    case Leg::RightFront:  return std::string("right_front");  break;
    case Leg::RightMiddle: return std::string("right_middle"); break;
    case Leg::RightBack:   return std::string("right_back");   break;
    default: __builtin_unreachable();
  }
}

inline HydraulicLegJointKey make_key(Leg const leg, HydraulicJoint const joint)
{
  return std::tuple(leg, joint);
}

struct leg_joint_map_key_equal : public std::binary_function<HydraulicLegJointKey, HydraulicLegJointKey, bool>
{
  bool operator()(const HydraulicLegJointKey & v0, const HydraulicLegJointKey & v1) const
  {
    return (
      std::get<0>(v0) == std::get<0>(v1) &&
      std::get<1>(v0) == std::get<1>(v1)
    );
  }
};

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static std::list<Leg> const LEG_LIST =
{
  Leg::LeftFront, Leg::LeftMiddle, Leg::LeftBack, Leg::RightFront, Leg::RightMiddle, Leg::RightBack
};

static std::list<HydraulicJoint> const HYDRAULIC_JOINT_LIST =
{
  HydraulicJoint::Femur, HydraulicJoint::Tibia
};

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node()
: rclcpp::Node("l3xz_hydraulic_ctrl")
, _pressure_0_actual_pascal{0.0f}
, _pressure_1_actual_pascal{0.0f}
, _prev_ctrl_loop_timepoint{std::chrono::steady_clock::now()}
, _pump_rpm_setpoint{STARTUP_PUMP_RAMP_START_RPM}
, _servo_pulse_width{DEFAULT_SERVO_PULSE_WIDTH}
, _state{State::Startup}
, _startup_prev_rpm_inc{std::chrono::steady_clock::now()}
, _control_prev_no_pressure_error{std::chrono::steady_clock::now()}
{
  init_heartbeat();
  init_sub();
  init_pub();

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

  _heartbeat_pub = heartbeat::Publisher::create(*this, heartbeat_topic.str(), HEARTBEAT_LOOP_RATE);
}

void Node::init_sub()
{
  for (auto leg : LEG_LIST)
    for (auto joint : HYDRAULIC_JOINT_LIST)
    {
      {
        std::stringstream angle_actual_sub_topic;
        angle_actual_sub_topic << "/l3xz/leg/" << LegToStr(leg) << "/" << JointToStr(joint) << "/angle/actual";

        _angle_actual_rad_map[make_key(leg, joint)] = 0.0f;

        _angle_actual_sub[make_key(leg, joint)] = create_subscription<std_msgs::msg::Float32>(
          angle_actual_sub_topic.str(),
          1,
          [this, leg, joint](std_msgs::msg::Float32::SharedPtr const msg)
          {
            _angle_actual_rad_map.at(make_key(leg, joint)) = msg->data;
          });
      }
      {
        std::stringstream angle_target_sub_topic;
        angle_target_sub_topic << "/l3xz/leg/" << LegToStr(leg) << "/" << JointToStr(joint) << "/angle/target";

        _angle_target_rad_map[make_key(leg, joint)] = 0.0f;

        _angle_target_sub[make_key(leg, joint)] = create_subscription<std_msgs::msg::Float32>(
          angle_target_sub_topic.str(),
          1,
          [this, leg, joint](std_msgs::msg::Float32::SharedPtr const msg)
          {
            _angle_target_rad_map.at(make_key(leg, joint)) = msg->data;
          });
      }
    }

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
  _servo_pulse_width_pub = create_publisher<std_msgs::msg::UInt16MultiArray>("/l3xz/servo_pulse_width/target", 1);
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

void Node::valve_block_publish_servo_pulse_width()
{
  std_msgs::msg::UInt16MultiArray msg;
  /* Configure dimensions. */
  msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  msg.layout.dim[0].size = _servo_pulse_width.size();
  /* Copy in the data. */
  msg.data.clear();
  msg.data.insert(msg.data.end(), _servo_pulse_width.begin(), _servo_pulse_width.end());
  /* Publish the message. */
  _servo_pulse_width_pub->publish(msg);
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


  /* Perform state dependent actions. */
  switch (_state)
  {
    case State::Startup: _state = handle_Startup(); break;
    case State::Control: _state = handle_Control(); break;
    default: __builtin_unreachable(); break;
  }

  pump_publish_readiness();
  pump_publish_rpm_setpoint();
  valve_block_publish_servo_pulse_width();
}

Node::State Node::handle_Startup()
{
  /* Valve block: */
  _servo_pulse_width = DEFAULT_SERVO_PULSE_WIDTH;

  /* Pump: */
  auto const now = std::chrono::steady_clock::now();
  auto const duration_since_last_incr = now - _startup_prev_rpm_inc;

  if (duration_since_last_incr > std::chrono::milliseconds(100))
  {
    _pump_rpm_setpoint += 10.0f;
    _startup_prev_rpm_inc = now;
  }

  /* State transition: */
  if (_pump_rpm_setpoint >= STARTUP_PUMP_RAMP_STOP_RPM)
  {
    _pump_rpm_setpoint = STARTUP_PUMP_RAMP_STOP_RPM;
    RCLCPP_INFO(get_logger(), "pump ramp up complete (RPM = %0.1f).", _pump_rpm_setpoint);
    return State::Control;
  }

  return State::Startup;
}

Node::State Node::handle_Control()
{
  /* Valve block: */
  _servo_pulse_width = calc_ServoPulseWidth(_angle_actual_rad_map, _angle_target_rad_map);

  /* Pump: */
  bool is_error_in_pressure_reading = false;
  if (_pressure_0_actual_pascal < 0.0)
  {
    is_error_in_pressure_reading = true;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "error obtaining pressure from circuit #0 (possibly loose cable)");
  }
  if (_pressure_1_actual_pascal < 0.0)
  {
    is_error_in_pressure_reading = true;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "error obtaining pressure from circuit #1 (possibly loose cable)");
  }

  if (is_error_in_pressure_reading) {
    _pump_rpm_setpoint = STARTUP_PUMP_RAMP_STOP_RPM;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "error in pressure sensor circuits, defaulting to RPM = %0.1f.", _pump_rpm_setpoint);
    return State::Control;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "P[0] = %0.1f Bar, P[1] = %0.1f Bar",
                       _pressure_0_actual_pascal / (100*1000.0f), _pressure_1_actual_pascal / (100*1000.0f));


  static float constexpr PRESSURE_TARGET_Pascal = 10.f * (100*1000.0f); /* 10 bar */
  float pressure_error_pascal = 0.0f;

  if (_pressure_0_actual_pascal < PRESSURE_TARGET_Pascal) {
    pressure_error_pascal += (PRESSURE_TARGET_Pascal - _pressure_0_actual_pascal);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Low pressure in circuit #0: %0.1f", _pressure_0_actual_pascal / (100*1000.0f));
  }

  if (_pressure_1_actual_pascal < PRESSURE_TARGET_Pascal) {
    pressure_error_pascal += (PRESSURE_TARGET_Pascal - _pressure_1_actual_pascal);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Low pressure in circuit #1: %0.1f", _pressure_1_actual_pascal / (100*1000.0f));
  }

  if (pressure_error_pascal <= 0.0f)
  {
    _pump_rpm_setpoint = STARTUP_PUMP_RAMP_STOP_RPM;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Both pressure circuits at or above target, defaulting to RPM = %0.1f.", _pump_rpm_setpoint);
    _control_prev_no_pressure_error = std::chrono::steady_clock::now();
    return State::Control;
  }

  static float constexpr k_PRESSURE_ERROR = 100.0f;
  float const pressure_error_bar = pressure_error_pascal / (100*1000.0f);
  float const pump_rpm_setpoint_increase = k_PRESSURE_ERROR * pressure_error_bar;

  _pump_rpm_setpoint = STARTUP_PUMP_RAMP_STOP_RPM + pump_rpm_setpoint_increase;
  _pump_rpm_setpoint = std::min(_pump_rpm_setpoint, 1000.0f);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "pressur_error_bar = %0.1f, _pump_rpm_setpoint = %0.1f", pressure_error_bar, _pump_rpm_setpoint);

  /* State transition: */
  return State::Control;
}

Node::ServoPulseWidth const Node::calc_ServoPulseWidth(std::map<HydraulicLegJointKey, float> const & angle_actual_rad_map,
                                                       std::map<HydraulicLegJointKey, float> const & angle_target_rad_map)
{
  std::map<HydraulicLegJointKey, float> angle_diff_rad_map;
  for (auto leg: LEG_LIST)
    for (auto joint: HYDRAULIC_JOINT_LIST)
    {
      float const angle_actual_rad = angle_actual_rad_map.at(make_key(leg, joint));
      float const angle_target_rad = angle_target_rad_map.at(make_key(leg, joint));
      float const angle_diff_rad = angle_actual_rad - angle_target_rad;

      angle_diff_rad_map[make_key(leg, joint)] = angle_diff_rad;
    }

  auto const angle_diff_to_pulse_width_us =
    [](float const angle_diff_rad) -> uint16_t
    {
      static float constexpr ANGLE_DIFF_EPSILON_rad = 2.5f * M_PI / 180.0f;

      if (fabs(angle_diff_rad) < ANGLE_DIFF_EPSILON_rad)
        return SERVO_PULSE_WIDTH_NEUTRAL_us;

      float const k_ANGLE_DIFF = 50.0f;

      if (angle_diff_rad > 0.0)
      {
        float const servo_pulse_width = SERVO_PULSE_WIDTH_NEUTRAL_us - (k_ANGLE_DIFF * fabs(angle_diff_rad * 180.f / M_PI));
        return std::max(static_cast<uint16_t>(servo_pulse_width), SERVO_PULSE_WIDTH_MIN_us);
      }
      else
      {
        float const servo_pulse_width = SERVO_PULSE_WIDTH_NEUTRAL_us + (k_ANGLE_DIFF * fabs(angle_diff_rad * 180.f / M_PI));
        return std::min(static_cast<uint16_t>(servo_pulse_width), SERVO_PULSE_WIDTH_MAX_us);
      }
    };


  ServoPulseWidth servo_pulse_width;

  servo_pulse_width[ 0] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::RightBack,   HydraulicJoint::Tibia)));
  servo_pulse_width[ 1] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::RightBack,   HydraulicJoint::Femur)));
  servo_pulse_width[ 2] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::RightMiddle, HydraulicJoint::Tibia)));
  servo_pulse_width[ 3] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::RightMiddle, HydraulicJoint::Femur)));
  servo_pulse_width[ 4] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::RightFront,  HydraulicJoint::Tibia)));
  servo_pulse_width[ 5] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::RightFront,  HydraulicJoint::Femur)));

  servo_pulse_width[ 6] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::LeftFront,   HydraulicJoint::Femur)));
  servo_pulse_width[ 7] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::LeftFront,   HydraulicJoint::Tibia)));
  servo_pulse_width[ 8] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::LeftMiddle,  HydraulicJoint::Femur)));
  servo_pulse_width[ 9] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::LeftMiddle,  HydraulicJoint::Tibia)));
  servo_pulse_width[10] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::LeftBack,    HydraulicJoint::Femur)));
  servo_pulse_width[11] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::LeftBack,    HydraulicJoint::Tibia)));

  return servo_pulse_width;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
