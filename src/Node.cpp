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


  /* Periodically send engaged message in order to keep the Orel 20
   * from accepting drive commands (keep it ready).
   */
  {
    static int8_t const OREL20_READINESS_ENGAGED = 3;
    std_msgs::msg::Int8 msg;
    msg.data = OREL20_READINESS_ENGAGED;
    _pump_readiness_pub->publish(msg);
  }

  /* Compare all actual to target angles and calculate the necessary
   * RPM speed as a dependency of this.
   */
  std::map<HydraulicLegJointKey, float> angle_diff_rad_map;
  for (auto leg: LEG_LIST)
    for (auto joint: HYDRAULIC_JOINT_LIST)
    {
      float const angle_actual_rad = _angle_actual_rad_map.at(make_key(leg, joint));
      float const angle_target_rad = _angle_target_rad_map.at(make_key(leg, joint));
      float const angle_diff_rad = angle_actual_rad - angle_target_rad;

      angle_diff_rad_map[make_key(leg, joint)] = angle_diff_rad;
    }

  /* Calculate desired RPM set point and publish the necessary
   * ROS message which will then be translated to the Cyphal layer.
   */
  {
    static float const K_PUMP = 10.0f;

    float const abs_angle_diff_sum = std::accumulate(
      std::begin(angle_diff_rad_map),
      std::end(angle_diff_rad_map),
      0,
      [] (float value, std::map<HydraulicLegJointKey, float>::value_type const & p) { return value + p.second; });

    float const pump_rpm_setpoint = K_PUMP * abs_angle_diff_sum;

    std_msgs::msg::Float32 msg;
    msg.data = pump_rpm_setpoint;
    _pump_rpm_setpoint_pub->publish(msg);
  }

  /* Calculate the desired servo pulse width which are
   * turned into ROS message -> Cyphal message ->
   * servo pulse width via OpenCyphalServoController-12
   */
  {
    std::array<uint16_t, 12> servo_pulse_width;

    auto const angle_diff_to_pulse_width_us =
      [](float const angle_diff_rad) -> uint16_t
      {
        static float constexpr ANGLE_DIFF_EPSILON_rad = 2.0f * M_PI / 180.0f;

        static uint16_t constexpr SERVO_PULSE_WIDTH_NEUTRAL_us = 1500;

        if (fabs(angle_diff_rad) < ANGLE_DIFF_EPSILON_rad)
          return SERVO_PULSE_WIDTH_NEUTRAL_us;

        if (angle_diff_rad < 0.0)
          return 1200U;
        else
          return 1800U;
      };

    servo_pulse_width[ 0] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::LeftFront,   HydraulicJoint::Femur)));
    servo_pulse_width[ 1] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::LeftFront,   HydraulicJoint::Tibia)));
    servo_pulse_width[ 2] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::LeftMiddle,  HydraulicJoint::Femur)));
    servo_pulse_width[ 3] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::LeftMiddle,  HydraulicJoint::Tibia)));
    servo_pulse_width[ 4] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::LeftBack,    HydraulicJoint::Femur)));
    servo_pulse_width[ 5] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::LeftBack,    HydraulicJoint::Tibia)));

    servo_pulse_width[ 6] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::RightFront,  HydraulicJoint::Femur)));
    servo_pulse_width[ 7] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::RightFront,  HydraulicJoint::Tibia)));
    servo_pulse_width[ 8] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::RightMiddle, HydraulicJoint::Femur)));
    servo_pulse_width[ 9] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::RightMiddle, HydraulicJoint::Tibia)));
    servo_pulse_width[10] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::RightBack,   HydraulicJoint::Femur)));
    servo_pulse_width[11] = angle_diff_to_pulse_width_us(angle_diff_rad_map.at(make_key(Leg::RightBack,   HydraulicJoint::Tibia)));

    std_msgs::msg::UInt16MultiArray msg;
    /* Configure dimensions. */
    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.dim[0].size = servo_pulse_width.size();
    /* Copy in the data. */
    msg.data.clear();
    msg.data.insert(msg.data.end(), servo_pulse_width.begin(), servo_pulse_width.end());
    /* Publish the message. */
    _servo_pulse_width_pub->publish(msg);
  }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
