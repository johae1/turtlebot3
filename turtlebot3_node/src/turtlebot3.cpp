// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include "turtlebot3_node/turtlebot3.hpp"

#include <memory>
#include <string>
#include <vector>

using robotis::turtlebot3::TurtleBot3;
using namespace std::chrono_literals;

TurtleBot3::TurtleBot3(const std::string & usb_port)
: Node("turtlebot3_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Init TurtleBot3 Node Main");
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  init_dynamixel_sdk_wrapper(usb_port);
  check_device_status();

  add_motors();
  add_wheels();
  add_sensors();
  add_devices();

  run();
}

TurtleBot3::Wheels * TurtleBot3::get_wheels()
{
  return &wheels_;
}

TurtleBot3::Motors * TurtleBot3::get_motors()
{
  return &motors_;
}

void TurtleBot3::init_dynamixel_sdk_wrapper(const std::string & usb_port)
{
  DynamixelSDKWrapper::Device opencr = {usb_port, 200, 1000000, 2.0f};

  this->declare_parameter<uint8_t>("opencr.id");
  this->declare_parameter<int>("opencr.baud_rate");
  this->declare_parameter<float>("opencr.protocol_version");
  this->declare_parameter<std::string>("namespace");

  this->get_parameter_or<uint8_t>("opencr.id", opencr.id, 200);
  this->get_parameter_or<int>("opencr.baud_rate", opencr.baud_rate, 1000000);
  this->get_parameter_or<float>("opencr.protocol_version", opencr.protocol_version, 2.0f);

  RCLCPP_INFO(this->get_logger(), "Init DynamixelSDKWrapper");

  dxl_sdk_wrapper_ = std::make_shared<DynamixelSDKWrapper>(opencr);

  dxl_sdk_wrapper_->init_read_memory(
    extern_control_table.millis.addr,
    (extern_control_table.profile_acceleration_right.addr - extern_control_table.millis.addr) +
    extern_control_table.profile_acceleration_right.length
  );
}

void TurtleBot3::check_device_status()
{
  if (dxl_sdk_wrapper_->is_connected_to_device()) {
    std::string sdk_msg;
    uint8_t reset = 1;

    dxl_sdk_wrapper_->set_data_to_device(
      extern_control_table.imu_re_calibration.addr,
      extern_control_table.imu_re_calibration.length,
      &reset,
      &sdk_msg);

    RCLCPP_INFO(this->get_logger(), "Start Calibration of Gyro");
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Calibration End");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed connection with Devices");
    rclcpp::shutdown();
    return;
  }

  const int8_t NOT_CONNECTED_MOTOR = -1;

  int8_t device_status = dxl_sdk_wrapper_->get_data_from_device<int8_t>(
    extern_control_table.device_status.addr,
    extern_control_table.device_status.length);

  switch (device_status) {
    case NOT_CONNECTED_MOTOR:
      RCLCPP_WARN(this->get_logger(), "Please double check your Dynamixels and Power");
      break;

    default:
      break;
  }
}

void TurtleBot3::add_motors()
{
  RCLCPP_INFO(this->get_logger(), "Add Motors");

  this->declare_parameter<int>("motors.operating_mode", 1);
  this->declare_parameter<float>("motors.profile_acceleration_constant");
  this->declare_parameter<float>("motors.profile_acceleration");

  this->declare_parameter<bool>("segway.ctrl_enable", false);
  this->declare_parameter<std::vector<double>>(
    "segway.k_ext",
    std::vector<double>{-2.5604, -5.5234, -0.6686, 0.8714});

  this->get_parameter_or<int>(
    "motors.operating_mode",
    motors_.operating_mode,
    1);

  motors_.operating_mode = std::clamp(motors_.operating_mode, 0, 255);

  {
    std::string sdk_msg;
    uint8_t operating_mode = static_cast<uint8_t>(motors_.operating_mode);
    dxl_sdk_wrapper_->set_data_to_device(
      extern_control_table.operating_mode.addr,
      extern_control_table.operating_mode.length,
      &operating_mode,
      &sdk_msg);
    RCLCPP_INFO(
      this->get_logger(),
      "Set initial operating_mode : %d sdk_msg : %s",
      motors_.operating_mode,
      sdk_msg.c_str());
  }

  this->get_parameter_or<float>(
    "motors.profile_acceleration_constant",
    motors_.profile_acceleration_constant,
    214.577);

  this->get_parameter_or<float>(
    "motors.profile_acceleration",
    motors_.profile_acceleration,
    0.0);

  this->get_parameter_or<bool>("segway.ctrl_enable", segway_ctrl_enable_, false);
  std::vector<double> segway_k_ext_param;
  this->get_parameter_or<std::vector<double>>(
    "segway.k_ext",
    segway_k_ext_param,
    std::vector<double>{-2.5604, -5.5234, -0.6686, 0.8714});

  if (segway_k_ext_param.size() < 4) {
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter segway.k_ext has %zu values, expected 4. Missing values are set to 0.0.",
      segway_k_ext_param.size());
  } else if (segway_k_ext_param.size() > 4) {
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter segway.k_ext has %zu values, expected 4. Extra values are ignored.",
      segway_k_ext_param.size());
  }

  for (size_t i = 0; i < 4; ++i) {
    segway_k_ext_[i] =
      static_cast<float>(i < segway_k_ext_param.size() ? segway_k_ext_param[i] : 0.0);
  }

  {
    std::string sdk_msg;
    uint8_t segway_ctrl_enable = segway_ctrl_enable_ ? 1 : 0;
    dxl_sdk_wrapper_->set_data_to_device(
      extern_control_table.segway_ctrl_enable.addr,
      extern_control_table.segway_ctrl_enable.length,
      &segway_ctrl_enable,
      &sdk_msg);

    union Data {
      float values[4];
      uint8_t bytes[4 * 4];
    } data;

    data.values[0] = segway_k_ext_[0];
    data.values[1] = segway_k_ext_[1];
    data.values[2] = segway_k_ext_[2];
    data.values[3] = segway_k_ext_[3];

    uint16_t start_addr = extern_control_table.segway_k_ext_1.addr;
    uint16_t addr_length =
      (extern_control_table.segway_k_ext_4.addr -
      extern_control_table.segway_k_ext_1.addr) +
      extern_control_table.segway_k_ext_4.length;

    dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, &data.bytes[0], &sdk_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Set initial segway params : ctrl_enable=%d, k_ext=[%f, %f, %f, %f], sdk_msg=%s",
      segway_ctrl_enable_ ? 1 : 0,
      segway_k_ext_[0],
      segway_k_ext_[1],
      segway_k_ext_[2],
      segway_k_ext_[3],
      sdk_msg.c_str());
  }
}

void TurtleBot3::add_wheels()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels");

  this->declare_parameter<float>("wheels.separation");
  this->declare_parameter<float>("wheels.radius");

  this->get_parameter_or<float>("wheels.separation", wheels_.separation, 0.160);
  this->get_parameter_or<float>("wheels.radius", wheels_.radius, 0.033);
}

void TurtleBot3::add_sensors()
{
  RCLCPP_INFO(this->get_logger(), "Add Sensors");

  uint8_t is_connected_bumper_1 = 0;
  uint8_t is_connected_bumper_2 = 0;
  uint8_t is_connected_illumination = 0;
  uint8_t is_connected_ir = 0;
  uint8_t is_connected_sonar = 0;

  this->declare_parameter<uint8_t>("sensors.bumper_1");
  this->declare_parameter<uint8_t>("sensors.bumper_2");
  this->declare_parameter<uint8_t>("sensors.illumination");
  this->declare_parameter<uint8_t>("sensors.ir");
  this->declare_parameter<uint8_t>("sensors.sonar");

  this->get_parameter_or<uint8_t>(
    "sensors.bumper_1",
    is_connected_bumper_1,
    0);
  this->get_parameter_or<uint8_t>(
    "sensors.bumper_2",
    is_connected_bumper_2,
    0);
  this->get_parameter_or<uint8_t>(
    "sensors.illumination",
    is_connected_illumination,
    0);
  this->get_parameter_or<uint8_t>(
    "sensors.ir",
    is_connected_ir,
    0);
  this->get_parameter_or<uint8_t>(
    "sensors.sonar",
    is_connected_sonar,
    0);

  sensors_.push_back(
    new sensors::BatteryState(
      node_handle_,
      "battery_state"));

  sensors_.push_back(
    new sensors::Imu(
      node_handle_,
      "imu",
      "magnetic_field",
      "imu_link"));

  sensors_.push_back(
    new sensors::SensorState(
      node_handle_,
      "sensor_state",
      is_connected_bumper_1,
      is_connected_bumper_2,
      is_connected_illumination,
      is_connected_ir,
      is_connected_sonar));

  dxl_sdk_wrapper_->read_data_set();
  sensors_.push_back(
    new sensors::JointState(
      node_handle_,
      dxl_sdk_wrapper_,
      "joint_states",
      "base_link"));
}

void TurtleBot3::add_devices()
{
  RCLCPP_INFO(this->get_logger(), "Add Devices");
  devices_["motor_power"] =
    new devices::MotorPower(node_handle_, dxl_sdk_wrapper_, "motor_power");
  devices_["reset"] =
    new devices::Reset(node_handle_, dxl_sdk_wrapper_, "reset");
  devices_["sound"] =
    new devices::Sound(node_handle_, dxl_sdk_wrapper_, "sound");
}

void TurtleBot3::run()
{
  RCLCPP_INFO(this->get_logger(), "Run!");

  int publish_period_ms = this->declare_parameter<int>("publish_period_ms", 5);
  this->get_parameter_or<int>("publish_period_ms", publish_period_ms, 5);

  publish_timer(std::chrono::milliseconds(publish_period_ms));
  heartbeat_timer(std::chrono::milliseconds(100));

  parameter_event_callback();
  cmd_vel_callback();
  cmd_pwm_callback();
}

void TurtleBot3::publish_timer(const std::chrono::milliseconds timeout)
{
  publish_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      rclcpp::Time now = this->now();

      dxl_sdk_wrapper_->read_data_set();

      for (const auto & sensor : sensors_) {
        sensor->publish(now, dxl_sdk_wrapper_);
      }
    }
  );
}

void TurtleBot3::heartbeat_timer(const std::chrono::milliseconds timeout)
{
  heartbeat_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      static uint8_t count = 0;
      std::string msg;

      dxl_sdk_wrapper_->set_data_to_device(
        extern_control_table.heartbeat.addr,
        extern_control_table.heartbeat.length,
        &count,
        &msg);

      RCLCPP_DEBUG(this->get_logger(), "hearbeat count : %d, msg : %s", count, msg.c_str());

      count++;
    }
  );
}

void TurtleBot3::parameter_event_callback()
{
  priv_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  while (!priv_parameters_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }

    RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
  }

  auto param_event_callback =
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
      for (const auto & changed_parameter : event->changed_parameters) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "changed parameter name : %s",
          changed_parameter.name.c_str());

        if (changed_parameter.name == "motors.profile_acceleration") {
          std::string sdk_msg;

          motors_.profile_acceleration =
            rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();

          motors_.profile_acceleration =
            motors_.profile_acceleration / motors_.profile_acceleration_constant;

          union Data {
            int32_t dword[2];
            uint8_t byte[4 * 2];
          } data;

          data.dword[0] = static_cast<int32_t>(motors_.profile_acceleration);
          data.dword[1] = static_cast<int32_t>(motors_.profile_acceleration);

          uint16_t start_addr = extern_control_table.profile_acceleration_left.addr;
          uint16_t addr_length =
            (extern_control_table.profile_acceleration_right.addr -
            extern_control_table.profile_acceleration_left.addr) +
            extern_control_table.profile_acceleration_right.length;

          uint8_t * p_data = &data.byte[0];

          dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

          RCLCPP_INFO(
            this->get_logger(),
            "changed parameter value : %f [rev/min2] sdk_msg : %s",
            motors_.profile_acceleration,
            sdk_msg.c_str());
        }

        if (changed_parameter.name == "motors.operating_mode") {
          std::string sdk_msg;

          motors_.operating_mode =
            rclcpp::Parameter::from_parameter_msg(changed_parameter).as_int();
          motors_.operating_mode = std::clamp(motors_.operating_mode, 0, 255);

          uint8_t operating_mode = static_cast<uint8_t>(motors_.operating_mode);

          dxl_sdk_wrapper_->set_data_to_device(
            extern_control_table.operating_mode.addr,
            extern_control_table.operating_mode.length,
            &operating_mode,
            &sdk_msg);

          RCLCPP_INFO(
            this->get_logger(),
            "changed parameter value : %d [operating_mode] sdk_msg : %s",
            motors_.operating_mode,
            sdk_msg.c_str());
        }

        if (changed_parameter.name == "segway.ctrl_enable") {
          std::string sdk_msg;

          segway_ctrl_enable_ =
            rclcpp::Parameter::from_parameter_msg(changed_parameter).as_bool();

          uint8_t segway_ctrl_enable = segway_ctrl_enable_ ? 1 : 0;

          dxl_sdk_wrapper_->set_data_to_device(
            extern_control_table.segway_ctrl_enable.addr,
            extern_control_table.segway_ctrl_enable.length,
            &segway_ctrl_enable,
            &sdk_msg);

          RCLCPP_INFO(
            this->get_logger(),
            "changed parameter value : %d [segway.ctrl_enable] sdk_msg : %s",
            segway_ctrl_enable_ ? 1 : 0,
            sdk_msg.c_str());
        }

        if (changed_parameter.name == "segway.k_ext") {
          std::string sdk_msg;

          auto segway_k_ext_param =
            rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double_array();

          if (segway_k_ext_param.size() < 4) {
            RCLCPP_WARN(
              this->get_logger(),
              "Parameter segway.k_ext has %zu values, expected 4. Missing values are set to 0.0.",
              segway_k_ext_param.size());
          } else if (segway_k_ext_param.size() > 4) {
            RCLCPP_WARN(
              this->get_logger(),
              "Parameter segway.k_ext has %zu values, expected 4. Extra values are ignored.",
              segway_k_ext_param.size());
          }

          for (size_t i = 0; i < 4; ++i) {
            segway_k_ext_[i] =
              static_cast<float>(i < segway_k_ext_param.size() ? segway_k_ext_param[i] : 0.0);
          }

          union Data {
            float values[4];
            uint8_t bytes[4 * 4];
          } data;

          data.values[0] = segway_k_ext_[0];
          data.values[1] = segway_k_ext_[1];
          data.values[2] = segway_k_ext_[2];
          data.values[3] = segway_k_ext_[3];

          uint16_t start_addr = extern_control_table.segway_k_ext_1.addr;
          uint16_t addr_length =
            (extern_control_table.segway_k_ext_4.addr -
            extern_control_table.segway_k_ext_1.addr) +
            extern_control_table.segway_k_ext_4.length;

          dxl_sdk_wrapper_->set_data_to_device(
            start_addr,
            addr_length,
            &data.bytes[0],
            &sdk_msg);

          RCLCPP_INFO(
            this->get_logger(),
            "changed parameter value : [%f, %f, %f, %f] [segway.k_ext] sdk_msg : %s",
            segway_k_ext_[0],
            segway_k_ext_[1],
            segway_k_ext_[2],
            segway_k_ext_[3],
            sdk_msg.c_str());
        }
      }
    };

  parameter_event_sub_ = priv_parameters_client_->on_parameter_event(param_event_callback);
}

void TurtleBot3::cmd_vel_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = std::make_unique<TwistSubscriber>(
    node_handle_,
    "cmd_vel",
    qos,
    std::function<void(const geometry_msgs::msg::Twist::SharedPtr)>(
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        std::string sdk_msg;

        union Data {
          int32_t dword[6];
          uint8_t byte[4 * 6];
        } data;

        data.dword[0] = static_cast<int32_t>(msg->linear.x * 100);
        data.dword[1] = 0;
        data.dword[2] = 0;
        data.dword[3] = 0;
        data.dword[4] = 0;
        data.dword[5] = static_cast<int32_t>(msg->angular.z * 100);

        uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
        uint16_t addr_length =
        (extern_control_table.cmd_velocity_angular_z.addr -
        extern_control_table.cmd_velocity_linear_x.addr) +
        extern_control_table.cmd_velocity_angular_z.length;

        uint8_t * p_data = &data.byte[0];

        dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

        RCLCPP_DEBUG(
          this->get_logger(),
          "lin_vel: %f ang_vel: %f msg : %s", msg->linear.x, msg->angular.z, sdk_msg.c_str());
      }
    ),
    std::function<void(const geometry_msgs::msg::TwistStamped::SharedPtr)>(
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) -> void
      {
        std::string sdk_msg;

        union Data {
          int32_t dword[6];
          uint8_t byte[4 * 6];
        } data;

        data.dword[0] = static_cast<int32_t>(msg->twist.linear.x * 100);
        data.dword[1] = 0;
        data.dword[2] = 0;
        data.dword[3] = 0;
        data.dword[4] = 0;
        data.dword[5] = static_cast<int32_t>(msg->twist.angular.z * 100);

        uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
        uint16_t addr_length =
        (extern_control_table.cmd_velocity_angular_z.addr -
        extern_control_table.cmd_velocity_linear_x.addr) +
        extern_control_table.cmd_velocity_angular_z.length;

        uint8_t * p_data = &data.byte[0];

        dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

        RCLCPP_DEBUG(
          this->get_logger(),
          "lin_vel: %f ang_vel: %f msg : %s",
          msg->twist.linear.x,
          msg->twist.angular.z,
          sdk_msg.c_str());
      }
    )
  );
}

void TurtleBot3::cmd_pwm_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_pwm_sub_ = this->create_subscription<turtlebot3_msgs::msg::WheelPwm>(
    "cmd_pwm",
    qos,
    [this](const turtlebot3_msgs::msg::WheelPwm::SharedPtr msg) -> void
    {
      std::string sdk_msg;

      union Data {
        int16_t word[2];
        uint8_t byte[2 * 2];
      } data;

      data.word[0] = std::clamp<int16_t>(msg->left_pwm, -885, 885);
      data.word[1] = std::clamp<int16_t>(msg->right_pwm, -885, 885);

      uint16_t start_addr = extern_control_table.cmd_pwm_left.addr;
      uint16_t addr_length =
      (extern_control_table.cmd_pwm_right.addr -
      extern_control_table.cmd_pwm_left.addr) +
      extern_control_table.cmd_pwm_right.length;

      uint8_t * p_data = &data.byte[0];

      dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

      RCLCPP_DEBUG(
        this->get_logger(),
        "pwm_left: %d pwm_right: %d msg : %s",
        msg->left_pwm,
        msg->right_pwm,
        sdk_msg.c_str());
    }
  );
}