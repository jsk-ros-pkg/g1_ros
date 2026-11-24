// Copyright 2026 Yoshiki Obinata
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
//    * Neither the name of the Yoshiki Obinata nor the names of its
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

#include "g1_hardware/inspire_rh56dfx_hardware_interface.hpp"

#include "rh56dfx_hand.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <unordered_map>

namespace g1_hardware
{
namespace
{

constexpr size_t kFingers = 6;
constexpr std::array<const char*, 2> kSide{"right", "left"};

using JointConfig = RH56DFXHardwareBase::JointConfig;

// The table stores ROS joint positions for open, fully closed, and deactivation states.
// The deactivation pose matches inspire_dfx_hand_open_close_demo.py, including its
// collision-free thumb yaw position.
const std::unordered_map<std::string, JointConfig> kJointNameToConfig = {
    {"R_pinky_proximal_joint", {0, 0.0, 1.7, 1.7}},
    {"R_ring_proximal_joint", {1, 0.0, 1.7, 1.7}},
    {"R_middle_proximal_joint", {2, 0.0, 1.7, 1.7}},
    {"R_index_proximal_joint", {3, 0.0, 1.7, 1.7}},
    {"R_thumb_proximal_pitch_joint", {4, 0.0, 0.6, 0.6}},
    {"R_thumb_proximal_yaw_joint", {5, 0.0, 1.3, 0.3}},
    {"L_pinky_proximal_joint", {6, 0.0, 1.7, 1.7}},
    {"L_ring_proximal_joint", {7, 0.0, 1.7, 1.7}},
    {"L_middle_proximal_joint", {8, 0.0, 1.7, 1.7}},
    {"L_index_proximal_joint", {9, 0.0, 1.7, 1.7}},
    {"L_thumb_proximal_pitch_joint", {10, 0.0, 0.6, 0.6}},
    {"L_thumb_proximal_yaw_joint", {11, 0.0, 1.3, 0.3}},
};

double Clamp01(double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::min(1.0, std::max(0.0, value));
}

double OpenFractionToJointPosition(double open_fraction, const JointConfig& cfg)
{
  const double normalized_closed = 1.0 - Clamp01(open_fraction);
  return cfg.open_position + normalized_closed * (cfg.closed_position - cfg.open_position);
}

double JointPositionToOpenFraction(double position, const JointConfig& cfg)
{
  const double normalized_closed =
      (position - cfg.open_position) / (cfg.closed_position - cfg.open_position);
  return 1.0 - Clamp01(normalized_closed);
}

} // namespace

hardware_interface::CallbackReturn
RH56DFXHardwareBase::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (info.hardware_parameters.count("close_hand_on_deactivate")) {
    const auto& parameter_value = info.hardware_parameters.at("close_hand_on_deactivate");
    try {
      close_hand_on_deactivate_ = hardware_interface::parse_bool(parameter_value);
    } catch (const std::invalid_argument& e) {
      RCLCPP_FATAL(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
                   "Invalid close_hand_on_deactivate parameter '%s': %s", parameter_value.c_str(),
                   e.what());
      return CallbackReturn::ERROR;
    }
  }

  const size_t n = info_.joints.size();
  joint_configs_.resize(n);
  hw_positions_.resize(n, 0.0);
  hw_velocities_.resize(n, 0.0);
  hw_commands_.resize(n, 0.0);

  for (size_t i = 0; i < n; ++i) {
    const std::string& name = info_.joints[i].name;
    auto it = kJointNameToConfig.find(name);
    if (it == kJointNameToConfig.end()) {
      RCLCPP_FATAL(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
                   "Unknown RH56DFX joint '%s' - no motor mapping defined.", name.c_str());
      return CallbackReturn::ERROR;
    }
    joint_configs_[i] = it->second;
    hw_positions_[i] = it->second.open_position;
    hw_commands_[i] = hw_positions_[i];

    RCLCPP_INFO(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
                "Joint '%s' -> Inspire motor %zu, open %.3f, closed %.3f", name.c_str(),
                it->second.motor_index, it->second.open_position, it->second.closed_position);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RH56DFXHardwareBase::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface> RH56DFXHardwareBase::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return ci;
}

hardware_interface::CallbackReturn
RH56DFXHardwareBase::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
              "Activating RH56DFX hand interface.");

  try {
    connect();
  } catch (const std::exception& e) {
    disconnect(nullptr);
    RCLCPP_ERROR(get_logger(), "Could not activate RH56DFX hands: %s", e.what());
    return CallbackReturn::ERROR;
  }
  read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0));
  std::copy(hw_positions_.begin(), hw_positions_.end(), hw_commands_.begin());

  active_command_interfaces_ = 0;
  RCLCPP_INFO(rclcpp::get_logger("InspireRH56DFXHardwareInterface"), "Activated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
RH56DFXHardwareBase::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  Positions targets{};
  for (const auto& cfg : joint_configs_) {
    targets[cfg.motor_index / kFingers][cfg.motor_index % kFingers] =
        JointPositionToOpenFraction(cfg.deactivation_position, cfg);
  }
  active_command_interfaces_ = 0;
  try {
    disconnect(close_hand_on_deactivate_ ? &targets : nullptr);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to close RH56DFX hands: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
RH56DFXHardwareBase::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                 const std::vector<std::string>& stop_interfaces)
{
  auto is_own_command_interface = [this](const std::string& interface_name) {
    const auto separator = interface_name.find('/');
    const std::string joint_name =
        separator == std::string::npos ? interface_name : interface_name.substr(0, separator);
    return std::any_of(info_.joints.begin(), info_.joints.end(), [&joint_name](const auto& joint) {
      return joint.name == joint_name && !joint.command_interfaces.empty();
    });
  };

  const auto started = static_cast<int>(
      std::count_if(start_interfaces.begin(), start_interfaces.end(), is_own_command_interface));
  const auto stopped = static_cast<int>(
      std::count_if(stop_interfaces.begin(), stop_interfaces.end(), is_own_command_interface));

  active_command_interfaces_ += started;
  active_command_interfaces_ -= stopped;
  active_command_interfaces_ = std::max(0, active_command_interfaces_);

  RCLCPP_INFO(rclcpp::get_logger("InspireRH56DFXHardwareInterface"),
              "Command mode switch: %d started, %d stopped -> %d active.", started, stopped,
              active_command_interfaces_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RH56DFXHardwareBase::read(const rclcpp::Time& /*time*/,
                                                          const rclcpp::Duration& period)
{
  const auto state = positions();
  const double dt = period.seconds();
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const double previous_position = hw_positions_[i];
    const auto& cfg = joint_configs_[i];
    const double open_fraction = state[cfg.motor_index / kFingers][cfg.motor_index % kFingers];
    hw_positions_[i] = OpenFractionToJointPosition(open_fraction, cfg);
    hw_velocities_[i] = dt > 1e-9 ? (hw_positions_[i] - previous_position) / dt : 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RH56DFXHardwareBase::write(const rclcpp::Time& /*time*/,
                                                           const rclcpp::Duration& /*period*/)
{
  if (active_command_interfaces_ == 0)
    return hardware_interface::return_type::OK;
  Positions targets{};
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& cfg = joint_configs_[i];
    targets[cfg.motor_index / kFingers][cfg.motor_index % kFingers] =
        JointPositionToOpenFraction(hw_commands_[i], cfg);
  }
  command(targets);
  return hardware_interface::return_type::OK;
}

void InspireRH56DFXHardwareInterface::connect()
{
  for (size_t i = 0; i < hands_.size(); ++i) {
    const auto param = info_.hardware_parameters.find(std::string(kSide[i]) + "_device");
    const std::string device = param == info_.hardware_parameters.end()
                                   ? std::string("/dev/inspire_") + kSide[i]
                                   : param->second;
    if (device == "none")
      continue;
    hands_[i] = std::make_shared<RH56DFXHand>(device);
    hands_[i]->start(get_node(), kSide[i]);
  }
}

RH56DFXHardwareBase::Positions InspireRH56DFXHardwareInterface::positions()
{
  Positions state;
  for (size_t i = 0; i < hands_.size(); ++i) {
    if (hands_[i])
      state[i] = hands_[i]->positions();
    else
      state[i].fill(1.0);
  }
  return state;
}

void InspireRH56DFXHardwareInterface::command(const Positions& targets)
{
  for (size_t i = 0; i < hands_.size(); ++i) {
    if (hands_[i])
      hands_[i]->command(targets[i]);
  }
}

void InspireRH56DFXHardwareInterface::disconnect(const Positions* final_target)
{
  std::exception_ptr failure;
  for (size_t i = 0; i < hands_.size(); ++i) {
    if (!hands_[i])
      continue;
    try {
      hands_[i]->stop(final_target ? &(*final_target)[i] : nullptr);
    } catch (...) {
      failure = std::current_exception();
    }
    hands_[i].reset();
  }
  if (failure)
    std::rethrow_exception(failure);
}

} // namespace g1_hardware

PLUGINLIB_EXPORT_CLASS(g1_hardware::InspireRH56DFXHardwareInterface,
                       hardware_interface::SystemInterface)
