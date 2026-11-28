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

#include "inspire_hand_driver/rh56dfx_hardware.hpp"

#include "rh56dfx_hand.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <exception>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <set>
#include <stdexcept>

namespace inspire_hand_driver
{
namespace
{

int integer_parameter(const hardware_interface::HardwareInfo& info, const std::string& name,
                      int fallback)
{
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end())
    return fallback;
  const auto& value = it->second;
  const char* end = value.data() + value.size();
  int result;
  const auto parsed = std::from_chars(value.data(), end, result);
  if (parsed.ec != std::errc{} || parsed.ptr != end)
    throw std::invalid_argument("Invalid " + name + ": " + value);
  return result;
}

using JointConfig = RH56DFXHardwareBase::JointConfig;

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
      RCLCPP_FATAL(get_logger(), "Invalid close_hand_on_deactivate parameter '%s': %s",
                   parameter_value.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  }

  const size_t n = info_.joints.size();
  if (n != motor_count_) {
    RCLCPP_ERROR(get_logger(), "Expected %zu actuated joints, got %zu", motor_count_, n);
    return CallbackReturn::ERROR;
  }
  motor_positions_.resize(n);
  motor_targets_.resize(n);
  std::set<size_t> indices;
  joint_configs_.resize(n);
  hw_positions_.resize(n, 0.0);
  hw_velocities_.resize(n, 0.0);
  hw_commands_.resize(n, 0.0);

  for (size_t i = 0; i < n; ++i) {
    const std::string& name = info_.joints[i].name;
    const auto& params = info_.joints[i].parameters;
    try {
      auto& cfg = joint_configs_[i];
      cfg.motor_index = std::stoul(params.at("motor_index"));
      cfg.open_position = 0.0;
      cfg.closed_position = std::stod(params.at("closed_position"));
      cfg.deactivation_position = std::stod(params.at("deactivation_position"));
      if (cfg.motor_index >= n || !indices.insert(cfg.motor_index).second ||
          !std::isfinite(cfg.closed_position) || cfg.closed_position <= 0 ||
          !std::isfinite(cfg.deactivation_position) || cfg.deactivation_position < 0 ||
          cfg.deactivation_position > cfg.closed_position)
        throw std::invalid_argument("invalid motor index or joint range");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Invalid RH56DFX joint '%s': %s", name.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
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
  RCLCPP_INFO(get_logger(), "Activating RH56DFX hand interface.");

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
  RCLCPP_INFO(get_logger(), "Activated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
RH56DFXHardwareBase::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  auto& targets = motor_targets_;
  for (const auto& cfg : joint_configs_) {
    targets[cfg.motor_index] = JointPositionToOpenFraction(cfg.deactivation_position, cfg);
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

  RCLCPP_INFO(get_logger(), "Command mode switch: %d started, %d stopped -> %d active.", started,
              stopped, active_command_interfaces_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RH56DFXHardwareBase::read(const rclcpp::Time& /*time*/,
                                                          const rclcpp::Duration& period)
{
  read_positions(motor_positions_);
  const double dt = period.seconds();
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const double previous_position = hw_positions_[i];
    const auto& cfg = joint_configs_[i];
    const double open_fraction = motor_positions_[cfg.motor_index];
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
  auto& targets = motor_targets_;
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& cfg = joint_configs_[i];
    targets[cfg.motor_index] = JointPositionToOpenFraction(hw_commands_[i], cfg);
  }
  command(targets);
  return hardware_interface::return_type::OK;
}

class RH56DFXHardware : public RH56DFXHardwareBase
{
public:
  RH56DFXHardware() : RH56DFXHardwareBase(6) {}

private:
  std::shared_ptr<RH56DFXHand> hand_;

  void connect() override
  {
    const auto& params = info_.hardware_parameters;
    const bool exit_on_disconnect = params.count("exit_on_disconnect") &&
                                    hardware_interface::parse_bool(params.at("exit_on_disconnect"));
    if (params.at("device") == "none" && !exit_on_disconnect)
      return;
    hand_ = std::make_shared<RH56DFXHand>(
        params.at("device"), integer_parameter(info_, "baudrate", 115200),
        integer_parameter(info_, "hand_id", 1), exit_on_disconnect);
    auto prefix = params.find("action_prefix");
    std::string action_prefix = prefix == params.end() ? "" : prefix->second;
    if (!action_prefix.empty() && action_prefix.back() != '/')
      action_prefix += '/';
    const auto frames = params.find("frame_prefix");
    hand_->start(get_node(), action_prefix, frames == params.end() ? "" : frames->second);
  }

  void read_positions(Positions& state) override
  {
    if (hand_) {
      const auto sample = hand_->positions();
      std::copy(sample.begin(), sample.end(), state.begin());
    } else {
      std::fill(state.begin(), state.end(), 1.0);
    }
  }

  void command(const Positions& target) override
  {
    if (!hand_)
      return;
    RH56DFXHand::Positions positions;
    std::copy(target.begin(), target.end(), positions.begin());
    hand_->command(positions);
  }

  void disconnect(const Positions* final_target) override
  {
    auto hand = std::move(hand_);
    if (!hand)
      return;
    RH56DFXHand::Positions positions;
    if (final_target)
      std::copy(final_target->begin(), final_target->end(), positions.begin());
    hand->stop(final_target ? &positions : nullptr);
  }
};

} // namespace inspire_hand_driver

PLUGINLIB_EXPORT_CLASS(inspire_hand_driver::RH56DFXHardware, hardware_interface::SystemInterface)
