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

#include "g1_hardware/g1_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <urdf/model.h>

namespace g1_hardware
{

class G1MotorTemperatures : public rclcpp::Node
{
public:
  explicit G1MotorTemperatures(const rclcpp::NodeOptions& options)
  : Node("motor_temperatures", options)
  {
    const auto network_interface = declare_parameter<std::string>("network_interface");
    const int domain_id = declare_parameter("domain_id", 0);
    const double publish_rate = declare_parameter("publish_rate", 1.0);
    state_timeout_ = declare_parameter("state_timeout", 2.0);
    if (!std::isfinite(publish_rate) || publish_rate <= 0.0 || !std::isfinite(state_timeout_) ||
        state_timeout_ <= 0.0)
    {
      throw std::invalid_argument("publish_rate and state_timeout must be finite and positive");
    }

    urdf::Model model;
    if (!model.initString(declare_parameter<std::string>("robot_description", ""))) {
      throw std::invalid_argument("A valid robot_description is required for motor TF frames");
    }
    // Unitree's g1/common/terminations.hpp: [0] casing, [1] winding.
    const std::array<std::string, 2> channels{"casing", "winding"};
    for (size_t i = 0; i < channels.size(); ++i) {
      // Shared streams allow RViz to display all motor positions with one display.
      all_motors_pub_[i] = create_publisher<Temperature>("~/" + channels[i], 100);
    }
    for (const auto& [name, index] : kJointNameToSdkIndex) {
      const auto joint = model.getJoint(name);
      if (!joint) {
        continue;
      }
      Motor motor;
      motor.index = index;
      motor.frame = joint->child_link_name;
      for (size_t i = 0; i < channels.size(); ++i) {
        motor.publishers[i] = create_publisher<Temperature>("~/" + name + "/" + channels[i], 1);
      }
      motors_.push_back(std::move(motor));
    }
    if (motors_.empty()) {
      throw std::invalid_argument("robot_description contains no supported G1 motor joints");
    }

    unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
    low_state_subscriber_ =
        std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
            "rt/lowstate");
    low_state_subscriber_->InitChannel(
        [this](const void* data) {
          const auto& state = *static_cast<const unitree_hg::msg::dds_::LowState_*>(data);
          std::lock_guard<std::mutex> lock(state_mutex_);
          for (const auto& motor : motors_) {
            temperatures_[motor.index] = state.motor_state().at(motor.index).temperature();
          }
          // LowState has no ROS timestamp; use host reception time.
          stamp_ = now();
          received_at_ = std::chrono::steady_clock::now();
          new_sample_ = true;
        },
        1);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate),
                               [this]() { PublishTemperatures(); });
  }

  ~G1MotorTemperatures() override
  {
    timer_->cancel();
    low_state_subscriber_.reset();
  }

private:
  using Temperature = sensor_msgs::msg::Temperature;
  using Temperatures = std::array<std::array<int16_t, 2>, 29>;

  struct Motor
  {
    int index;
    std::string frame;
    std::array<rclcpp::Publisher<Temperature>::SharedPtr, 2> publishers;
  };

  void PublishTemperatures()
  {
    Temperatures temperatures;
    builtin_interfaces::msg::Time stamp;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!new_sample_) {
        return;
      }
      new_sample_ = false;
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - received_at_).count() >
          state_timeout_)
      {
        return;
      }
      temperatures = temperatures_;
      stamp = stamp_;
    }
    for (const auto& motor : motors_) {
      Temperature message;
      message.header.stamp = stamp;
      message.header.frame_id = motor.frame;
      message.variance = 0.0; // Unknown; the SDK provides no variance.
      for (size_t i = 0; i < motor.publishers.size(); ++i) {
        message.temperature = temperatures[motor.index][i];
        motor.publishers[i]->publish(message);
        all_motors_pub_[i]->publish(message);
      }
    }
  }

  std::vector<Motor> motors_;
  std::array<rclcpp::Publisher<Temperature>::SharedPtr, 2> all_motors_pub_;
  std::mutex state_mutex_;
  Temperatures temperatures_{};
  builtin_interfaces::msg::Time stamp_;
  std::chrono::steady_clock::time_point received_at_;
  bool new_sample_{false};
  double state_timeout_{2.0};
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> low_state_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace g1_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(g1_hardware::G1MotorTemperatures)
