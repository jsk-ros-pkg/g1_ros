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

#include "inspire_hand_driver/deactivation.hpp"
#include "inspire_hand_driver/rh56dfx_hardware.hpp"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <pluginlib/class_list_macros.hpp>
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace g1_hardware
{
class InspireRH56DFXDDSHardwareInterface : public inspire_hand_driver::RH56DFXHardwareBase
{
public:
  InspireRH56DFXDDSHardwareInterface() : RH56DFXHardwareBase(12) {}

private:
  using MotorStates = unitree_go::msg::dds_::MotorStates_;
  using MotorCmds = unitree_go::msg::dds_::MotorCmds_;

  struct State
  {
    Positions positions = Positions(12);
    std::mutex mutex;
    std::condition_variable ready;
    bool received{false};
  };

  std::shared_ptr<State> state_;
  unitree::robot::ChannelPublisherPtr<MotorCmds> publisher_;
  unitree::robot::ChannelSubscriberPtr<MotorStates> subscriber_;
  MotorCmds message_;

  void connect() override
  {
    const auto& params = info_.hardware_parameters;
    const double timeout = std::stod(params.at("state_timeout_sec"));
    if (!std::isfinite(timeout) || timeout < 0) {
      throw std::invalid_argument("Invalid state_timeout_sec");
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, params.at("network_interface"));
    publisher_ =
        std::make_shared<unitree::robot::ChannelPublisher<MotorCmds>>(params.at("command_topic"));
    publisher_->InitChannel();
    state_ = std::make_shared<State>();
    message_.cmds().resize(12);
    subscriber_ =
        std::make_shared<unitree::robot::ChannelSubscriber<MotorStates>>(params.at("state_topic"));
    subscriber_->InitChannel(
        [state = state_](const void* data) {
          const auto& motors = static_cast<const MotorStates*>(data)->states();
          if (motors.size() < 12) {
            return;
          }
          std::lock_guard lock(state->mutex);
          for (size_t i = 0; i < state->positions.size(); ++i) {
            state->positions[i] = motors[i].q();
          }
          if (!state->received) {
            state->received = true;
            state->ready.notify_one();
          }
        },
        1);
    std::unique_lock lock(state_->mutex);
    if (!state_->ready.wait_for(lock, std::chrono::duration<double>(timeout),
                                [this] { return state_->received; }))
    {
      throw std::runtime_error("No 12-motor state on " + params.at("state_topic") +
                               ". Is inspire_g1 running?");
    }
  }

  void read_positions(Positions& state) override
  {
    if (!state_) {
      return;
    }
    std::lock_guard lock(state_->mutex);
    state = state_->positions;
  }

  bool PublishTarget(const Positions& target)
  {
    for (size_t i = 0; i < target.size(); ++i) {
      message_.cmds()[i].q(static_cast<float>(target[i]));
    }
    return publisher_->Write(message_);
  }

  void command(const Positions& target) override { PublishTarget(target); }

  void disconnect(const Positions* final_target) override
  {
    if (publisher_ && final_target) {
      if (PublishTarget(Positions(final_target->size(), 1.0))) {
        std::this_thread::sleep_for(inspire_hand_driver::deactivation::open_dwell);
        // Give the bridge time to receive the final command.
        for (int i = 0; i < 5; ++i) {
          command(*final_target);
          if (i < 4) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
        }
      } else {
        RCLCPP_WARN(get_logger(), "Failed to publish opening command; skipping shutdown close.");
      }
    }
    subscriber_.reset();
    publisher_.reset();
    state_.reset();
  }
};
} // namespace g1_hardware

PLUGINLIB_EXPORT_CLASS(g1_hardware::InspireRH56DFXDDSHardwareInterface,
                       hardware_interface::SystemInterface)
