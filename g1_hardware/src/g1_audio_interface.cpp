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

#include "g1_hardware/g1_audio_interface.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

namespace g1_hardware
{
namespace
{
constexpr size_t kBytesPerSecond = 16000 * 2;
constexpr size_t kChunkBytes = kBytesPerSecond / 10;
constexpr size_t kMaxBufferBytes = kBytesPerSecond * 10;
} // namespace

class SdkAudioClient : public unitree::robot::g1::AudioClient
{
public:
  int32_t Stop(const std::string& app)
  {
    // SDK PlayStop discards the return code.
    unitree::robot::g1::PlayStopParameter parameter;
    parameter.app_name = app;
    std::string response;
    return Call(unitree::robot::g1::ROBOT_API_ID_AUDIO_STOP_PLAY,
                unitree::common::ToJsonString(parameter), response);
  }
};

G1AudioInterfaceNode::G1AudioInterfaceNode(const rclcpp::NodeOptions& options)
: Node("g1_audio_interface", options)
{
  rcl_interfaces::msg::ParameterDescriptor fixed;
  fixed.read_only = true;
  const auto network_interface = declare_parameter<std::string>("network_interface", "", fixed);
  app_name_ = declare_parameter<std::string>("app_name", "g1_ros", fixed);
  speaker_id_ = declare_parameter<int>("speaker_id", 1, fixed);
  const auto format = declare_parameter<std::string>("format", "wave", fixed);
  const auto sample_format = declare_parameter<std::string>("sample_format", "S16LE", fixed);
  const auto sample_rate = declare_parameter<int>("sample_rate", 16000, fixed);
  const auto channels = declare_parameter<int>("channels", 1, fixed);
  const auto depth = declare_parameter<int>("depth", 16, fixed);
  if (format != "wave" || sample_format != "S16LE" || sample_rate != 16000 || channels != 1 ||
      depth != 16)
  {
    throw std::invalid_argument("G1 audio requires wave (raw PCM), S16LE, 16000 Hz, mono, 16 bits");
  }
  if (app_name_.empty() || (speaker_id_ != 0 && speaker_id_ != 1)) {
    throw std::invalid_argument("app_name must be nonempty; speaker_id must be 0 or 1");
  }
  if (network_interface.empty()) {
    throw std::invalid_argument("network_interface is required");
  }
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  client_ = std::make_unique<SdkAudioClient>();
  client_->SetTimeout(1.0F);
  client_->Init();
  // The default mutually exclusive group serializes all SDK calls and queue access.
  audio_sub_ = create_subscription<audio_common_msgs::msg::AudioData>(
      "audio", 10, [this](const audio_common_msgs::msg::AudioData& msg) { OnAudio(msg); });
  info_sub_ = create_subscription<audio_common_msgs::msg::AudioInfo>(
      "audio_info", rclcpp::QoS(1).transient_local(),
      [this](const audio_common_msgs::msg::AudioInfo& msg) { OnInfo(msg); });
  sound_sub_ = create_subscription<sound_play_msgs::msg::SoundRequest>(
      "robotsound", 10, [this](const sound_play_msgs::msg::SoundRequest& msg) { OnSound(msg); });
  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { SendAudio(); });
}

G1AudioInterfaceNode::~G1AudioInterfaceNode()
{
  try {
    StopAudio();
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Audio shutdown failed: %s", error.what());
  }
}

bool G1AudioInterfaceNode::CheckResult(int32_t result, const char* operation)
{
  if (result != 0) {
    RCLCPP_ERROR(get_logger(), "%s failed: Unitree error %d", operation, result);
    return false;
  }
  return true;
}

bool G1AudioInterfaceNode::StopAudio()
{
  pending_.clear();
  next_send_ = {};
  if (stream_started_) {
    stop_failed_ = true;
    if (!CheckResult(client_->Stop(app_name_), "PlayStop")) {
      return false;
    }
    stream_started_ = false;
    stream_id_.clear();
  }
  stop_failed_ = false;
  return true;
}

void G1AudioInterfaceNode::OnInfo(const audio_common_msgs::msg::AudioInfo& msg)
{
  format_valid_ = msg.channels == 1 && msg.sample_rate == 16000 && msg.sample_format == "S16LE" &&
                  msg.coding_format == "wave";
  if (!format_valid_) {
    RCLCPP_ERROR(get_logger(), "audio_info must describe 16000 Hz mono S16LE raw PCM (wave)");
    try {
      StopAudio();
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "PlayStop failed: %s", error.what());
    }
  }
}

void G1AudioInterfaceNode::OnAudio(const audio_common_msgs::msg::AudioData& msg)
{
  if (msg.data.empty()) {
    return;
  }
  if (stop_failed_ || !format_valid_ || msg.data.size() % 2 != 0) {
    RCLCPP_ERROR(get_logger(),
                 "Rejected audio: check format, complete S16LE samples, and previous stop errors");
    return;
  }
  if (msg.data.size() > kMaxBufferBytes - pending_.size()) {
    RCLCPP_ERROR(get_logger(), "Audio buffer full (10 seconds); pace the publisher in real time");
    return;
  }
  pending_.insert(pending_.end(), msg.data.begin(), msg.data.end());
}

void G1AudioInterfaceNode::SendAudio()
{
  const auto now = std::chrono::steady_clock::now();
  if (pending_.empty() || now < next_send_) {
    return;
  }
  if (stream_id_.empty() || now > next_send_ + std::chrono::seconds(1)) {
    stream_id_ = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
  }
  const auto count = std::min(kChunkBytes, pending_.size());
  std::vector<uint8_t> pcm(pending_.begin(), pending_.begin() + count);
  pending_.erase(pending_.begin(), pending_.begin() + count);
  try {
    stream_started_ = true;
    if (!CheckResult(client_->PlayStream(app_name_, stream_id_, pcm), "PlayStream")) {
      StopAudio();
      return;
    }
    // Pace by audio duration, independent of ROS simulated time.
    if (now - next_send_ > std::chrono::milliseconds(100)) {
      next_send_ = now;
    }
    next_send_ += std::chrono::microseconds(count * 1000000 / kBytesPerSecond);
  } catch (const std::exception& error) {
    pending_.clear();
    RCLCPP_ERROR(get_logger(), "Audio playback failed: %s", error.what());
    try {
      StopAudio();
    } catch (const std::exception& stop_error) {
      RCLCPP_ERROR(get_logger(), "PlayStop failed: %s", stop_error.what());
    }
  }
  // Do not stop at EOF: the robot may still be playing its buffered tail.
}

void G1AudioInterfaceNode::OnSound(const sound_play_msgs::msg::SoundRequest& msg)
{
  using Request = sound_play_msgs::msg::SoundRequest;
  try {
    if (msg.sound == Request::ALL && msg.command == Request::PLAY_STOP) {
      if (StopAudio()) {
        RCLCPP_WARN(get_logger(), "Stopped PCM stream; Unitree does not expose TTS cancellation");
      }
      return;
    }
    if (msg.sound != Request::SAY || msg.command != Request::PLAY_ONCE) {
      RCLCPP_ERROR(get_logger(), "Supported requests: SAY/PLAY_ONCE and ALL/PLAY_STOP (PCM only)");
      return;
    }
    if (msg.arg.empty() || !std::isfinite(msg.volume) || msg.volume < 0 || msg.volume > 1) {
      RCLCPP_ERROR(get_logger(), "SAY requires nonempty text and finite volume in [0, 1]");
      return;
    }
    int speaker = speaker_id_;
    if (!msg.arg2.empty()) {
      if (msg.arg2 != "0" && msg.arg2 != "1") {
        RCLCPP_ERROR(get_logger(), "SAY voice (arg2) must be empty, 0 (Chinese), or 1 (English)");
        return;
      }
      speaker = msg.arg2 == "0" ? 0 : 1;
    }
    if (!StopAudio() ||
        !CheckResult(client_->SetVolume(static_cast<uint8_t>(std::lround(msg.volume * 100))),
                     "SetVolume"))
    {
      return;
    }
    CheckResult(client_->TtsMaker(msg.arg, speaker), "TtsMaker");
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Sound request failed: %s", error.what());
  }
}
} // namespace g1_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(g1_hardware::G1AudioInterfaceNode)
