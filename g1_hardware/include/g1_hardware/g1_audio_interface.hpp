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

#ifndef G1_HARDWARE__G1_AUDIO_INTERFACE_HPP_
#define G1_HARDWARE__G1_AUDIO_INTERFACE_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <audio_common_msgs/msg/audio_data.hpp>
#include <audio_common_msgs/msg/audio_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sound_play_msgs/action/sound_request.hpp>
#include <sound_play_msgs/msg/sound_request.hpp>

namespace g1_hardware
{
class SdkAudioClient;
struct AudioPlayback;

class G1AudioInterfaceNode : public rclcpp::Node
{
public:
  explicit G1AudioInterfaceNode(const rclcpp::NodeOptions& options);
  ~G1AudioInterfaceNode() override;

private:
  using Request = sound_play_msgs::msg::SoundRequest;
  using Action = sound_play_msgs::action::SoundRequest;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using Clock = std::chrono::steady_clock;

  void OnAudio(const audio_common_msgs::msg::AudioData& msg);
  void OnInfo(const audio_common_msgs::msg::AudioInfo& msg);
  void OnSound(const Request& msg);
  void ApplySound(const Request& msg);
  void ValidateRequest(const Request& msg) const;
  std::string SoundKey(const Request& msg) const;
  void SendAudio();
  void StopStream();
  void StopAll();
  void FinishGoal(rclcpp_action::ResultCode code);
  void AcceptGoal(const std::shared_ptr<GoalHandle>& goal);

  std::unique_ptr<SdkAudioClient> client_;
  std::unique_ptr<AudioPlayback> audio_;
  std::map<std::string, std::unique_ptr<AudioPlayback>> sounds_;
  std::vector<uint8_t> audio_partial_;
  std::string app_name_;
  std::string stream_id_;
  std::string format_;
  std::string sample_format_;
  int sample_rate_;
  int channels_;
  size_t frame_bytes_;
  bool format_valid_{true};
  bool stream_started_{false};
  bool stop_failed_{false};
  Clock::time_point next_send_{};
  std::shared_ptr<GoalHandle> goal_;
  std::string goal_key_;
  Clock::time_point goal_start_{};
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr audio_sub_;
  rclcpp::Subscription<audio_common_msgs::msg::AudioInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<Request>::SharedPtr sound_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace g1_hardware

#endif // G1_HARDWARE__G1_AUDIO_INTERFACE_HPP_
