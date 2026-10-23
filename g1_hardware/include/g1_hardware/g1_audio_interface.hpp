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
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <audio_common_msgs/msg/audio_data.hpp>
#include <audio_common_msgs/msg/audio_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sound_play_msgs/msg/sound_request.hpp>

namespace g1_hardware
{
class AudioBackend
{
public:
  virtual ~AudioBackend() = default;
  virtual int32_t Play(const std::string& app, const std::string& stream,
                       const std::vector<uint8_t>& pcm) = 0;
  virtual int32_t Stop(const std::string& app) = 0;
  virtual int32_t Say(const std::string& text, int32_t speaker) = 0;
  virtual int32_t SetVolume(uint8_t volume) = 0;
};

class G1AudioInterfaceNode : public rclcpp::Node
{
public:
  explicit G1AudioInterfaceNode(const rclcpp::NodeOptions& options);
  G1AudioInterfaceNode(const rclcpp::NodeOptions& options, std::unique_ptr<AudioBackend> backend);
  ~G1AudioInterfaceNode() override;

private:
  void OnAudio(const audio_common_msgs::msg::AudioData& msg);
  void OnInfo(const audio_common_msgs::msg::AudioInfo& msg);
  void OnSound(const sound_play_msgs::msg::SoundRequest& msg);
  void SendAudio();
  bool StopAudio();
  bool CheckResult(int32_t result, const char* operation);

  std::unique_ptr<AudioBackend> backend_;
  std::string app_name_;
  std::string stream_id_;
  int speaker_id_;
  bool format_valid_{true};
  bool stream_started_{false};
  bool stop_failed_{false};
  std::deque<uint8_t> pending_;
  std::chrono::steady_clock::time_point next_send_{};
  rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr audio_sub_;
  rclcpp::Subscription<audio_common_msgs::msg::AudioInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sound_play_msgs::msg::SoundRequest>::SharedPtr sound_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace g1_hardware

#endif // G1_HARDWARE__G1_AUDIO_INTERFACE_HPP_
