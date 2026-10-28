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
#include <deque>
#include <filesystem>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/audio/audio.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

namespace g1_hardware
{
namespace
{
constexpr size_t kSamplesPerChunk = 1600;
constexpr size_t kMaxInputBytes = 4 * 1024 * 1024;
const char* const kBuiltinFiles[] = {"BACKINGUP.ogg", "NEEDS_UNPLUGGING.ogg", "NEEDS_PLUGGING.ogg",
                                     "NEEDS_UNPLUGGING_BADLY.ogg", "NEEDS_PLUGGING_BADLY.ogg"};

void CheckSdk(int32_t result, const char* operation)
{
  if (result != 0) {
    throw std::runtime_error(std::string(operation) + " failed: Unitree error " +
                             std::to_string(result));
  }
}

size_t FrameBytes(const std::string& format, const std::string& sample_format, int rate,
                  int channels)
{
  const auto* info = gst_audio_format_get_info(gst_audio_format_from_string(sample_format.c_str()));
  if ((format != "wave" && format != "mp3") || rate <= 0 || channels <= 0 || channels > 64 ||
      !info || GST_AUDIO_FORMAT_INFO_WIDTH(info) == 0)
  {
    throw std::invalid_argument(
        "Expected format wave/mp3, a valid PCM sample_format, rate and channels");
  }
  return static_cast<size_t>(GST_AUDIO_FORMAT_INFO_WIDTH(info) / 8) * channels;
}
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

struct AudioPlayback
{
  GstElement* pipeline{nullptr};
  GstElement* source{nullptr};
  GstElement* sink{nullptr};
  std::deque<uint8_t> pcm;
  double volume{1.0};
  bool loop{false};
  bool received_samples{false};
  std::chrono::steady_clock::time_point playing_until{};

  ~AudioPlayback()
  {
    if (pipeline) {
      gst_element_set_state(pipeline, GST_STATE_NULL);
    }
    if (source) {
      gst_object_unref(source);
    }
    if (sink) {
      gst_object_unref(sink);
    }
    if (pipeline) {
      gst_object_unref(pipeline);
    }
  }

  void PullSamples()
  {
    GstBus* bus = gst_element_get_bus(pipeline);
    GstMessage* error_message = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
    gst_object_unref(bus);
    if (error_message) {
      GError* error = nullptr;
      gst_message_parse_error(error_message, &error, nullptr);
      const std::string message = error->message;
      g_error_free(error);
      gst_message_unref(error_message);
      throw std::runtime_error(message);
    }
    while (pcm.size() < kSamplesPerChunk * 2) {
      GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 0);
      if (!sample) {
        break;
      }
      GstBuffer* buffer = gst_sample_get_buffer(sample);
      GstMapInfo map{};
      if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        throw std::runtime_error("Cannot read decoded audio");
      }
      pcm.insert(pcm.end(), map.data, map.data + map.size);
      received_samples = received_samples || map.size != 0;
      gst_buffer_unmap(buffer, &map);
      gst_sample_unref(sample);
    }
  }
};

namespace
{
std::unique_ptr<AudioPlayback> CreatePlayback(const std::string& uri, const std::string& format,
                                              const std::string& sample_format, int rate,
                                              int channels)
{
  auto playback = std::make_unique<AudioPlayback>();
  std::string pipeline = uri.empty() ? "appsrc name=input " : "uridecodebin name=input ";
  if (uri.empty() && format == "mp3") {
    pipeline += "! decodebin ";
  }
  pipeline += "! audioconvert ! audioresample ! "
              "audio/x-raw,format=S16LE,rate=16000,channels=1,layout=interleaved "
              "! appsink name=output sync=false max-buffers=4";
  GError* error = nullptr;
  playback->pipeline = gst_parse_launch(pipeline.c_str(), &error);
  if (error) {
    const std::string message = error->message;
    g_error_free(error);
    throw std::runtime_error(message);
  }
  playback->source = gst_bin_get_by_name(GST_BIN(playback->pipeline), "input");
  playback->sink = gst_bin_get_by_name(GST_BIN(playback->pipeline), "output");
  if (!uri.empty()) {
    g_object_set(playback->source, "uri", uri.c_str(), nullptr);
  } else {
    g_object_set(playback->source, "format", GST_FORMAT_TIME, "max-bytes",
                 static_cast<guint64>(kMaxInputBytes), nullptr);
    if (format == "wave") {
      GstCaps* caps = gst_caps_new_simple(
          "audio/x-raw", "format", G_TYPE_STRING, sample_format.c_str(), "rate", G_TYPE_INT, rate,
          "channels", G_TYPE_INT, channels, "layout", G_TYPE_STRING, "interleaved", nullptr);
      gst_app_src_set_caps(GST_APP_SRC(playback->source), caps);
      gst_caps_unref(caps);
    }
  }
  if (gst_element_set_state(playback->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    throw std::runtime_error("Cannot start audio decoder");
  }
  return playback;
}
} // namespace

G1AudioInterfaceNode::G1AudioInterfaceNode(const rclcpp::NodeOptions& options)
: Node("g1_audio_interface", options)
{
  gst_init(nullptr, nullptr);
  rcl_interfaces::msg::ParameterDescriptor fixed;
  fixed.read_only = true;
  const auto network_interface = declare_parameter<std::string>("network_interface", "", fixed);
  app_name_ = declare_parameter<std::string>("app_name", "g1_ros", fixed);
  format_ = declare_parameter<std::string>("format", "mp3", fixed);
  sample_format_ = declare_parameter<std::string>("sample_format", "S16LE", fixed);
  sample_rate_ = declare_parameter<int>("sample_rate", 16000, fixed);
  channels_ = declare_parameter<int>("channels", 1, fixed);
  const auto depth = declare_parameter<int>("depth", 16, fixed);
  frame_bytes_ = FrameBytes(format_, sample_format_, sample_rate_, channels_);
  if (depth != GST_AUDIO_FORMAT_INFO_DEPTH(
                   gst_audio_format_get_info(gst_audio_format_from_string(sample_format_.c_str()))))
  {
    throw std::invalid_argument("depth must match sample_format");
  }
  const auto voice = declare_parameter<std::string>("default_voice", "voice_kal_diphone", fixed);
  if (voice != "voice_kal_diphone") {
    throw std::invalid_argument("Only the default English voice is supported by Unitree TTS");
  }
  if (network_interface.empty() || app_name_.empty()) {
    throw std::invalid_argument("network_interface and app_name must be nonempty");
  }
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  client_ = std::make_unique<SdkAudioClient>();
  client_->SetTimeout(1.0F);
  client_->Init();
  audio_sub_ = create_subscription<audio_common_msgs::msg::AudioData>(
      "audio", 10, [this](const audio_common_msgs::msg::AudioData& msg) { OnAudio(msg); });
  info_sub_ = create_subscription<audio_common_msgs::msg::AudioInfo>(
      "audio_info", rclcpp::QoS(1).transient_local(),
      [this](const audio_common_msgs::msg::AudioInfo& msg) { OnInfo(msg); });
  sound_sub_ =
      create_subscription<Request>("robotsound", 10, [this](const Request& msg) { OnSound(msg); });
  action_server_ = rclcpp_action::create_server<Action>(
      this, "sound_play",
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal> goal) {
        try {
          ValidateRequest(goal->sound_request);
          if (goal->sound_request.sound == Request::SAY) {
            throw std::invalid_argument("Unitree TTS has no completion/cancel API; use robotsound");
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } catch (const std::exception& error) {
          RCLCPP_ERROR(get_logger(), "%s", error.what());
          return rclcpp_action::GoalResponse::REJECT;
        }
      },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const std::shared_ptr<GoalHandle> goal) { AcceptGoal(goal); });
  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { SendAudio(); });
}

G1AudioInterfaceNode::~G1AudioInterfaceNode()
{
  try {
    StopAll();
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Audio shutdown failed: %s", error.what());
  }
}

void G1AudioInterfaceNode::StopStream()
{
  next_send_ = {};
  if (stream_started_) {
    stop_failed_ = true;
    CheckSdk(client_->Stop(app_name_), "PlayStop");
    stream_started_ = false;
    stream_id_.clear();
  }
  stop_failed_ = false;
}

void G1AudioInterfaceNode::StopAll()
{
  sounds_.clear();
  audio_.reset();
  audio_partial_.clear();
  StopStream();
}

void G1AudioInterfaceNode::OnInfo(const audio_common_msgs::msg::AudioInfo& msg)
{
  try {
    const size_t frame_bytes =
        FrameBytes(msg.coding_format, msg.sample_format, msg.sample_rate, msg.channels);
    if (format_ != msg.coding_format || sample_format_ != msg.sample_format ||
        sample_rate_ != static_cast<int>(msg.sample_rate) || channels_ != msg.channels)
    {
      audio_.reset();
      audio_partial_.clear();
    }
    format_ = msg.coding_format;
    sample_format_ = msg.sample_format;
    sample_rate_ = msg.sample_rate;
    channels_ = msg.channels;
    frame_bytes_ = frame_bytes;
    format_valid_ = true;
  } catch (const std::exception& error) {
    format_valid_ = false;
    audio_.reset();
    audio_partial_.clear();
    RCLCPP_ERROR(get_logger(), "Invalid audio_info: %s", error.what());
  }
}

void G1AudioInterfaceNode::OnAudio(const audio_common_msgs::msg::AudioData& msg)
{
  if (msg.data.empty()) {
    return;
  }
  try {
    if (stop_failed_ || !format_valid_) {
      throw std::runtime_error("Audio is blocked by invalid metadata or a failed stop");
    }
    if (msg.data.size() > kMaxInputBytes) {
      throw std::runtime_error("Audio message exceeds the 4 MiB input buffer; pace the publisher");
    }
    if (!audio_) {
      audio_ = CreatePlayback("", format_, sample_format_, sample_rate_, channels_);
    }
    if (gst_app_src_get_current_level_bytes(GST_APP_SRC(audio_->source)) + msg.data.size() >
        kMaxInputBytes)
    {
      throw std::runtime_error("Audio input buffer is full; pace the publisher");
    }
    audio_partial_.insert(audio_partial_.end(), msg.data.begin(), msg.data.end());
    const size_t bytes = format_ == "wave" ? audio_partial_.size() / frame_bytes_ * frame_bytes_
                                           : audio_partial_.size();
    if (bytes == 0) {
      return;
    }
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, bytes, nullptr);
    gst_buffer_fill(buffer, 0, audio_partial_.data(), bytes);
    audio_partial_.erase(audio_partial_.begin(), audio_partial_.begin() + bytes);
    if (gst_app_src_push_buffer(GST_APP_SRC(audio_->source), buffer) != GST_FLOW_OK) {
      audio_.reset();
      throw std::runtime_error("Audio decoder rejected input");
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Audio input failed: %s", error.what());
  }
}

void G1AudioInterfaceNode::ValidateRequest(const Request& msg) const
{
  if (msg.command != Request::PLAY_STOP && msg.command != Request::PLAY_ONCE &&
      msg.command != Request::PLAY_START)
  {
    throw std::invalid_argument("Unknown SoundRequest command");
  }
  if (msg.sound == Request::ALL && msg.command == Request::PLAY_STOP) {
    return;
  }
  if (msg.sound != Request::SAY && msg.sound != Request::PLAY_FILE &&
      (msg.sound < Request::BACKINGUP || msg.sound > Request::NEEDS_PLUGGING_BADLY))
  {
    throw std::invalid_argument("Unknown SoundRequest sound");
  }
  if (msg.command != Request::PLAY_STOP &&
      (!std::isfinite(msg.volume) || msg.volume < 0 || msg.volume > 1))
  {
    throw std::invalid_argument("volume must be finite and in [0, 1]");
  }
  if ((msg.sound == Request::SAY || msg.sound == Request::PLAY_FILE) && msg.arg.empty()) {
    throw std::invalid_argument("Text or filename must be nonempty");
  }
  if (msg.sound == Request::SAY &&
      (msg.command != Request::PLAY_ONCE || (!msg.arg2.empty() && msg.arg2 != "voice_kal_diphone")))
  {
    throw std::invalid_argument("Unitree supports English SAY/PLAY_ONCE only; no TTS stop or loop");
  }
}

std::string G1AudioInterfaceNode::SoundKey(const Request& msg) const
{
  std::string path;
  if (msg.sound == Request::PLAY_FILE) {
    path = msg.arg2.empty()
               ? msg.arg
               : (std::filesystem::path(ament_index_cpp::get_package_share_directory(msg.arg2)) /
                  msg.arg)
                     .string();
  } else {
    path = ament_index_cpp::get_package_share_directory("sound_play") + "/sounds/" +
           kBuiltinFiles[msg.sound - Request::BACKINGUP];
  }
  if (gst_uri_is_valid(path.c_str())) {
    return path;
  }
  const auto absolute = std::filesystem::absolute(path).lexically_normal().string();
  gchar* uri = gst_filename_to_uri(absolute.c_str(), nullptr);
  if (!uri) {
    throw std::invalid_argument("Invalid audio filename");
  }
  const std::string result(uri);
  g_free(uri);
  return result;
}

void G1AudioInterfaceNode::ApplySound(const Request& msg)
{
  ValidateRequest(msg);
  if (msg.sound == Request::ALL) {
    StopAll();
    RCLCPP_WARN(get_logger(), "Stopped PCM/file playback; Unitree TTS cannot be stopped");
    return;
  }
  if (msg.sound == Request::SAY) {
    CheckSdk(client_->SetVolume(static_cast<uint8_t>(std::lround(msg.volume * 100))), "SetVolume");
    CheckSdk(client_->TtsMaker(msg.arg, 1), "TtsMaker");
    return;
  }
  const auto key = SoundKey(msg);
  if (msg.command == Request::PLAY_STOP) {
    if (sounds_.erase(key) != 0 && sounds_.empty() && !audio_) {
      StopStream();
    }
    return;
  }
  if (stop_failed_) {
    throw std::runtime_error("Retry ALL/PLAY_STOP before starting new playback");
  }
  const auto existing = sounds_.find(key);
  if (existing != sounds_.end() && msg.command == Request::PLAY_START && existing->second->loop) {
    existing->second->volume = msg.volume;
    return;
  }
  auto playback = CreatePlayback(key, "", "", 0, 0);
  playback->loop = msg.command == Request::PLAY_START;
  playback->volume = msg.volume;
  sounds_[key] = std::move(playback);
}

void G1AudioInterfaceNode::OnSound(const Request& msg)
{
  try {
    ValidateRequest(msg);
    const bool interrupts = goal_ && (msg.sound == Request::ALL ||
                                      (msg.sound != Request::SAY && SoundKey(msg) == goal_key_));
    ApplySound(msg);
    if (interrupts) {
      FinishGoal(rclcpp_action::ResultCode::ABORTED);
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Sound request failed: %s", error.what());
  }
}

void G1AudioInterfaceNode::FinishGoal(rclcpp_action::ResultCode code)
{
  if (!goal_) {
    return;
  }
  auto result = std::make_shared<Action::Result>();
  result->playing = false;
  result->stamp = rclcpp::Duration(
      std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - goal_start_));
  if (code == rclcpp_action::ResultCode::SUCCEEDED) {
    goal_->succeed(result);
  } else if (code == rclcpp_action::ResultCode::CANCELED) {
    goal_->canceled(result);
  } else {
    goal_->abort(result);
  }
  goal_.reset();
  goal_key_.clear();
}

void G1AudioInterfaceNode::AcceptGoal(const std::shared_ptr<GoalHandle>& goal)
{
  FinishGoal(rclcpp_action::ResultCode::ABORTED);
  goal_ = goal;
  goal_start_ = Clock::now();
  try {
    StopAll();
    const auto& request = goal->get_goal()->sound_request;
    ApplySound(request);
    if (request.command == Request::PLAY_STOP) {
      FinishGoal(rclcpp_action::ResultCode::SUCCEEDED);
    } else {
      goal_key_ = SoundKey(request);
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Sound action failed: %s", error.what());
    FinishGoal(rclcpp_action::ResultCode::ABORTED);
  }
}

void G1AudioInterfaceNode::SendAudio()
{
  try {
    if (goal_ && goal_->is_canceling()) {
      sounds_.erase(goal_key_);
      if (sounds_.empty() && !audio_) {
        StopStream();
      }
      FinishGoal(rclcpp_action::ResultCode::CANCELED);
    }
    if (stop_failed_) {
      return;
    }
    const auto now = Clock::now();
    if (now < next_send_) {
      return;
    }
    std::vector<double> mixed(kSamplesPerChunk, 0.0);
    size_t count = 0;
    const auto mix = [&mixed, &count, now](AudioPlayback& sound) {
      sound.PullSamples();
      const size_t samples = std::min(kSamplesPerChunk, sound.pcm.size() / 2);
      for (size_t index = 0; index < samples; ++index) {
        int value = sound.pcm.front();
        sound.pcm.pop_front();
        value |= static_cast<int>(sound.pcm.front()) << 8;
        sound.pcm.pop_front();
        if (value >= 32768) {
          value -= 65536;
        }
        mixed[index] += value * sound.volume;
      }
      if (samples != 0) {
        sound.playing_until = now + std::chrono::microseconds(samples * 1000000 / 16000);
      }
      count = std::max(count, samples);
    };
    if (audio_) {
      mix(*audio_);
    }
    for (auto it = sounds_.begin(); it != sounds_.end();) {
      try {
        auto& sound = *it->second;
        mix(sound);
        if (sound.pcm.empty() && gst_app_sink_is_eos(GST_APP_SINK(sound.sink)) &&
            now >= sound.playing_until)
        {
          if (!sound.received_samples) {
            throw std::runtime_error("File contains no decodable audio");
          }
          if (sound.loop) {
            if (!gst_element_seek_simple(sound.pipeline, GST_FORMAT_TIME, GST_SEEK_FLAG_FLUSH, 0)) {
              throw std::runtime_error("Audio source cannot be looped");
            }
          } else {
            if (goal_ && goal_key_ == it->first) {
              FinishGoal(rclcpp_action::ResultCode::SUCCEEDED);
            }
            it = sounds_.erase(it);
            continue;
          }
        }
        ++it;
      } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Audio decode failed: %s", error.what());
        if (goal_ && goal_key_ == it->first) {
          FinishGoal(rclcpp_action::ResultCode::ABORTED);
        }
        it = sounds_.erase(it);
      }
    }
    if (count != 0) {
      std::vector<uint8_t> pcm(count * 2);
      for (size_t index = 0; index < count; ++index) {
        const auto value =
            static_cast<int32_t>(std::clamp(std::lround(mixed[index]), -32768L, 32767L));
        pcm[index * 2] = static_cast<uint8_t>(value & 255);
        pcm[index * 2 + 1] = static_cast<uint8_t>((value >> 8) & 255);
      }
      if (stream_id_.empty() || now > next_send_ + std::chrono::seconds(1)) {
        stream_id_ = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
      }
      stream_started_ = true;
      CheckSdk(client_->PlayStream(app_name_, stream_id_, pcm), "PlayStream");
      const auto rpc_time = Clock::now() - now;
      for (auto& entry : sounds_) {
        if (entry.second->playing_until > now) {
          entry.second->playing_until += rpc_time;
        }
      }
      if (now - next_send_ > std::chrono::milliseconds(100)) {
        next_send_ = now;
      }
      next_send_ += std::chrono::microseconds(count * 1000000 / 16000);
    }
    if (goal_) {
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->playing = true;
      feedback->stamp = rclcpp::Duration(
          std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - goal_start_));
      goal_->publish_feedback(feedback);
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Audio playback failed: %s", error.what());
    FinishGoal(rclcpp_action::ResultCode::ABORTED);
    try {
      StopAll();
    } catch (const std::exception& stop_error) {
      RCLCPP_ERROR(get_logger(), "PlayStop failed: %s", stop_error.what());
    }
  }
}
} // namespace g1_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(g1_hardware::G1AudioInterfaceNode)
