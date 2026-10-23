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

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

namespace g1_hardware
{
namespace
{
struct Calls
{
  std::vector<std::vector<uint8_t>> chunks;
  std::vector<std::string> streams;
  std::vector<std::chrono::steady_clock::time_point> times;
  std::string app;
  std::string stopped_app;
  std::string text;
  int speaker{-1};
  int volume{-1};
  int stops{0};
  int play_result{0};
  int stop_result{0};
  int volume_result{0};
  bool throw_play{false};
};

class FakeBackend : public AudioBackend
{
public:
  explicit FakeBackend(std::shared_ptr<Calls> calls) : calls_(std::move(calls)) {}

  int32_t Play(const std::string& app, const std::string& stream,
               const std::vector<uint8_t>& pcm) override
  {
    calls_->app = app;
    calls_->streams.push_back(stream);
    calls_->chunks.push_back(pcm);
    calls_->times.push_back(std::chrono::steady_clock::now());
    if (calls_->throw_play) {
      throw std::runtime_error("connection lost");
    }
    return calls_->play_result;
  }

  int32_t Stop(const std::string& app) override
  {
    ++calls_->stops;
    calls_->stopped_app = app;
    return calls_->stop_result;
  }

  int32_t Say(const std::string& text, int32_t speaker) override
  {
    calls_->text = text;
    calls_->speaker = speaker;
    return 0;
  }

  int32_t SetVolume(uint8_t volume) override
  {
    calls_->volume = volume;
    return calls_->volume_result;
  }

private:
  std::shared_ptr<Calls> calls_;
};

class AudioTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    calls_ = std::make_shared<Calls>();
    rclcpp::NodeOptions options;
    // Use simulated time to verify that streaming uses a wall clock.
    options.arguments({"--ros-args", "-r", "__ns:=/g1_audio_test"});
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    node_ = std::make_shared<G1AudioInterfaceNode>(options, std::make_unique<FakeBackend>(calls_));
    peer_ = std::make_shared<rclcpp::Node>("audio_test_publisher", "/g1_audio_test");
    audio_ = peer_->create_publisher<audio_common_msgs::msg::AudioData>("audio", 10);
    info_ = peer_->create_publisher<audio_common_msgs::msg::AudioInfo>(
        "audio_info", rclcpp::QoS(1).transient_local());
    sound_ = peer_->create_publisher<sound_play_msgs::msg::SoundRequest>("robotsound", 10);
    executor_.add_node(node_);
    executor_.add_node(peer_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (audio_->get_subscription_count() == 0 || sound_->get_subscription_count() == 0 ||
           info_->get_subscription_count() == 0)
    {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline);
      Spin(10);
    }
  }

  void TearDown() override
  {
    executor_.remove_node(node_);
    executor_.remove_node(peer_);
    node_.reset();
    peer_.reset();
  }

  void Spin(int milliseconds)
  {
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    do {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < until);
  }

  void PublishAudio(size_t bytes)
  {
    audio_common_msgs::msg::AudioData msg;
    for (size_t index = 0; index < bytes; ++index) {
      msg.data.push_back(static_cast<uint8_t>(index % 256));
    }
    audio_->publish(msg);
  }

  void Stop()
  {
    sound_play_msgs::msg::SoundRequest msg;
    msg.sound = msg.ALL;
    msg.command = msg.PLAY_STOP;
    sound_->publish(msg);
    Spin(30);
  }

  std::shared_ptr<Calls> calls_;
  std::shared_ptr<G1AudioInterfaceNode> node_;
  rclcpp::Node::SharedPtr peer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<audio_common_msgs::msg::AudioData>::SharedPtr audio_;
  rclcpp::Publisher<audio_common_msgs::msg::AudioInfo>::SharedPtr info_;
  rclcpp::Publisher<sound_play_msgs::msg::SoundRequest>::SharedPtr sound_;
};

TEST_F(AudioTest, SplitsAndPacesPcmWithoutTruncatingTail)
{
  PublishAudio(7000);
  Spin(350);
  ASSERT_EQ(calls_->chunks.size(), 3U);
  EXPECT_EQ(calls_->chunks[0].size(), 3200U);
  EXPECT_EQ(calls_->chunks[1].size(), 3200U);
  EXPECT_EQ(calls_->chunks[2].size(), 600U);
  std::vector<uint8_t> received;
  for (const auto& chunk : calls_->chunks) {
    received.insert(received.end(), chunk.begin(), chunk.end());
  }
  for (size_t index = 0; index < received.size(); ++index) {
    EXPECT_EQ(received[index], index % 256);
  }
  EXPECT_EQ(calls_->streams[0], calls_->streams[2]);
  EXPECT_GE(calls_->times[1] - calls_->times[0], std::chrono::milliseconds(90));
  EXPECT_EQ(calls_->stops, 0);
  EXPECT_EQ(calls_->app, "g1_ros");
}

TEST_F(AudioTest, RejectsOddEmptyAndOversizedBuffers)
{
  PublishAudio(0);
  PublishAudio(3);
  PublishAudio(320002);
  Spin(100);
  EXPECT_TRUE(calls_->chunks.empty());
}

TEST_F(AudioTest, RejectsIncompatibleMetadataAndRecovers)
{
  audio_common_msgs::msg::AudioInfo info;
  info.channels = 1;
  info.sample_rate = 48000;
  info.sample_format = "S16LE";
  info.coding_format = "wave";
  info_->publish(info);
  Spin(30);
  PublishAudio(100);
  Spin(30);
  EXPECT_TRUE(calls_->chunks.empty());
  info.sample_rate = 16000;
  info_->publish(info);
  Spin(30);
  PublishAudio(100);
  Spin(30);
  EXPECT_EQ(calls_->chunks.size(), 1U);
}

TEST_F(AudioTest, StopClearsPendingAudioAndUsesAppName)
{
  PublishAudio(32000);
  Spin(30);
  ASSERT_FALSE(calls_->chunks.empty());
  Stop();
  const auto count = calls_->chunks.size();
  Spin(150);
  EXPECT_EQ(calls_->chunks.size(), count);
  EXPECT_EQ(calls_->stopped_app, "g1_ros");
  const auto old_stream = calls_->streams.back();
  PublishAudio(100);
  Spin(30);
  EXPECT_NE(calls_->streams.back(), old_stream);
}

TEST_F(AudioTest, PlaybackFailureDropsPendingAudio)
{
  calls_->play_result = 42;
  PublishAudio(32000);
  Spin(200);
  EXPECT_EQ(calls_->chunks.size(), 1U);
  EXPECT_EQ(calls_->stops, 1);
}

TEST_F(AudioTest, PlaybackExceptionIsContained)
{
  calls_->throw_play = true;
  PublishAudio(32000);
  Spin(200);
  EXPECT_EQ(calls_->chunks.size(), 1U);
  EXPECT_EQ(calls_->stops, 1);
}

TEST_F(AudioTest, FailedStopBlocksNewAudioUntilSuccessfulStop)
{
  PublishAudio(100);
  Spin(30);
  calls_->stop_result = 42;
  Stop();
  PublishAudio(100);
  Spin(30);
  EXPECT_EQ(calls_->chunks.size(), 1U);
  calls_->stop_result = 0;
  Stop();
  PublishAudio(100);
  Spin(30);
  EXPECT_EQ(calls_->chunks.size(), 2U);
}

TEST_F(AudioTest, MapsSayVoiceAndNormalizedVolume)
{
  sound_play_msgs::msg::SoundRequest msg;
  msg.sound = msg.SAY;
  msg.command = msg.PLAY_ONCE;
  msg.arg = "Hello";
  msg.volume = 0.42F;
  sound_->publish(msg);
  Spin(30);
  EXPECT_EQ(calls_->text, "Hello");
  EXPECT_EQ(calls_->speaker, 1);
  EXPECT_EQ(calls_->volume, 42);
  msg.arg2 = "0";
  msg.volume = 0;
  sound_->publish(msg);
  Spin(30);
  EXPECT_EQ(calls_->speaker, 0);
  EXPECT_EQ(calls_->volume, 0);
}

TEST_F(AudioTest, RejectsUnsupportedRequestsAndInvalidVolume)
{
  sound_play_msgs::msg::SoundRequest msg;
  msg.sound = msg.SAY;
  msg.command = msg.PLAY_ONCE;
  msg.arg = "Hello";
  for (const auto volume : {-1.0F, 1.1F, std::numeric_limits<float>::quiet_NaN()}) {
    msg.volume = volume;
    sound_->publish(msg);
    Spin(20);
  }
  msg.volume = 0.5F;
  msg.arg2 = "unsupported_voice";
  sound_->publish(msg);
  Spin(20);
  msg.arg2.clear();
  msg.command = msg.PLAY_START;
  sound_->publish(msg);
  Spin(20);
  msg.command = msg.PLAY_ONCE;
  msg.sound = msg.PLAY_FILE;
  sound_->publish(msg);
  Spin(20);
  EXPECT_TRUE(calls_->text.empty());
  EXPECT_EQ(calls_->volume, -1);
}

TEST_F(AudioTest, VolumeFailurePreventsSpeech)
{
  calls_->volume_result = 42;
  sound_play_msgs::msg::SoundRequest msg;
  msg.sound = msg.SAY;
  msg.command = msg.PLAY_ONCE;
  msg.arg = "Hello";
  msg.volume = 1;
  sound_->publish(msg);
  Spin(30);
  EXPECT_TRUE(calls_->text.empty());
}

TEST_F(AudioTest, RejectsUnsupportedStartupParameters)
{
  for (const auto& parameter :
       {rclcpp::Parameter("sample_rate", 48000), rclcpp::Parameter("channels", 2),
        rclcpp::Parameter("depth", 8), rclcpp::Parameter("sample_format", "S16BE"),
        rclcpp::Parameter("format", "mp3"), rclcpp::Parameter("speaker_id", 2),
        rclcpp::Parameter("app_name", "")})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({parameter});
    EXPECT_THROW(G1AudioInterfaceNode(options, std::make_unique<FakeBackend>(calls_)),
                 std::invalid_argument);
  }
}
} // namespace
} // namespace g1_hardware
