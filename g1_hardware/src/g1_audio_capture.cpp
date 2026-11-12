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

#include <array>
#include <cerrno>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>

#include <arpa/inet.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#include <audio_common_msgs/msg/audio_data.hpp>
#include <audio_common_msgs/msg/audio_data_stamped.hpp>
#include <audio_common_msgs/msg/audio_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace g1_hardware
{
namespace
{
class MicrophoneSocket
{
public:
  explicit MicrophoneSocket(const std::string& interface)
  {
    const auto index = if_nametoindex(interface.c_str());
    if (index == 0) {
      throw std::system_error(errno, std::generic_category(), "Microphone network interface");
    }
    fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) {
      throw std::system_error(errno, std::generic_category(), "Microphone socket");
    }
    try {
      const int reuse = 1;
      Check(setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)), "SO_REUSEADDR");
      const int all = 0;
      Check(setsockopt(fd, IPPROTO_IP, IP_MULTICAST_ALL, &all, sizeof(all)), "IP_MULTICAST_ALL");
      sockaddr_in address{};
      address.sin_family = AF_INET;
      address.sin_port = htons(5555);
      inet_pton(AF_INET, "239.168.123.161", &address.sin_addr);
      Check(bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)), "Microphone bind");
      ip_mreqn membership{};
      membership.imr_multiaddr = address.sin_addr;
      membership.imr_ifindex = static_cast<int>(index);
      Check(setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &membership, sizeof(membership)),
            "Microphone multicast join");
    } catch (...) {
      close(fd);
      throw;
    }
  }

  ~MicrophoneSocket() { close(fd); }

  MicrophoneSocket(const MicrophoneSocket&) = delete;
  MicrophoneSocket& operator=(const MicrophoneSocket&) = delete;
  int fd{-1};

private:
  static void Check(int result, const char* operation)
  {
    if (result < 0) {
      throw std::system_error(errno, std::generic_category(), operation);
    }
  }
};
} // namespace

class G1AudioCaptureNode : public rclcpp::Node
{
public:
  explicit G1AudioCaptureNode(const rclcpp::NodeOptions& options)
  : Node("g1_audio_capture", options)
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    const auto interface = declare_parameter<std::string>("network_interface", "", fixed);
    frame_id_ = declare_parameter<std::string>("frame_id", "", fixed);
    if (interface.empty()) {
      throw std::invalid_argument("network_interface must be nonempty");
    }
    socket_ = std::make_unique<MicrophoneSocket>(interface);
    audio_pub_ = create_publisher<audio_common_msgs::msg::AudioData>("microphone/audio", 10);
    stamped_pub_ =
        create_publisher<audio_common_msgs::msg::AudioDataStamped>("microphone/audio_stamped", 10);
    info_pub_ = create_publisher<audio_common_msgs::msg::AudioInfo>(
        "microphone/audio_info", rclcpp::QoS(1).transient_local());
    asr_pub_ = create_publisher<std_msgs::msg::String>("recognizer/output", 10);
    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    asr_sub_ = std::make_unique<unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>>(
        "rt/audio_msg");
    asr_sub_->InitChannel(
        [this](const void* message) {
          std_msgs::msg::String output;
          output.data = static_cast<const std_msgs::msg::dds_::String_*>(message)->data();
          asr_pub_->publish(output);
        },
        10);
    info_timer_ = create_wall_timer(std::chrono::seconds(5), [this] { PublishInfo(); });
    receive_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { Receive(); });
    PublishInfo();
    RCLCPP_INFO(get_logger(), "Receiving microphone and ASR on %s", interface.c_str());
  }

  ~G1AudioCaptureNode() override
  {
    if (asr_sub_) {
      asr_sub_->CloseChannel();
    }
  }

private:
  void PublishInfo()
  {
    audio_common_msgs::msg::AudioInfo info;
    info.channels = 1;
    info.sample_rate = 16000;
    info.sample_format = "S16LE";
    info.bitrate = 256000;
    info.coding_format = "wave";
    info_pub_->publish(info);
  }

  void Receive()
  {
    std::array<uint8_t, 65536> buffer{};
    for (int packet = 0; packet < 64; ++packet) {
      const auto size = recv(socket_->fd, buffer.data(), buffer.size(), 0);
      if (size < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Microphone receive failed: %s",
                                std::system_category().message(errno).c_str());
        }
        return;
      }
      if (size == 0 || size % 2 != 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Discarding invalid microphone PCM packet (%zd bytes)", size);
        continue;
      }
      audio_common_msgs::msg::AudioDataStamped message;
      // Unitree's UDP payload has no timestamp; use ROS reception time.
      message.header.stamp = now();
      message.header.frame_id = frame_id_;
      message.audio.data.assign(buffer.begin(), buffer.begin() + size);
      audio_pub_->publish(message.audio);
      stamped_pub_->publish(message);
    }
  }

  std::string frame_id_;
  std::unique_ptr<MicrophoneSocket> socket_;
  rclcpp::Publisher<audio_common_msgs::msg::AudioData>::SharedPtr audio_pub_;
  rclcpp::Publisher<audio_common_msgs::msg::AudioDataStamped>::SharedPtr stamped_pub_;
  rclcpp::Publisher<audio_common_msgs::msg::AudioInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr asr_pub_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>> asr_sub_;
  rclcpp::TimerBase::SharedPtr info_timer_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
};
} // namespace g1_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(g1_hardware::G1AudioCaptureNode)
