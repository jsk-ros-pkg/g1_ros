#include "rh56dfx_hand.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <system_error>

namespace inspire_hand_driver
{
namespace
{
constexpr uint8_t kRead = 0x11, kWrite = 0x12;
constexpr uint16_t kPosition = 0x060a, kTarget = 0x05ce, kClear = 0x03ec, kCalibrate = 0x03f1;

bool device_missing(int error)
{
  return error == ENOENT || error == ENODEV || error == ENXIO || error == EIO;
}

bool hung_up(int fd)
{
  pollfd event{fd, 0, 0};
  return poll(&event, 1, 0) > 0 && (event.revents & (POLLERR | POLLHUP | POLLNVAL));
}

void transfer(int fd, uint8_t* buf, size_t size, bool writing, int timeout_ms)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (size) {
    auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                    end - std::chrono::steady_clock::now())
                    .count();
    pollfd event{fd, static_cast<short>(writing ? POLLOUT : POLLIN), 0};
    if (left <= 0)
      throw std::runtime_error("Serial timeout");
    int ready = poll(&event, 1, static_cast<int>(left));
    if (ready < 0 && errno == EINTR)
      continue;
    if (ready <= 0 || !(event.revents & event.events))
      throw std::runtime_error("Serial timeout or disconnect");
    ssize_t count = writing ? ::write(fd, buf, size) : ::read(fd, buf, size);
    if (count < 0 && (errno == EINTR || errno == EAGAIN))
      continue;
    if (count <= 0)
      throw std::runtime_error("Serial transfer failed");
    buf += count;
    size -= count;
  }
}

uint8_t checksum(const std::vector<uint8_t>& frame)
{
  return std::accumulate(frame.begin() + 2, frame.end() - 1, uint8_t{0});
}
} // namespace

RH56DFXHand::RH56DFXHand(const std::string& device, int baudrate, int hand_id) : hand_id_(hand_id)
{
  if (hand_id < 1 || hand_id > 254)
    throw std::invalid_argument("hand_id must be between 1 and 254");
  speed_t speed;
  switch (baudrate) {
  case 115200:
    speed = B115200;
    break;
  case 57600:
    speed = B57600;
    break;
  case 19200:
    speed = B19200;
    break;
  default:
    throw std::invalid_argument("baudrate must be 115200, 57600 or 19200");
  }
  fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0 && device_missing(errno)) {
    positions_.fill(1.0);
    return;
  }
  if (fd_ < 0)
    throw std::system_error(errno, std::generic_category(), device);
  try {
    termios settings{};
    if (flock(fd_, LOCK_EX | LOCK_NB) || tcgetattr(fd_, &settings))
      throw std::system_error(errno, std::generic_category(), device);
    cfmakeraw(&settings);
    settings.c_cflag &= ~(CSTOPB | CRTSCTS);
    settings.c_cflag |= CLOCAL | CREAD;
    if (cfsetspeed(&settings, speed) || tcsetattr(fd_, TCSANOW, &settings))
      throw std::system_error(errno, std::generic_category(), device);
    maintenance(Operation::ClearError);
    positions_ = read_positions();
  } catch (const std::exception&) {
    const bool unplugged = hung_up(fd_);
    close(fd_);
    fd_ = -1;
    if (unplugged) {
      positions_.fill(1.0);
      return;
    }
    throw;
  }
}

RH56DFXHand::~RH56DFXHand()
{
  stop();
  if (fd_ >= 0)
    close(fd_);
}

void RH56DFXHand::send(uint8_t op, uint16_t address, const std::vector<uint8_t>& data)
{
  std::vector<uint8_t> frame{0xeb,
                             0x90,
                             static_cast<uint8_t>(hand_id_),
                             static_cast<uint8_t>(3 + data.size()),
                             op,
                             static_cast<uint8_t>(address),
                             static_cast<uint8_t>(address >> 8)};
  frame.insert(frame.end(), data.begin(), data.end());
  frame.push_back(0);
  frame.back() = checksum(frame);
  if (tcflush(fd_, TCIFLUSH))
    throw std::runtime_error("Serial flush failed");
  transfer(fd_, frame.data(), frame.size(), true, 100);
}

std::vector<uint8_t> RH56DFXHand::reply(uint8_t op, uint16_t address, size_t size, int timeout_ms)
{
  std::vector<uint8_t> frame(size + 8);
  transfer(fd_, frame.data(), frame.size(), false, timeout_ms);
  if (frame[0] != 0x90 || frame[1] != 0xeb || frame[2] != hand_id_ || frame[3] != size + 3 ||
      frame[4] != op || frame[5] != (address & 255) || frame[6] != (address >> 8) ||
      frame.back() != checksum(frame) || (op == kWrite && frame[7] != 1))
    throw std::runtime_error("Invalid RH56DFX response for register " + std::to_string(address));
  return frame;
}

RH56DFXHand::Positions RH56DFXHand::read_positions()
{
  send(kRead, kPosition, {12});
  auto frame = reply(kRead, kPosition, 12);
  Positions result{};
  for (size_t i = 0; i < result.size(); ++i) {
    unsigned value = frame[7 + 2 * i] | (frame[8 + 2 * i] << 8);
    if (value > 1000)
      throw std::runtime_error("Invalid RH56DFX position");
    result[i] = value / 1000.0;
  }
  return result;
}

void RH56DFXHand::send_positions(const Positions& target)
{
  std::vector<uint8_t> data;
  for (double q : target) {
    auto raw = static_cast<uint16_t>(std::lround(std::clamp(q, 0.0, 1.0) * 1000));
    data.push_back(raw & 255);
    data.push_back(raw >> 8);
  }
  send(kWrite, kTarget, data);
  reply(kWrite, kTarget, 1);
}

void RH56DFXHand::maintenance(Operation op)
{
  const auto address = op == Operation::Calibrate ? kCalibrate : kClear;
  send(kWrite, address, {1});
  reply(kWrite, address, 1);
  // RH56 series manual, example 5: calibration finishes with a status frame.
  if (op == Operation::Calibrate && reply(kRead, 0x0002, 1, 10000)[7] != 0)
    throw std::runtime_error("Force sensor calibration failed");
}

void RH56DFXHand::add_action(const rclcpp::Node::SharedPtr& node, const std::string& name,
                             Operation op)
{
  std::weak_ptr<RH56DFXHand> weak = shared_from_this();
  servers_.push_back(rclcpp_action::create_server<Action>(
      node, name,
      [weak](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal>) {
        if (auto self = weak.lock()) {
          std::lock_guard lock(self->mutex_);
          if (!self->busy_ && !self->stopping_) {
            self->busy_ = true;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          }
        }
        return rclcpp_action::GoalResponse::REJECT;
      },
      [](std::shared_ptr<Goal>) { return rclcpp_action::CancelResponse::REJECT; },
      [weak, op](std::shared_ptr<Goal> goal) {
        if (auto self = weak.lock()) {
          std::lock_guard lock(self->mutex_);
          if (!self->stopping_) {
            self->goal_ = goal;
            self->operation_ = op;
            self->wake_.notify_one();
            return;
          }
        }
        auto result = std::make_shared<Action::Result>();
        result->message = "Hand deactivated";
        goal->abort(result);
      }));
}

void RH56DFXHand::start(const rclcpp::Node::SharedPtr& node, const std::string& action_prefix)
{
  logger_ = node->get_logger();
  stopping_ = fd_ < 0;
  add_action(node, action_prefix + "clear_error", Operation::ClearError);
  add_action(node, action_prefix + "calibrate", Operation::Calibrate);
  if (!stopping_)
    worker_ = std::thread(&RH56DFXHand::run, this);
  else
    RCLCPP_WARN(logger_, "Hand is not connected; commands are disabled until reactivation.");
}

void RH56DFXHand::stop(const Positions* final_target)
{
  {
    std::lock_guard lock(mutex_);
    stopping_ = true;
  }
  wake_.notify_one();
  if (worker_.joinable())
    worker_.join();
  if (final_target && fd_ >= 0) {
    try {
      send_positions(*final_target);
    } catch (const std::exception&) {
      if (!hung_up(fd_))
        throw;
      RCLCPP_WARN(logger_, "Hand disconnected before the final position could be sent.");
    }
  }
}

RH56DFXHand::Positions RH56DFXHand::positions()
{
  std::lock_guard lock(mutex_);
  return positions_;
}

void RH56DFXHand::command(const Positions& target)
{
  std::lock_guard lock(mutex_);
  if (stopping_)
    return;
  target_ = target;
  pending_ = true;
}

void RH56DFXHand::run()
{
  std::unique_lock lock(mutex_);
  while (true) {
    auto goal = std::move(goal_);
    const bool stopping = stopping_;
    if (stopping && !goal)
      break;
    const auto op = operation_;
    const auto target = target_;
    const bool write_target = pending_;
    pending_ = false;
    lock.unlock();
    try {
      if (goal) {
        auto result = std::make_shared<Action::Result>();
        try {
          if (stopping)
            throw std::runtime_error("Hand deactivated");
          maintenance(op);
        } catch (const std::exception& e) {
          result->message = e.what();
        }
        if (result->message.empty()) {
          result->message = "Completed";
          goal->succeed(result);
        } else {
          goal->abort(result);
        }
      } else {
        if (write_target)
          send_positions(target);
        auto sample = read_positions();
        std::lock_guard guard(mutex_);
        positions_ = sample;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(logger_, clock_, 2000, "%s", e.what());
    }
    lock.lock();
    if (fd_ >= 0 && hung_up(fd_)) {
      close(fd_);
      fd_ = -1;
      stopping_ = true;
      RCLCPP_WARN(logger_, "Hand disconnected; commands are disabled until reactivation.");
    }
    if (goal)
      busy_ = false;
    wake_.wait_for(lock, std::chrono::milliseconds(10), [this] { return stopping_ || goal_; });
  }
}

} // namespace inspire_hand_driver
