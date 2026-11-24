#pragma once

#include <array>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <g1_hand_interfaces/action/hand_maintenance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace g1_hardware
{

// One worker owns the request/reply stream for one physical hand.
class RH56DFXHand : public std::enable_shared_from_this<RH56DFXHand>
{
public:
  using Positions = std::array<double, 6>;
  explicit RH56DFXHand(const std::string& device);
  ~RH56DFXHand();
  void start(const rclcpp::Node::SharedPtr& node, const std::string& side);
  Positions positions();
  void command(const Positions& target);
  void stop(const Positions* final_target = nullptr);

private:
  using Action = g1_hand_interfaces::action::HandMaintenance;
  using Goal = rclcpp_action::ServerGoalHandle<Action>;
  enum class Operation
  {
    ClearError,
    Calibrate
  };
  void run();
  void send_positions(const Positions& target);
  Positions read_positions();
  void send(uint8_t op, uint16_t address, const std::vector<uint8_t>& data);
  std::vector<uint8_t> reply(uint8_t op, uint16_t address, size_t size, int timeout_ms = 100);
  void maintenance(Operation op);
  void add_action(const rclcpp::Node::SharedPtr& node, const std::string& name, Operation op);

  int fd_{-1};
  std::mutex mutex_;
  std::condition_variable wake_;
  Positions positions_{}, target_{};
  bool pending_{false}, stopping_{false}, busy_{false};
  Operation operation_{Operation::ClearError};
  std::shared_ptr<Goal> goal_;
  std::vector<rclcpp_action::Server<Action>::SharedPtr> servers_;
  rclcpp::Logger logger_{rclcpp::get_logger("RH56DFXHand")};
  rclcpp::Clock clock_{RCL_STEADY_TIME};
  std::thread worker_;
};

} // namespace g1_hardware
