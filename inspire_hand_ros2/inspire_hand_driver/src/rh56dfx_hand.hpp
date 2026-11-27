#pragma once

#include <array>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <inspire_hand_interfaces/action/hand_maintenance.hpp>
#include <inspire_hand_interfaces/action/set_force.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace inspire_hand_driver
{

// One worker owns the request/reply stream for one physical hand.
class RH56DFXHand : public std::enable_shared_from_this<RH56DFXHand>
{
public:
  using Positions = std::array<double, 6>;
  RH56DFXHand(const std::string& device, int baudrate, int hand_id);
  ~RH56DFXHand();
  void start(const rclcpp::Node::SharedPtr& node, const std::string& action_prefix,
             const std::string& frame_prefix);
  Positions positions();
  void command(const Positions& target);
  void stop(const Positions* final_target = nullptr);

private:
  using Action = inspire_hand_interfaces::action::HandMaintenance;
  using SetForce = inspire_hand_interfaces::action::SetForce;
  using Wrench = geometry_msgs::msg::WrenchStamped;
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
  template <typename A, typename Validate, typename Execute>
  typename rclcpp_action::Server<A>::SharedPtr add_action(const rclcpp::Node::SharedPtr& node,
                                                          const std::string& name,
                                                          Validate validate, Execute execute);
  void set_force(const SetForce::Goal& goal);
  void publish_forces();

  const int hand_id_;
  int fd_{-1};
  std::mutex mutex_;
  std::condition_variable wake_;
  Positions positions_{}, target_{};
  bool pending_{false}, stopping_{false}, busy_{false};
  std::function<void(bool)> request_;
  std::vector<rclcpp_action::Server<Action>::SharedPtr> servers_;
  rclcpp_action::Server<SetForce>::SharedPtr force_server_;
  std::array<rclcpp::Publisher<Wrench>::SharedPtr, 6> wrench_pubs_;
  std::array<std::string, 6> wrench_frames_;
  rclcpp::Clock::SharedPtr ros_clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("RH56DFXHand")};
  rclcpp::Clock clock_{RCL_STEADY_TIME};
  std::thread worker_;
};

} // namespace inspire_hand_driver
