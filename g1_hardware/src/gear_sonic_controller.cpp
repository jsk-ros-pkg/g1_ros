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

/**
 * FollowJointTrajectory → SONIC VR 3-point adapter.
 *
 * Exposes the standard `~/follow_joint_trajectory` action (the same interface
 * a joint_trajectory_controller provides, so MoveIt can execute on it via
 * moveit_simple_controller_manager) and converts the commanded upper-body
 * joint trajectory into gear_sonic VR 3-point pose targets:
 *
 *   1. The trajectory is sampled at publish_rate with linear interpolation.
 *   2. Forward kinematics (KDL, chains parsed from /robot_description) gives
 *      the pelvis-frame poses of left_wrist_yaw_link, right_wrist_yaw_link
 *      and torso_link for the sampled joint positions.
 *   3. The three poses are published to the gear_sonic_interface target
 *      topics; gear_sonic_interface converts them to policy keypoints
 *      (vr_3point_body_offset) and streams them to the SONIC deploy stack.
 *   4. The same sampled 17-DOF upper-body joint reference is published by
 *      name. SONIC uses it as the motion-joint reference while VR keypoints
 *      remain the end-effector task, preventing the policy from choosing a
 *      very different arm branch with the same approximate wrist position.
 *
 * The RL policy will generally realise DIFFERENT joint angles than the
 * commanded trajectory — only the end-effector (wrist) and torso poses are
 * reproduced. Keep gear_sonic_interface's vr_compliance at {0,0,0} (default)
 * for accurate reproduction.
 *
 * After a trajectory finishes the last FK targets are held (published every
 * tick) so that gear_sonic_interface's pose_timeout does not release VR
 * tracking. Call ~/release_targets after a task to stop that hold and let the
 * SONIC interface time out back to its natural attention policy. Chain joints
 * that are not part of the trajectory are frozen at their current
 * /joint_states values when the goal is accepted.
 *
 * ── Virtual base-height DOF (squat planning) ───────────────────────────────
 * If the trajectory contains height_joint (default "pelvis_height_joint" —
 * the virtual prismatic joint added to the MoveIt model by use_pelvis_lift),
 * its value q (<= 0) is NOT fed to FK. Instead:
 *   - base height = standing_height + q is published to gear_sonic_interface
 *     ~/target_height every tick, and
 *   - the locomotion mode is switched to idle_squat while q is below a small
 *     threshold, and back to default when q returns to ~0. The SONIC policy
 *     ignores the height command in IDLE but follows it continuously in
 *     idle_squat (verified in sim), so the mode switch is handled here and
 *     hidden from the user.
 * The joint's commanded value is also published to /joint_states so that
 * MoveIt's current state stays consistent (open-loop: the commanded value is
 * assumed achieved). robot_state_publisher derives the pelvis_lift_base ->
 * pelvis TF from it when the URDF includes the lift joint (use_pelvis_lift);
 * set publish_lift_tf:=true to broadcast that TF from here instead when no
 * lift-aware robot_state_publisher is running.
 *
 * ── Parameters ─────────────────────────────────────────────────────────────
 *   publish_rate       (double, default 50.0 Hz)
 *   base_link          (string, default "pelvis")
 *   left_tip_link      (string, default "left_wrist_yaw_link")
 *   right_tip_link     (string, default "right_wrist_yaw_link")
 *   torso_tip_link     (string, default "torso_link")
 *   target_topic_left  (string, default "/gear_sonic_interface/target_left_wrist_yaw_link")
 *   target_topic_right (string, default "/gear_sonic_interface/target_right_wrist_yaw_link")
 *   target_topic_torso (string, default "/gear_sonic_interface/target_torso_link")
 *   target_topic_height (string, default "/gear_sonic_interface/target_height")
 *   target_topic_upper_body (string, default
 *       "/gear_sonic_interface/target_upper_body_joints")
 *   height_joint       (string, default "pelvis_height_joint")
 *   lift_base_link     (string, default "pelvis_lift_base") — TF parent for the lift joint
 *   publish_lift_tf    (bool, default false) — broadcast lift TF from this node
 *   standing_height    (double, default 0.789 m — kplanner default_height)
 *   squat_mode_service (string, default "/gear_sonic_interface/mode/idle_squat")
 *   default_mode_service (string, default "/gear_sonic_interface/mode/default")
 *   release service      ~/release_targets (std_srvs/srv/Trigger)
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace g1_hardware
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class GearSonicController : public rclcpp::Node
{
public:
  explicit GearSonicController(const rclcpp::NodeOptions& options)
  : rclcpp::Node("gear_sonic_controller", options)
  {
    const double publish_rate = declare_parameter("publish_rate", 50.0);
    base_link_ = declare_parameter("base_link", std::string("pelvis"));
    chains_[0].tip = declare_parameter("left_tip_link", std::string("left_wrist_yaw_link"));
    chains_[1].tip = declare_parameter("right_tip_link", std::string("right_wrist_yaw_link"));
    chains_[2].tip = declare_parameter("torso_tip_link", std::string("torso_link"));

    const std::array<std::string, 3> topics = {
        declare_parameter("target_topic_left",
                          std::string("/gear_sonic_interface/target_left_wrist_yaw_link")),
        declare_parameter("target_topic_right",
                          std::string("/gear_sonic_interface/target_right_wrist_yaw_link")),
        declare_parameter("target_topic_torso",
                          std::string("/gear_sonic_interface/target_torso_link")),
    };
    for (size_t i = 0; i < 3; ++i) {
      chains_[i].pub = create_publisher<geometry_msgs::msg::PoseStamped>(topics[i], 1);
    }

    // Virtual base-height DOF (squat planning); see the header comment.
    height_joint_ = declare_parameter("height_joint", std::string("pelvis_height_joint"));
    lift_base_link_ = declare_parameter("lift_base_link", std::string("pelvis_lift_base"));
    standing_height_ = declare_parameter("standing_height", 0.789);
    height_pub_ = create_publisher<std_msgs::msg::Float64>(
        declare_parameter("target_topic_height",
                          std::string("/gear_sonic_interface/target_height")),
        1);
    squat_mode_cli_ = create_client<std_srvs::srv::Trigger>(declare_parameter(
        "squat_mode_service", std::string("/gear_sonic_interface/mode/idle_squat")));
    default_mode_cli_ = create_client<std_srvs::srv::Trigger>(declare_parameter(
        "default_mode_service", std::string("/gear_sonic_interface/mode/default")));
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    upper_body_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        declare_parameter("target_topic_upper_body",
                          std::string("/gear_sonic_interface/target_upper_body_joints")),
        1);
    if (declare_parameter("publish_lift_tf", false)) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // robot_state_publisher publishes /robot_description with transient_local QoS
    robot_description_sub_ = create_subscription<std_msgs::msg::String>(
        "/robot_description", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) { OnRobotDescription(msg); });

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
            latest_joint_states_[msg->name[i]] = msg->position[i];
          }
        });

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        this, "~/follow_joint_trajectory",
        [this](const rclcpp_action::GoalUUID&,
               std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
          return HandleGoal(goal);
        },
        [](std::shared_ptr<GoalHandleFJT>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](std::shared_ptr<GoalHandleFJT> goal_handle) { HandleAccepted(goal_handle); });

    release_targets_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/release_targets",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (active_goal_) {
            response->success = false;
            response->message =
                "Cannot release SONIC targets while a trajectory is active";
            return;
          }
          holding_ = false;
          trajectory_.points.clear();
          joint_cmd_.clear();
          joint_vel_cmd_.clear();
          response->success = true;
          response->message =
              "Stopped FK target publication; SONIC may return to attention";
          RCLCPP_INFO(
              get_logger(),
              "Released held FK targets; waiting for gear_sonic_interface pose timeout.");
        });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = create_wall_timer(period, [this]() { Tick(); });

    RCLCPP_INFO(
        get_logger(), "Waiting for /robot_description to build FK chains (%s -> %s, %s, %s).",
        base_link_.c_str(), chains_[0].tip.c_str(), chains_[1].tip.c_str(), chains_[2].tip.c_str());
  }

private:
  struct ChainContext
  {
    std::string tip;
    KDL::Chain chain;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> solver;
    std::vector<std::string> joint_names; // moving joints, in chain order
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
  };

  void OnRobotDescription(std_msgs::msg::String::ConstSharedPtr msg)
  {
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(msg->data, tree)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse /robot_description into a KDL tree.");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    known_joints_.clear();
    for (auto& ctx : chains_) {
      if (!tree.getChain(base_link_, ctx.tip, ctx.chain)) {
        RCLCPP_ERROR(get_logger(), "No kinematic chain from '%s' to '%s' in the URDF.",
                     base_link_.c_str(), ctx.tip.c_str());
        return;
      }
      ctx.solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(ctx.chain);
      ctx.joint_names.clear();
      for (unsigned int i = 0; i < ctx.chain.getNrOfSegments(); ++i) {
        const auto& joint = ctx.chain.getSegment(i).getJoint();
        if (joint.getType() != KDL::Joint::None) {
          ctx.joint_names.push_back(joint.getName());
          known_joints_.insert(joint.getName());
        }
      }
    }
    // The virtual lift joint is not in the (real) URDF chains but is a valid
    // trajectory joint: it maps to the SONIC height command instead of FK.
    known_joints_.insert(height_joint_);
    ready_ = true;
    RCLCPP_INFO(get_logger(),
                "FK chains ready (%zu joints total, incl. virtual '%s'). Action server: "
                "~/follow_joint_trajectory",
                known_joints_.size(), height_joint_.c_str());
  }

  rclcpp_action::GoalResponse HandleGoal(std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready_) {
      RCLCPP_ERROR(get_logger(), "Goal rejected: /robot_description not received yet.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->trajectory.points.empty()) {
      RCLCPP_ERROR(get_logger(), "Goal rejected: empty trajectory.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    for (const auto& name : goal->trajectory.joint_names) {
      if (known_joints_.count(name) == 0) {
        RCLCPP_ERROR(get_logger(),
                     "Goal rejected: joint '%s' is not part of the %s -> wrist/torso chains.",
                     name.c_str(), base_link_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    const size_t n = goal->trajectory.joint_names.size();
    for (const auto& pt : goal->trajectory.points) {
      if (pt.positions.size() != n) {
        RCLCPP_ERROR(get_logger(), "Goal rejected: point with %zu positions, expected %zu.",
                     pt.positions.size(), n);
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void HandleAccepted(std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      result->error_string = "Preempted by a new trajectory goal";
      active_goal_->abort(result);
    }
    // Freeze chain joints that are not commanded by this trajectory at their
    // current measured positions (the RL policy may have moved them).
    const auto& names = goal_handle->get_goal()->trajectory.joint_names;
    for (const auto& joint : known_joints_) {
      if (std::find(names.begin(), names.end(), joint) == names.end()) {
        const auto it = latest_joint_states_.find(joint);
        if (it != latest_joint_states_.end()) {
          joint_cmd_[joint] = it->second;
        } else if (joint_cmd_.count(joint) == 0) {
          RCLCPP_WARN(get_logger(),
                      "Joint '%s' is neither in the trajectory nor in /joint_states; using 0.0.",
                      joint.c_str());
          joint_cmd_[joint] = 0.0;
        }
      }
      if (std::find(names.begin(), names.end(), joint) == names.end()) {
        joint_vel_cmd_[joint] = 0.0;
      }
    }
    active_goal_ = goal_handle;
    trajectory_ = goal_handle->get_goal()->trajectory;
    start_time_ = now();
  }

  // Sample trajectory_ at time t (seconds from start) into joint_cmd_.
  // Returns true when t is past the final point.
  bool SampleTrajectory(double t)
  {
    const auto& pts = trajectory_.points;
    const auto tfs = [](const trajectory_msgs::msg::JointTrajectoryPoint& p) {
      return rclcpp::Duration(p.time_from_start).seconds();
    };

    size_t seg = 0;
    double alpha = 0.0;
    bool finished = false;
    if (t <= tfs(pts.front())) {
      seg = 0;
      alpha = 0.0;
    } else if (t >= tfs(pts.back())) {
      seg = pts.size() - 1;
      alpha = 0.0;
      finished = true;
    } else {
      while (seg + 1 < pts.size() && tfs(pts[seg + 1]) <= t) {
        ++seg;
      }
      const double t0 = tfs(pts[seg]);
      const double t1 = tfs(pts[seg + 1]);
      alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
    }

    for (size_t j = 0; j < trajectory_.joint_names.size(); ++j) {
      double q = pts[seg].positions[j];
      double dq = 0.0;
      if (alpha > 0.0 && seg + 1 < pts.size()) {
        q += alpha * (pts[seg + 1].positions[j] - q);
      }
      if (!finished && seg + 1 < pts.size()) {
        const double dt = tfs(pts[seg + 1]) - tfs(pts[seg]);
        if (dt > 1e-9) {
          dq = (pts[seg + 1].positions[j] - pts[seg].positions[j]) / dt;
        }
      }
      joint_cmd_[trajectory_.joint_names[j]] = q;
      joint_vel_cmd_[trajectory_.joint_names[j]] = dq;
    }
    return finished;
  }

  void PublishFkTargets()
  {
    const auto stamp = now();
    for (auto& ctx : chains_) {
      KDL::JntArray q(ctx.chain.getNrOfJoints());
      for (size_t j = 0; j < ctx.joint_names.size(); ++j) {
        const auto it = joint_cmd_.find(ctx.joint_names[j]);
        q(j) = (it != joint_cmd_.end()) ? it->second : 0.0;
      }
      KDL::Frame frame;
      if (ctx.solver->JntToCart(q, frame) < 0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "FK failed for chain to '%s'.",
                              ctx.tip.c_str());
        continue;
      }
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = base_link_;
      msg.pose.position.x = frame.p.x();
      msg.pose.position.y = frame.p.y();
      msg.pose.position.z = frame.p.z();
      frame.M.GetQuaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                            msg.pose.orientation.w);
      ctx.pub->publish(msg);
    }
  }

  void PublishUpperBodyTargets()
  {
    // Exact SONIC IsaacLab upper-body order. Keep names in the message so the
    // interface independently validates and orders the data before packing ZMQ.
    static constexpr std::array<const char*, 17> kJointNames{
        "waist_yaw_joint",          "waist_roll_joint",          "waist_pitch_joint",
        "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
        "left_shoulder_roll_joint",  "right_shoulder_roll_joint",
        "left_shoulder_yaw_joint",   "right_shoulder_yaw_joint",
        "left_elbow_joint",          "right_elbow_joint",
        "left_wrist_roll_joint",     "right_wrist_roll_joint",
        "left_wrist_pitch_joint",    "right_wrist_pitch_joint",
        "left_wrist_yaw_joint",      "right_wrist_yaw_joint"};

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.reserve(kJointNames.size());
    msg.position.reserve(kJointNames.size());
    msg.velocity.reserve(kJointNames.size());
    for (const char* name : kJointNames) {
      const auto pos = joint_cmd_.find(name);
      if (pos == joint_cmd_.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Cannot publish SONIC upper-body reference: joint '%s' has no target.",
                             name);
        return;
      }
      const auto vel = joint_vel_cmd_.find(name);
      msg.name.emplace_back(name);
      msg.position.push_back(pos->second);
      msg.velocity.push_back(vel != joint_vel_cmd_.end() ? vel->second : 0.0);
    }
    upper_body_pub_->publish(msg);
  }

  void UpdateSquatMode(double lift_q)
  {
    // Hysteresis so the mode does not flap around lift_q ~ 0.
    if (!squat_mode_ && lift_q < -0.02) {
      squat_mode_ = true;
      RCLCPP_INFO(get_logger(), "Lift %.3f m: switching locomotion mode to idle_squat.", lift_q);
      squat_mode_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    } else if (squat_mode_ && lift_q > -0.005) {
      squat_mode_ = false;
      RCLCPP_INFO(get_logger(), "Lift returned to ~0 m: switching locomotion mode to default.");
      default_mode_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }
  }

  void Tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready_) {
      return;
    }

    if (active_goal_) {
      const double t = (now() - start_time_).seconds();
      const bool finished = SampleTrajectory(t);

      if (active_goal_->is_canceling()) {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        result->error_string = "Trajectory canceled; holding current targets";
        active_goal_->canceled(result);
        active_goal_.reset();
        holding_ = true;
      } else if (finished) {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        active_goal_->succeed(result);
        active_goal_.reset();
        holding_ = true;
      }
    }

    const auto lift_it = joint_cmd_.find(height_joint_);
    const double lift_q = (lift_it != joint_cmd_.end()) ? lift_it->second : 0.0;

    // The virtual lift joint's commanded value is published every tick — even
    // before the first goal — so MoveIt's current state monitor is complete
    // from startup and robot_state_publisher can derive the lift TF.
    const auto stamp = now();
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name.push_back(height_joint_);
    js.position.push_back(lift_q);
    joint_state_pub_->publish(js);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = lift_base_link_;
      tf.child_frame_id = base_link_;
      tf.transform.translation.z = lift_q;
      tf.transform.rotation.w = 1.0;
      tf_broadcaster_->sendTransform(tf);
    }

    if (!active_goal_ && !holding_) {
      return;
    }

    // Publish every tick (also while holding) so gear_sonic_interface's
    // pose_timeout does not release VR tracking between goals.
    PublishUpperBodyTargets();
    PublishFkTargets();

    std_msgs::msg::Float64 height;
    height.data = standing_height_ + lift_q;
    height_pub_->publish(height);
    UpdateSquatMode(lift_q);
  }

  std::string base_link_;
  std::array<ChainContext, 3> chains_; // 0 = left wrist, 1 = right wrist, 2 = torso

  // Virtual base-height DOF (squat planning)
  std::string height_joint_;
  std::string lift_base_link_;
  double standing_height_{0.789};
  bool squat_mode_{false};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr upper_body_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr squat_mode_cli_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr default_mode_cli_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_targets_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  bool ready_{false};
  bool holding_{false}; // keep publishing last targets after a goal ends
  std::set<std::string> known_joints_;
  std::map<std::string, double> latest_joint_states_;
  std::map<std::string, double> joint_cmd_;
  std::map<std::string, double> joint_vel_cmd_;
  std::shared_ptr<GoalHandleFJT> active_goal_;
  trajectory_msgs::msg::JointTrajectory trajectory_;
  rclcpp::Time start_time_;
};

} // namespace g1_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(g1_hardware::GearSonicController)
