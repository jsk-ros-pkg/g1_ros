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

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <gear_sonic_interfaces/msg/smpl_motion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <zmq.hpp>

namespace g1_hardware
{

/**
 * ROS 2 node that bridges ROS commands to the SONIC deploy stack via ZMQ.
 * Mirrors pico_manager StreamMode::PLANNER_VR_3PT: locomotion commands and
 * VR 3-point upper-body targets are combined in a single planner message,
 * so walking and arm control work simultaneously (unlike the PICO UI which
 * disables one while the other is active).
 *
 * ── Services ──────────────────────────────────────────────────────────────
 *   ~/enable_control  (std_srvs/SetBool)
 *       true  → (1) send planner{IDLE, zero vel} first, then command{start=1, planner=1}
 *               Mirrors pico_manager order: planner message before start command, so the
 *               deploy stack has planner data when entering CONTROL (prevents bad initial pose).
 *               Also resets heading and starts the timer.
 *       false → stop sending planner messages only (no stop=1 command).
 *               Deploy reverts to IDLE after its 1-second planner timeout; robot stays balanced.
 *
 *   ~/mode/default    (std_srvs/Trigger) — auto-select gait from cmd_vel magnitude (default)
 *       mag < 0.05       → IDLE
 *       0.05 ≤ mag < 0.6 → SLOW_WALK
 *       0.6  ≤ mag < 1.5 → WALK  (speed auto)
 *       mag ≥ 1.5        → RUN
 *
 *   ~/mode/<name>     (std_srvs/Trigger) — fix a specific locomotion mode
 *       idle_squat, idle_kneel_two_legs, idle_kneel, idle_lying_face_down,
 *       crawling, idle_boxing, walk_boxing, left_punch, right_punch,
 *       random_punch, elbow_crawling, left_hook, right_hook, forward_jump,
 *       stealth_walk, injured_walk
 *
 * ── Topics ────────────────────────────────────────────────────────────────
 *   /cmd_vel      (geometry_msgs/Twist, subscribe) — ROS standard, absolute topic
 *       linear.x  → forward speed  (m/s, robot body frame)
 *       linear.y  → strafe speed   (m/s, robot body frame, positive = left)
 *       angular.z → turn rate      (rad/s); accumulated into heading angle
 *       speed is always -1 (auto) as per SONIC's default
 *
 *   ~/target_height (std_msgs/Float64, subscribe)
 *       Desired base height in meters, sent as the planner `height` field
 *       (standing default is 0.789 m; lower values make the policy crouch).
 *       -1.0 = mode default (initial state). The last received value is
 *       latched until a new one arrives.
 *
 *   ~/target_left_wrist_yaw_link  (geometry_msgs/PoseStamped, subscribe)
 *   ~/target_right_wrist_yaw_link (geometry_msgs/PoseStamped, subscribe)
 *   ~/target_torso_link           (geometry_msgs/PoseStamped, subscribe)
 *       VR 3-point upper-body targets (link names from G1 URDF).
 *       Sending any one topic enables VR tracking; unpublished endpoints
 *       hold their default values (natural standing pose FK result).
 *       With apply_vr_3point_body_offset (default true) the incoming poses are
 *       interpreted as URDF *link* poses and converted to the policy keypoints
 *       by adding vr_3point_body_offset (gear_sonic motion.yaml) rotated by the
 *       commanded orientation:
 *         left wrist:  pos + R(q) * ( 0.18, -0.025, 0.0 )
 *         right wrist: pos + R(q) * ( 0.18, +0.025, 0.0 )
 *         torso/head:  pos + R(q) * ( 0.0,   0.0,   0.35 )
 *       Set the parameter to false to send raw keypoint targets yourself.
 *       If no pose message arrives within pose_timeout, vr fields are dropped
 *       from planner messages (deploy releases VR tracking and reverts to the
 *       planner reference motion) — same semantics as pico_manager when the
 *       VR stream stops.
 *
 * ── Coordinate frame for PoseStamped positions ────────────────────────────
 *   Positions must be in the G1's "pelvis" TF frame (URDF root link).
 *   Formula: local_pos = inv(root_quat) * (world_pos - root_pos)
 *   This is equivalent to a tf2 transform into the "pelvis" frame:
 *     pose_pelvis = tf_buffer.transform(pose_world, "pelvis")
 *   The deploy stack uses vr_position values directly without conversion
 *   (g1_deploy_onnx_ref.cpp GatherVR3PointPosition():
 *    "The sender has already applied offsets and root normalization").
 *
 * ── Parameters ────────────────────────────────────────────────────────────
 *   zmq_port        (int,    default 5556)
 *   publish_rate    (double, default 50.0 Hz)
 *   cmd_vel_timeout (double, default 0.5 s) — cmd_vel zeroed if no message within this window
 *   pose_timeout    (double, default 0.5 s) — vr fields dropped from planner messages if no
 *                   pose target arrives within this window (<= 0 keeps the last targets forever)
 *   left_wrist_compliance  (double, default 0.0)
 *   right_wrist_compliance (double, default 0.0)
 *   head_compliance        (double, default 0.0)
 *                   End-effector tracking compliance, each 0.0-1.0; 0 = stiff
 *                   (exact tracking). Sent as the SONIC planner `vr_compliance`
 *                   field. NOTE: the deploy-side default when the field is NOT
 *                   sent is {0.5, 0.5, 0.0} (compliant wrists). We default to
 *                   stiff so commanded end-effector poses are reproduced
 *                   accurately.
 *   apply_vr_3point_body_offset (bool, default true) — see the topic section above
 *
 * gear_sonic launch (on robot):
 *   Execute gear_sonic's ./deploy.sh and ensure the ZMQ connection is available:
 *   ./deploy.sh --input-type zmq_manager --zmq-host <IP of this machine> real
 */
class GearSonicInterface : public rclcpp::Node
{
public:
  explicit GearSonicInterface(const rclcpp::NodeOptions& options);
  ~GearSonicInterface();

private:
  void OnEnableControl(std_srvs::srv::SetBool::Request::SharedPtr req,
                       std_srvs::srv::SetBool::Response::SharedPtr res);
  void OnCmdVel(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void OnLeftWrist(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void OnRightWrist(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void OnHead(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void OnUpperBodyJoints(sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void TimerCallback();

  // Full-body SMPL streaming (streamed-motion mode), used e.g. to make the robot jump.
  void OnEnableSmplStream(std_srvs::srv::SetBool::Request::SharedPtr req,
                          std_srvs::srv::SetBool::Response::SharedPtr res);
  void OnSmplMotion(gear_sonic_interfaces::msg::SmplMotion::ConstSharedPtr msg);

  // ZMQ wire-format builders (replicates gear_sonic/utils/teleop/zmq/zmq_planner_sender.py)
  static std::vector<uint8_t> BuildZmqHeader(const std::string& fields_json, int version = 1);
  static std::vector<uint8_t> BuildCommandMessage(bool start, bool stop, bool planner);
  // vr_position, vr_orientation and vr_compliance are optional (nullptr = field omitted).
  // This matches PLANNER_VR_3PT where they are None until VR data arrives.
  static std::vector<uint8_t>
  BuildPlannerMessage(int mode, std::array<float, 3> movement, std::array<float, 3> facing,
                      float speed, float height, const std::array<float, 9>* vr_position = nullptr,
                      const std::array<float, 12>* vr_orientation = nullptr,
                      const std::array<float, 3>* vr_compliance = nullptr,
                      const std::array<float, 17>* upper_body_position = nullptr,
                      const std::array<float, 17>* upper_body_velocity = nullptr);

  // Store one endpoint pose (idx: 0 = left wrist, 1 = right wrist, 2 = head/torso)
  // into vr_position_/vr_orientation_, applying the body offset if enabled.
  void StoreTargetPose(size_t idx, const geometry_msgs::msg::PoseStamped& msg);

  // One buffered frame of processed SMPL motion (deploy "pose"/streamed-motion stream).
  struct SmplFrame
  {
    std::array<float, 63> smpl_pose{};   // 21 x 3 axis-angle
    std::array<float, 72> smpl_joints{}; // 24 x 3 positions
    std::array<float, 4> body_quat_w{};  // root orientation, scalar-first (w,x,y,z)
    std::array<float, 29> joint_pos{};
    std::array<float, 29> joint_vel{};
    int64_t frame_index{0};
  };

  // Pack a sliding window of frames into a "pose" topic message (protocol v3),
  // replicating gear_sonic/utils/teleop/zmq/zmq_planner_sender.py pack_pose_message().
  static std::vector<uint8_t> BuildPoseMessage(const std::deque<SmplFrame>& frames);

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_control_srv_;
  // One Trigger service per LocomotionMode (indexed by mode value 0–19)
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> mode_services_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_height_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_wrist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_wrist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr head_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr upper_body_sub_;
  rclcpp::Subscription<gear_sonic_interfaces::msg::SmplMotion>::SharedPtr smpl_motion_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_smpl_stream_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex data_mutex_;

  // Locomotion state
  int locomotion_mode_{-1};       // -1 = auto (IDLE/SLOW_WALK/WALK/RUN from cmd_vel mag)
  double cmd_vel_x_{0.0};         // forward speed  (body frame, m/s)
  double cmd_vel_y_{0.0};         // strafe speed   (body frame, m/s, positive = left)
  double cmd_angular_z_{0.0};     // turn rate      (rad/s); integrated each timer tick
  double heading_rad_{0.0};       // accumulated heading angle (rad)
  double target_height_{-1.0};    // planner `height` field; -1 = mode default
  double publish_dt_{1.0 / 50.0}; // updated in constructor from publish_rate param
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME}; // last cmd_vel receive time
  double cmd_vel_timeout_{0.5}; // seconds; cmd_vel zeroed if no message within this window

  // VR 3-point upper-body targets
  // vr_position: [Lw_x,y,z, Rw_x,y,z, H_x,y,z]  (pelvis frame)
  // Default values from gear_sonic_deploy zmq_endpoint_interface.hpp
  // (InputInterface defaults, natural standing pose FK result).
  static constexpr std::array<float, 9> kDefaultVrPosition{
      0.0903f, 0.1615f,  -0.2411f, // left  wrist
      0.1280f, -0.1522f, -0.2461f, // right wrist
      0.0241f, -0.0081f, 0.4028f,  // head
  };
  // vr_orientation: [Lw_qw,qx,qy,qz, Rw_qw,qx,qy,qz, H_qw,qx,qy,qz]
  // Default values from same source.
  static constexpr std::array<float, 12> kDefaultVrOrientation{
      0.7295f, 0.3145f,  0.5533f, -0.2506f, // left
      0.7320f, -0.2639f, 0.5395f, 0.3217f,  // right
      0.9991f, 0.011f,   0.0402f, -0.0002f, // head
  };
  // Keypoint offsets in the target link's local frame, from gear_sonic
  // config/manager_env/commands/terms/motion.yaml `vr_3point_body_offset`
  // (also mirrored in g1_deploy_onnx_ref.cpp VR_3POINT_OFFSETS). The policy
  // tracks link_pos + R(link_quat) * offset, NOT the link origin itself.
  static constexpr std::array<std::array<float, 3>, 3> kVr3PointBodyOffset{{
      {0.18f, -0.025f, 0.0f}, // left wrist
      {0.18f, +0.025f, 0.0f}, // right wrist
      {0.0f, 0.0f, 0.35f},    // head (offset from torso link)
  }};
  std::array<float, 9> vr_position_{kDefaultVrPosition};
  std::array<float, 12> vr_orientation_{kDefaultVrOrientation};
  std::array<float, 3> vr_compliance_{0.0f, 0.0f, 0.0f}; // set from parameter in ctor
  bool apply_body_offset_{true};                         // set from parameter in ctor
  bool any_pose_received_{false};                   // true once at least one pose topic arrives
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME}; // last pose target receive time
  double pose_timeout_{0.5}; // seconds; vr fields dropped if no pose within this window

  // MoveIt trajectory targets in the exact 17-DOF IsaacLab upper-body order
  // expected by SONIC:
  //   waist yaw/roll/pitch,
  //   L/R shoulder pitch, L/R shoulder roll, L/R shoulder yaw,
  //   L/R elbow, L/R wrist roll, L/R wrist pitch, L/R wrist yaw.
  // The ROS topic is name-based; OnUpperBodyJoints performs this ordering.
  std::array<float, 17> upper_body_position_{};
  std::array<float, 17> upper_body_velocity_{};
  bool any_upper_body_received_{false};
  rclcpp::Time last_upper_body_time_{0, 0, RCL_ROS_TIME};
  double upper_body_timeout_{0.5};

  // Graceful arm-lowering ramp duration in the destructor (fixed, not a parameter).
  // Gantry-suspended use is assumed; the robot goes limp after the socket closes.
  static constexpr double kShutdownSettleTime = 1.0; // seconds

  bool control_active_{false};

  // Full-body SMPL streaming state. When streaming_smpl_ is true the deploy is in
  // streamed-motion mode and the timer stops sending planner messages; pose messages are
  // sent from OnSmplMotion instead. A sliding window of frames is sent per message, matching
  // the pico_manager pose stream (deque maxlen = smpl_window_size_).
  bool streaming_smpl_{false};
  size_t smpl_window_size_{5};
  std::deque<SmplFrame> smpl_window_;

  zmq::context_t zmq_ctx_;
  // XPUB (extended PUB) instead of plain PUB: notifies us when a subscriber
  // connects (0x01 prefix) or disconnects (0x00 prefix), enabling connection monitoring.
  std::unique_ptr<zmq::socket_t> zmq_sock_;
  bool deploy_connected_{false}; // true once the deploy stack SUB connects

  static constexpr int kZmqHeaderSize = 1280;
};

} // namespace g1_hardware
