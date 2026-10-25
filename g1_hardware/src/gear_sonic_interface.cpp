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

#include "g1_hardware/gear_sonic_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

#include <rclcpp_components/register_node_macro.hpp>

namespace g1_hardware
{

// LocomotionMode enum values and their service name suffixes.
// Source: gear_sonic/scripts/pico_manager_thread_server.py
//         class LocomotionMode(IntEnum)
//
// Modes 0-3 (IDLE, SLOW_WALK, WALK, RUN) are NOT exposed as individual services.
// Instead ~/mode/default enables auto-selection based on cmd_vel magnitude:
//   mag < 0.05  → IDLE (0)
//   0.05-1.5    → WALK (2)  pico uses speed=-1 (auto)
//   mag >= 1.5  → RUN  (3)  pico minimum 1.5 m/s
// SLOW_WALK is skipped to avoid SLOW_WALK↔WALK oscillation at the boundary.
static const std::vector<std::pair<int, std::string>> kLocomotionModes = {
    {4, "idle_squat"},      {5, "idle_kneel_two_legs"},
    {6, "idle_kneel"},      {7, "idle_lying_face_down"},
    {8, "crawling"},        {9, "idle_boxing"},
    {10, "walk_boxing"},    {11, "left_punch"},
    {12, "right_punch"},    {13, "random_punch"},
    {14, "elbow_crawling"}, {15, "left_hook"},
    {16, "right_hook"},     {17, "forward_jump"},
    {18, "stealth_walk"},   {19, "injured_walk"},
};

static constexpr int kAutoMode = -1; // auto: IDLE/SLOW_WALK/WALK/RUN selected by cmd_vel mag

GearSonicInterface::GearSonicInterface(const rclcpp::NodeOptions& options)
: rclcpp::Node("gear_sonic_interface", options)
{
  const std::string zmq_host = declare_parameter("zmq_host", "0.0.0.0");
  const int zmq_port = declare_parameter("zmq_port", 5556);
  const double publish_rate = declare_parameter("publish_rate", 50.0);
  cmd_vel_timeout_ = declare_parameter("cmd_vel_timeout", 0.5);
  pose_timeout_ = declare_parameter("pose_timeout", 0.5);
  upper_body_timeout_ = declare_parameter("upper_body_timeout", 0.5);
  smpl_window_size_ = static_cast<size_t>(declare_parameter("smpl_window_size", 5));
  apply_body_offset_ = declare_parameter("apply_vr_3point_body_offset", true);
  // End-effector tracking compliance per keypoint, each 0.0-1.0; 0 = stiff.
  // Sent as the SONIC planner `vr_compliance` field. The deploy-side default
  // when the field is not sent is {0.5, 0.5, 0.0} (compliant wrists) — we
  // default to stiff so commanded poses are reproduced accurately.
  vr_compliance_[0] = static_cast<float>(declare_parameter("left_wrist_compliance", 0.0));
  vr_compliance_[1] = static_cast<float>(declare_parameter("right_wrist_compliance", 0.0));
  vr_compliance_[2] = static_cast<float>(declare_parameter("head_compliance", 0.0));
  publish_dt_ = 1.0 / publish_rate;

  // ZMQ: bind XPUB socket so the deploy stack SUB can connect to us.
  // XPUB = extended PUB: identical to PUB for sending, but additionally delivers
  // subscription frames when a subscriber connects (0x01 + filter) or disconnects
  // (0x00 + filter).  We poll these in TimerCallback to track deploy_connected_.
  zmq_sock_ = std::make_unique<zmq::socket_t>(zmq_ctx_, zmq::socket_type::xpub);
  const std::string addr = "tcp://" + zmq_host + ":" + std::to_string(zmq_port);
  RCLCPP_INFO(get_logger(),
              "Bound ZMQ XPUB to %s. "
              "Execute gear_sonic's ./deploy.sh and ensure the ZMQ connection is available.",
              addr.c_str());
  zmq_sock_->bind(addr);

  // ── Services ────────────────────────────────────────────────────────────

  enable_control_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable_control",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) { OnEnableControl(req, res); });

  // ~/mode/default: auto-select IDLE/SLOW_WALK/WALK/RUN from cmd_vel magnitude
  mode_services_.push_back(create_service<std_srvs::srv::Trigger>(
      "~/mode/default", [this](std_srvs::srv::Trigger::Request::SharedPtr,
                               std_srvs::srv::Trigger::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        locomotion_mode_ = kAutoMode;
        res->success = true;
        RCLCPP_INFO(get_logger(), "Locomotion mode → default (auto from cmd_vel)");
      }));

  // One Trigger service per non-locomotion mode: ~/mode/<name>
  for (const auto& [mode_val, mode_name] : kLocomotionModes) {
    const int mv = mode_val;
    auto srv = create_service<std_srvs::srv::Trigger>(
        "~/mode/" + mode_name, [this, mv](std_srvs::srv::Trigger::Request::SharedPtr,
                                          std_srvs::srv::Trigger::Response::SharedPtr res) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          locomotion_mode_ = mv;
          res->success = true;
          RCLCPP_INFO(get_logger(), "Locomotion mode → %d", mv);
        });
    mode_services_.push_back(srv);
  }

  // ── Subscriptions ────────────────────────────────────────────────────────

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { OnCmdVel(msg); });

  // Desired base height (m); forwarded as the planner `height` field.
  // Standing default is 0.789 m (kplanner config default_height); -1 = mode default.
  target_height_sub_ = create_subscription<std_msgs::msg::Float64>(
      "~/target_height", 1, [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (msg->data != target_height_) {
          RCLCPP_INFO(get_logger(), "Target base height → %.3f m", msg->data);
        }
        target_height_ = msg->data;
      });

  left_wrist_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_left_wrist_yaw_link", 1,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { OnLeftWrist(msg); });
  right_wrist_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_right_wrist_yaw_link", 1,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { OnRightWrist(msg); });
  head_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_torso_link", 1,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { OnHead(msg); });
  upper_body_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "~/target_upper_body_joints", 1,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { OnUpperBodyJoints(msg); });

  // Full-body SMPL streaming (streamed-motion mode), used e.g. for jumps.
  enable_smpl_stream_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable_smpl_stream",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) { OnEnableSmplStream(req, res); });
  smpl_motion_sub_ = create_subscription<gear_sonic_interfaces::msg::SmplMotion>(
      "~/smpl_motion", 10,
      [this](gear_sonic_interfaces::msg::SmplMotion::ConstSharedPtr msg) { OnSmplMotion(msg); });

  // Timer is always spinning; does nothing until control_active_ is set
  const auto period = std::chrono::duration<double>(publish_dt_);
  timer_ = create_wall_timer(period, [this]() { TimerCallback(); });

  // "Ready" is logged later, once the gear_sonic ZMQ connection is established.
}

GearSonicInterface::~GearSonicInterface()
{
  if (!zmq_sock_) {
    return;
  }

  // Stop the timer so it cannot send competing planner messages while we settle.
  if (timer_) {
    timer_->cancel();
  }

  // Graceful settle: if the deploy is under VR control, ramp the VR targets from
  // the last commanded pose to the natural rest pose over shutdown_settle_time_
  // so SONIC lowers the arms *smoothly* instead of snapping to the reference pose
  // when tracking is released. Plain C++ loop (no rclcpp time/logging) because
  // this runs during node teardown. Gantry-suspended use is assumed — the robot
  // goes limp once stop is sent / the socket closes below.
  if (control_active_ && deploy_connected_ && any_pose_received_ && kShutdownSettleTime > 0.0) {
    const std::array<float, 9> start_pos = vr_position_;
    const std::array<float, 12> start_orn = vr_orientation_;
    const int steps = std::max(1, static_cast<int>(std::lround(kShutdownSettleTime / publish_dt_)));

    auto normalize_quat = [](float& w, float& x, float& y, float& z) {
      const float n = std::sqrt(w * w + x * x + y * y + z * z);
      if (n > 1e-6f) {
        w /= n;
        x /= n;
        y /= n;
        z /= n;
      }
    };

    for (int i = 1; i <= steps; ++i) {
      const float a = static_cast<float>(i) / static_cast<float>(steps);
      std::array<float, 9> pos;
      std::array<float, 12> orn;
      for (size_t j = 0; j < 9; ++j) {
        pos[j] = start_pos[j] + a * (kDefaultVrPosition[j] - start_pos[j]);
      }
      // Per-endpoint quaternion nlerp (adequate for a ~1 s settle before damping).
      for (size_t k = 0; k < 3; ++k) {
        const size_t b = k * 4;
        for (size_t j = 0; j < 4; ++j) {
          orn[b + j] = start_orn[b + j] + a * (kDefaultVrOrientation[b + j] - start_orn[b + j]);
        }
        normalize_quat(orn[b + 0], orn[b + 1], orn[b + 2], orn[b + 3]);
      }
      const auto msg =
          BuildPlannerMessage(/*mode=*/0, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
                              /*speed=*/-1.0f, /*height=*/-1.0f, &pos, &orn, &vr_compliance_);
      zmq_sock_->send(zmq::buffer(msg), zmq::send_flags::dontwait);
      std::this_thread::sleep_for(std::chrono::duration<double>(publish_dt_));
    }
  }

  // Damping stop, then close (both make the robot go limp; sending stop is the
  // deterministic path). Arms are already at the rest pose from the settle above.
  const auto stop_msg = BuildCommandMessage(/*start=*/false, /*stop=*/true, /*planner=*/false);
  zmq_sock_->send(zmq::buffer(stop_msg), zmq::send_flags::none);
  zmq_sock_->close();
}

// ──────────────────────────────────────────────
// Services
// ──────────────────────────────────────────────

void GearSonicInterface::OnEnableControl(std_srvs::srv::SetBool::Request::SharedPtr req,
                                         std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (req->data) {
    if (!deploy_connected_) {
      RCLCPP_ERROR(get_logger(),
                   "enable_control(true) rejected: gear_sonic is not connected via ZMQ. "
                   "Execute gear_sonic's ./deploy.sh and ensure the ZMQ connection is available.");
      res->success = false;
      res->message = "gear_sonic ZMQ connection not available";
      return;
    }
    // Mirrors the pico_manager manager loop order
    // (gear_sonic/scripts/pico_manager_thread_server.py, manager main loop):
    //   run_once() sends a planner message FIRST, then the start command is sent AFTER.
    // This ensures the deploy stack's planner buffer is populated before it enters CONTROL,
    // preventing the policy from running with empty input and going to a bad pose.
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      heading_rad_ = 0.0;
      control_active_ = true;
    }
    // 1. Send one planner message (IDLE, zero velocity) so the deploy has data on entry
    const auto planner_msg = BuildPlannerMessage(
        /*mode=*/0, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
        /*speed=*/-1.0f, /*height=*/-1.0f);
    zmq_sock_->send(zmq::buffer(planner_msg), zmq::send_flags::none);
    // 2. Send start command — deploy transitions WAIT_FOR_CONTROL → CONTROL
    const auto cmd_msg = BuildCommandMessage(/*start=*/true, /*stop=*/false, /*planner=*/true);
    zmq_sock_->send(zmq::buffer(cmd_msg), zmq::send_flags::none);
    RCLCPP_INFO(get_logger(), "enable_control(true): SONIC is in control. "
                              "Use /cmd_vel for locomotion, ~/mode/* for mode, "
                              "~/left_wrist + ~/right_wrist + ~/head for VR 3-point tracking.");
  } else {
    // Stop sending planner messages. Do NOT send command{stop=1}:
    // stop=1 → g1_deploy_onnx_ref.cpp G1DeployOnnxRef::Stop() → CreateDampingCommand()
    //   → kp=0, kd=8 for all joints → robot goes limp under gravity.
    // The deploy's planner timeout (~1 s) reverts locomotion to IDLE while keeping balance.
    std::lock_guard<std::mutex> lock(data_mutex_);
    control_active_ = false;
    RCLCPP_INFO(get_logger(), "enable_control(false): stopped. Robot reverts to IDLE after ~1 s.");
  }
  res->success = true;
}

// ──────────────────────────────────────────────
// Subscription callbacks
// ──────────────────────────────────────────────

void GearSonicInterface::OnCmdVel(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  cmd_vel_x_ = msg->linear.x;
  cmd_vel_y_ = msg->linear.y;
  cmd_angular_z_ = msg->angular.z;
  last_cmd_vel_time_ = now();
}

// ──────────────────────────────────────────────
// Pose callbacks
//
// vr_position layout: [Lw_x,y,z, Rw_x,y,z, H_x,y,z]  (9 floats)
// vr_orientation layout: [Lw_qw,qx,qy,qz, Rw_qw,qx,qy,qz, H_qw,qx,qy,qz]  (12 floats)
// Source: gear_sonic/scripts/pico_manager_thread_server.py
//         PoseStreamer._pose_stream_common()
//           "vr_position": vr_3pt_pose[:, :3].flatten()    # [Lw, Rw, Head] xyz
//           "vr_orientation": vr_3pt_pose[:, 3:].flatten()  # [Lw, Rw, Head] wxyz
//
// Position frame: G1 "pelvis" TF frame (URDF root link = SONIC root).
// local_pos = inv(root_quat) * (world_pos - root_pos)
// = tf2 transform of the target pose into the "pelvis" TF frame.
// Source: gear_sonic_deploy g1_deploy_onnx_ref.cpp GatherVR3PointPosition():
//   "The sender (ZMQ/ROS2/planner) has already applied offsets and root normalization"
// Also matches pico_manager ThreePointPose._process_smpl_pose():
//   kp_poses[i, :3] = Rot(root_quat).inv() * (world_pos - root_pos)
//
// Quaternion order:
//   geometry_msgs: (x, y, z, w)
//   SONIC vr_orientation: (w, x, y, z) ← reordered in each callback below
// ──────────────────────────────────────────────

void GearSonicInterface::StoreTargetPose(size_t idx, const geometry_msgs::msg::PoseStamped& msg)
{
  const float qw = static_cast<float>(msg.pose.orientation.w);
  const float qx = static_cast<float>(msg.pose.orientation.x);
  const float qy = static_cast<float>(msg.pose.orientation.y);
  const float qz = static_cast<float>(msg.pose.orientation.z);
  float px = static_cast<float>(msg.pose.position.x);
  float py = static_cast<float>(msg.pose.position.y);
  float pz = static_cast<float>(msg.pose.position.z);

  if (apply_body_offset_) {
    // Convert the link pose to the policy keypoint: pos + R(quat) * offset.
    // R * v via quaternion: v + 2*qv x (qv x v + qw*v)
    const auto& o = kVr3PointBodyOffset[idx];
    const float tx = 2.0f * (qy * o[2] - qz * o[1]);
    const float ty = 2.0f * (qz * o[0] - qx * o[2]);
    const float tz = 2.0f * (qx * o[1] - qy * o[0]);
    px += o[0] + qw * tx + qy * tz - qz * ty;
    py += o[1] + qw * ty + qz * tx - qx * tz;
    pz += o[2] + qw * tz + qx * ty - qy * tx;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  vr_position_[idx * 3 + 0] = px;
  vr_position_[idx * 3 + 1] = py;
  vr_position_[idx * 3 + 2] = pz;
  vr_orientation_[idx * 4 + 0] = qw;
  vr_orientation_[idx * 4 + 1] = qx;
  vr_orientation_[idx * 4 + 2] = qy;
  vr_orientation_[idx * 4 + 3] = qz;
  any_pose_received_ = true;
  last_pose_time_ = now();
}

void GearSonicInterface::OnLeftWrist(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  StoreTargetPose(0, *msg);
}

void GearSonicInterface::OnRightWrist(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  StoreTargetPose(1, *msg);
}

void GearSonicInterface::OnHead(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  StoreTargetPose(2, *msg);
}

void GearSonicInterface::OnUpperBodyJoints(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  // SONIC's upper_body_position field is not in URDF/MuJoCo order. It is the
  // following IsaacLab upper-body order (policy_parameters.hpp). Accept a
  // name-based JointState so callers cannot silently use the wrong ordering.
  static constexpr std::array<const char*, 17> kJointNames{
      "waist_yaw_joint",          "waist_roll_joint",          "waist_pitch_joint",
      "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
      "left_shoulder_roll_joint",  "right_shoulder_roll_joint",
      "left_shoulder_yaw_joint",   "right_shoulder_yaw_joint",
      "left_elbow_joint",          "right_elbow_joint",
      "left_wrist_roll_joint",     "right_wrist_roll_joint",
      "left_wrist_pitch_joint",    "right_wrist_pitch_joint",
      "left_wrist_yaw_joint",      "right_wrist_yaw_joint"};

  std::array<float, 17> positions{};
  std::array<float, 17> velocities{};
  for (size_t target = 0; target < kJointNames.size(); ++target) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), kJointNames[target]);
    if (it == msg->name.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring incomplete upper-body target: missing joint '%s'.",
                           kJointNames[target]);
      return;
    }
    const size_t source = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (source >= msg->position.size() || !std::isfinite(msg->position[source])) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring invalid upper-body target for joint '%s'.",
                           kJointNames[target]);
      return;
    }
    positions[target] = static_cast<float>(msg->position[source]);
    if (source < msg->velocity.size() && std::isfinite(msg->velocity[source])) {
      velocities[target] = static_cast<float>(msg->velocity[source]);
    }
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  upper_body_position_ = positions;
  upper_body_velocity_ = velocities;
  any_upper_body_received_ = true;
  last_upper_body_time_ = now();
}

void GearSonicInterface::TimerCallback()
{
  // Poll XPUB subscription frames (non-blocking).
  // 0x01 prefix = subscriber connected; 0x00 prefix = subscriber disconnected.
  // Safe to call from the same thread as send().
  {
    zmq::message_t sub_msg;
    while (zmq_sock_->recv(sub_msg, zmq::recv_flags::dontwait)) {
      if (sub_msg.size() >= 1) {
        const bool connected = (static_cast<const uint8_t*>(sub_msg.data())[0] == 0x01);
        if (connected && !deploy_connected_) {
          deploy_connected_ = true;
          RCLCPP_INFO(get_logger(), "gear_sonic ZMQ connection established. "
                                    "Call ~/enable_control true to start.");
        } else if (!connected && deploy_connected_) {
          deploy_connected_ = false;
          RCLCPP_WARN(get_logger(), "gear_sonic ZMQ connection lost.");
        }
      }
    }
  }

  // While streaming full-body SMPL the deploy is in streamed-motion mode; pose messages are
  // sent from OnSmplMotion, so the timer must NOT also push planner messages.
  if (!control_active_ || streaming_smpl_) {
    return;
  }

  if (!deploy_connected_) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planner messages not sent: gear_sonic ZMQ connection not available. "
        "Execute gear_sonic's ./deploy.sh and ensure the ZMQ connection is available.");
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  // ── Pose target timeout ────────────────────────────────────────────────
  // Mirror pico_manager: when the VR pose stream stops, vr fields are no longer
  // sent, the deploy sets has_vr_3point_control_ = false and reverts to the
  // planner reference motion. Without this, the last target would be latched
  // forever and leak into later sessions.
  if (any_pose_received_ && pose_timeout_ > 0.0 &&
      (now() - last_pose_time_).seconds() > pose_timeout_)
  {
    any_pose_received_ = false;
    vr_position_ = kDefaultVrPosition;
    vr_orientation_ = kDefaultVrOrientation;
    RCLCPP_INFO(get_logger(),
                "No pose target for %.2f s: VR 3-point tracking released "
                "(deploy reverts to the planner reference motion).",
                pose_timeout_);
  }

  if (any_upper_body_received_ && upper_body_timeout_ > 0.0 &&
      (now() - last_upper_body_time_).seconds() > upper_body_timeout_)
  {
    any_upper_body_received_ = false;
    upper_body_position_.fill(0.0f);
    upper_body_velocity_.fill(0.0f);
    RCLCPP_INFO(get_logger(),
                "No upper-body joint target for %.2f s: SONIC joint-reference tracking released.",
                upper_body_timeout_);
  }

  // ── Locomotion: convert cmd_vel (body frame) to movement/facing (world frame) ──
  //
  // SONIC planner expects:
  //   movement[0,1]: velocity direction in world/global frame
  //   facing[0,1]:   unit vector of the robot's desired facing direction
  //
  // cmd_vel uses robot body frame:
  //   linear.x  = forward  (along facing direction)
  //   linear.y  = strafe   (perpendicular to facing, positive = left)
  //   angular.z = turn rate (rad/s) → integrated into heading_rad_ each tick
  //
  // Source: gear_sonic/scripts/pico_manager_thread_server.py
  //         PlannerLoop.run_once()
  //
  //   facing = self.yaw_accumulator.update(rx, self.dt)
  //     → accumulates right-stick X into a 2-D unit vector [cos(θ), sin(θ)]
  //   perp_x, perp_y = -facing[1], facing[0]
  //   rotation_facing = [[perp_x, perp_y],
  //                       [facing[0], facing[1]]]
  //   movement_global = rotation_facing @ movement_local
  //     where movement_local = [-lx, ly] (joystick → body frame)
  //
  // We map cmd_vel body frame onto the same model:
  //   angular.z → integrates into heading_rad_ (replaces yaw_accumulator)
  //   linear.x/y → movement_local forward/strafe components

  // Zero velocity if no cmd_vel has arrived within the timeout window
  // (standard ROS differential-drive behaviour).
  if (last_cmd_vel_time_.nanoseconds() > 0 &&
      (now() - last_cmd_vel_time_).seconds() > cmd_vel_timeout_)
  {
    cmd_vel_x_ = 0.0;
    cmd_vel_y_ = 0.0;
    cmd_angular_z_ = 0.0;
  }

  // Integrate angular.z into heading each timer tick (mirrors YawAccumulator.update())
  heading_rad_ += cmd_angular_z_ * publish_dt_;

  const double cx = std::cos(heading_rad_);
  const double sy = std::sin(heading_rad_);
  const std::array<float, 3> facing = {static_cast<float>(cx), static_cast<float>(sy), 0.0f};
  // movement_global = rotation_facing @ [linear.y, linear.x]  (strafe, forward)
  // = [-sin(h)*linear.y + cos(h)*linear.x,
  //    cos(h)*linear.y  + sin(h)*linear.x]  (matches PlannerLoop.run_once() movement rotation)
  const std::array<float, 3> movement = {
      static_cast<float>(cmd_vel_x_ * cx - cmd_vel_y_ * sy), // world x
      static_cast<float>(cmd_vel_x_ * sy + cmd_vel_y_ * cx), // world y
      0.0f};

  // ── Speed field ───────────────────────────────────────────────────────
  // Source: gear_sonic/scripts/pico_manager_thread_server.py
  //         PlannerLoop.run_once()
  //
  // pico_manager uses normalised joystick magnitude mag ∈ [0, 1] as follows:
  //   SLOW_WALK (1): speed = 0.1 + 0.5 * mag   (0.1 ~ 0.6 m/s)
  //   WALK      (2): speed = -1.0               (SONIC auto — no explicit speed)
  //   RUN       (3): speed = 1.5 + 3 * mag      (1.5 ~ 4.5 m/s)
  //   others       : speed = mag                (0 ~ 1.0, normalised)
  //
  // cmd_vel.linear carries actual speeds in m/s (not normalised [0,1]).
  // Mapping:
  //   WALK:      keep speed = -1.0 to match pico_manager exactly.
  //   RUN:       pass m/s directly; enforce the 1.5 m/s minimum that the run gait
  //              requires — sending below this causes "drunk" locomotion.
  //   SLOW_WALK: pass m/s directly; clamp to pico's 0.1-0.6 m/s range.
  //   others:    pass m/s magnitude directly (pico's "speed = mag" maps [0,1] →
  //              units unknown, so m/s passthrough is the best approximation).
  //   zero input: speed = -1.0 (auto) for all modes.
  const float mag =
      std::sqrt(static_cast<float>(cmd_vel_x_ * cmd_vel_x_ + cmd_vel_y_ * cmd_vel_y_));

  // ── Mode and speed selection ───────────────────────────────────────────
  // Source: gear_sonic/scripts/pico_manager_thread_server.py PlannerLoop.run_once()
  int active_mode;
  float speed;
  if (locomotion_mode_ == kAutoMode) {
    // ~/mode/default: auto-select locomotion gait from cmd_vel magnitude.
    // Source: gear_sonic/scripts/pico_manager_thread_server.py PlannerLoop.run_once()
    //   < 0.05   → IDLE      (0)  deadzone
    //   0.05-0.6 → SLOW_WALK (1)  speed = clamp(mag, 0.1, 0.6)
    //   0.6-1.5  → WALK      (2)  speed = mag
    //   >= 1.5   → RUN       (3)  speed = clamp(mag, 1.5, 4.5)
    // RUN mode is not used in auto selection: speeds above ~2.0 m/s cause instability
    // in simulation (possibly a MuJoCo floor / policy issue). Cap WALK at 2.0 m/s.
    // RUN can still be triggered explicitly via a future ~/mode/run service if needed.
    if (mag < 0.05f) {
      active_mode = 0;
      speed = -1.0f; // IDLE
    } else if (mag < 0.6f) {
      active_mode = 1;
      speed = std::max(0.1f, mag); // SLOW_WALK
    } else {
      if (mag > 2.0f) {
        RCLCPP_WARN_ONCE(get_logger(),
                         "cmd_vel magnitude %.2f m/s exceeds 2.0 m/s cap "
                         "(high speeds cause instability in simulation; "
                         "clamped to 2.0 m/s). On real hardware, increase the cap via "
                         "scene friction or remove the limit.",
                         static_cast<double>(mag));
      }
      active_mode = 2;
      speed = std::min(mag, 2.0f); // WALK, capped at 2.0 m/s
    }
  } else {
    // Explicit non-locomotion mode set via ~/mode/* service (modes 4-19)
    active_mode = locomotion_mode_;
    speed = (mag < 1e-6f) ? -1.0f : mag;
  }

  // ── Build planner message ──────────────────────────────────────────────
  // Include vr_position/vr_orientation as soon as ANY pose topic has been received.
  // Endpoints that have not been published yet keep their default values (set in the
  // header: natural standing pose in pelvis frame), so a single-topic publisher works.
  const float height = static_cast<float>(target_height_);
  const auto* vr_position = any_pose_received_ ? &vr_position_ : nullptr;
  const auto* vr_orientation = any_pose_received_ ? &vr_orientation_ : nullptr;
  const auto* vr_compliance = any_pose_received_ ? &vr_compliance_ : nullptr;
  const auto* upper_body_position =
      any_upper_body_received_ ? &upper_body_position_ : nullptr;
  const auto* upper_body_velocity =
      any_upper_body_received_ ? &upper_body_velocity_ : nullptr;
  const auto msg = BuildPlannerMessage(active_mode, movement, facing, speed, height, vr_position,
                                       vr_orientation, vr_compliance, upper_body_position,
                                       upper_body_velocity);
  zmq_sock_->send(zmq::buffer(msg), zmq::send_flags::dontwait);
}

// ──────────────────────────────────────────────
// ZMQ message builders
// Replicates gear_sonic/utils/teleop/zmq/zmq_planner_sender.py
//   build_command_message()
//   build_planner_message()
//   _build_header()   (HEADER_SIZE = 1280)
// Message layout: [topic_bytes][1280-byte JSON header][little-endian binary payload]
// ──────────────────────────────────────────────

std::vector<uint8_t> GearSonicInterface::BuildZmqHeader(const std::string& fields_json, int version)
{
  const std::string s = "{\"v\":" + std::to_string(version) +
                        ",\"endian\":\"le\",\"count\":1,\"fields\":" + fields_json + "}";
  std::vector<uint8_t> header(kZmqHeaderSize, 0);
  std::memcpy(header.data(), s.data(), std::min(s.size(), static_cast<size_t>(kZmqHeaderSize)));
  return header;
}

std::vector<uint8_t> GearSonicInterface::BuildCommandMessage(bool start, bool stop, bool planner)
{
  const std::string fields = "[{\"name\":\"start\",\"dtype\":\"u8\",\"shape\":[1]},"
                             "{\"name\":\"stop\",\"dtype\":\"u8\",\"shape\":[1]},"
                             "{\"name\":\"planner\",\"dtype\":\"u8\",\"shape\":[1]}]";
  std::vector<uint8_t> msg;
  const std::string topic = "command";
  msg.insert(msg.end(), topic.begin(), topic.end());
  const auto hdr = BuildZmqHeader(fields);
  msg.insert(msg.end(), hdr.begin(), hdr.end());
  msg.push_back(start ? 1u : 0u);
  msg.push_back(stop ? 1u : 0u);
  msg.push_back(planner ? 1u : 0u);
  return msg;
}

std::vector<uint8_t> GearSonicInterface::BuildPlannerMessage(
    int mode, std::array<float, 3> movement, std::array<float, 3> facing, float speed, float height,
    const std::array<float, 9>* vr_position, const std::array<float, 12>* vr_orientation,
    const std::array<float, 3>* vr_compliance,
    const std::array<float, 17>* upper_body_position,
    const std::array<float, 17>* upper_body_velocity)
{
  std::string fields = "[{\"name\":\"mode\",\"dtype\":\"i32\",\"shape\":[1]},"
                       "{\"name\":\"movement\",\"dtype\":\"f32\",\"shape\":[3]},"
                       "{\"name\":\"facing\",\"dtype\":\"f32\",\"shape\":[3]},"
                       "{\"name\":\"speed\",\"dtype\":\"f32\",\"shape\":[1]},"
                       "{\"name\":\"height\",\"dtype\":\"f32\",\"shape\":[1]}";
  if (vr_position) {
    fields += ",{\"name\":\"vr_position\",\"dtype\":\"f32\",\"shape\":[9]}";
  }
  if (vr_orientation) {
    fields += ",{\"name\":\"vr_orientation\",\"dtype\":\"f32\",\"shape\":[12]}";
  }
  if (vr_compliance) {
    fields += ",{\"name\":\"vr_compliance\",\"dtype\":\"f32\",\"shape\":[3]}";
  }
  if (upper_body_position) {
    fields += ",{\"name\":\"upper_body_position\",\"dtype\":\"f32\",\"shape\":[17]}";
  }
  if (upper_body_velocity) {
    fields += ",{\"name\":\"upper_body_velocity\",\"dtype\":\"f32\",\"shape\":[17]}";
  }
  fields += "]";

  auto append = [](std::vector<uint8_t>& v, const void* data, size_t n) {
    const auto* p = static_cast<const uint8_t*>(data);
    v.insert(v.end(), p, p + n);
  };

  std::vector<uint8_t> msg;
  const std::string topic = "planner";
  msg.insert(msg.end(), topic.begin(), topic.end());
  const auto hdr = BuildZmqHeader(fields);
  msg.insert(msg.end(), hdr.begin(), hdr.end());

  // All values are little-endian; x86 is natively LE so memcpy is correct
  append(msg, &mode, 4);
  for (float v : movement) {
    append(msg, &v, 4);
  }
  for (float v : facing) {
    append(msg, &v, 4);
  }
  append(msg, &speed, 4);
  append(msg, &height, 4);
  if (vr_position) {
    for (float v : *vr_position) {
      append(msg, &v, 4);
    }
  }
  if (vr_orientation) {
    for (float v : *vr_orientation) {
      append(msg, &v, 4);
    }
  }
  if (vr_compliance) {
    for (float v : *vr_compliance) {
      append(msg, &v, 4);
    }
  }
  if (upper_body_position) {
    for (float v : *upper_body_position) {
      append(msg, &v, 4);
    }
  }
  if (upper_body_velocity) {
    for (float v : *upper_body_velocity) {
      append(msg, &v, 4);
    }
  }
  return msg;
}

// ──────────────────────────────────────────────
// Full-body SMPL streaming (streamed-motion / "pose" topic)
// ──────────────────────────────────────────────

void GearSonicInterface::OnEnableSmplStream(std_srvs::srv::SetBool::Request::SharedPtr req,
                                            std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (req->data) {
    if (!deploy_connected_) {
      RCLCPP_ERROR(get_logger(),
                   "enable_smpl_stream(true) rejected: gear_sonic is not connected via ZMQ. "
                   "Execute gear_sonic's ./deploy.sh --input-type zmq_manager.");
      res->success = false;
      res->message = "gear_sonic ZMQ connection not available";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      smpl_window_.clear();
      streaming_smpl_ = true;
      control_active_ = true;
    }
    // Switch the deploy to streamed-motion mode (planner=0). The robot should already be
    // standing/balanced (call ~/enable_control true first), so the motion starts from a
    // stable pose rather than toppling.
    const auto cmd_msg = BuildCommandMessage(/*start=*/true, /*stop=*/false, /*planner=*/false);
    zmq_sock_->send(zmq::buffer(cmd_msg), zmq::send_flags::none);
    RCLCPP_INFO(get_logger(),
                "enable_smpl_stream(true): streamed-motion mode. Publish "
                "gear_sonic_interfaces/SmplMotion to ~/smpl_motion (window=%zu frames).",
                smpl_window_size_);
  } else {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      streaming_smpl_ = false;
      smpl_window_.clear();
      // Auto-resume planner/VR-3PT control so disabling the SMPL stream alone returns the
      // robot to its normal state — no separate ~/enable_control call needed.
      control_active_ = true;
    }
    // Return to planner balance/IDLE so the robot re-stabilizes instead of toppling once the
    // streamed frames stop. Planner message first, then the planner-mode command (same order
    // as OnEnableControl). The timer then resumes cmd_vel/mode/VR-3PT planner control.
    const auto planner_msg = BuildPlannerMessage(
        /*mode=*/0, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, /*speed=*/-1.0f, /*height=*/-1.0f);
    zmq_sock_->send(zmq::buffer(planner_msg), zmq::send_flags::none);
    const auto cmd_msg = BuildCommandMessage(/*start=*/true, /*stop=*/false, /*planner=*/true);
    zmq_sock_->send(zmq::buffer(cmd_msg), zmq::send_flags::none);
    RCLCPP_INFO(get_logger(),
                "enable_smpl_stream(false): back to planner/VR-3PT control (IDLE balance). "
                "No separate enable_control needed.");
  }
  res->success = true;
}

void GearSonicInterface::OnSmplMotion(gear_sonic_interfaces::msg::SmplMotion::ConstSharedPtr msg)
{
  std::vector<uint8_t> pose_msg;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!streaming_smpl_) {
      return; // ignore unless ~/enable_smpl_stream true is active
    }
    SmplFrame fr;
    for (size_t i = 0; i < fr.smpl_pose.size(); ++i) {
      fr.smpl_pose[i] = static_cast<float>(msg->smpl_pose[i]);
    }
    for (size_t i = 0; i < fr.smpl_joints.size(); ++i) {
      fr.smpl_joints[i] = static_cast<float>(msg->smpl_joints[i]);
    }
    // geometry_msgs/Quaternion is (x,y,z,w); the deploy "body_quat_w" is scalar-first (w,x,y,z).
    fr.body_quat_w[0] = static_cast<float>(msg->body_quat.w);
    fr.body_quat_w[1] = static_cast<float>(msg->body_quat.x);
    fr.body_quat_w[2] = static_cast<float>(msg->body_quat.y);
    fr.body_quat_w[3] = static_cast<float>(msg->body_quat.z);
    for (size_t i = 0; i < fr.joint_pos.size(); ++i) {
      fr.joint_pos[i] = static_cast<float>(msg->joint_pos[i]);
    }
    for (size_t i = 0; i < fr.joint_vel.size(); ++i) {
      fr.joint_vel[i] = static_cast<float>(msg->joint_vel[i]);
    }
    fr.frame_index = msg->frame_index;

    smpl_window_.push_back(fr);
    while (smpl_window_.size() > smpl_window_size_) {
      smpl_window_.pop_front();
    }
    if (smpl_window_.size() < smpl_window_size_) {
      return; // wait until the sliding window is full (mirrors pico buffer_cleared logic)
    }
    pose_msg = BuildPoseMessage(smpl_window_);
  }

  if (!deploy_connected_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "SMPL pose not sent: gear_sonic ZMQ connection not available.");
    return;
  }
  zmq_sock_->send(zmq::buffer(pose_msg), zmq::send_flags::dontwait);
}

std::vector<uint8_t> GearSonicInterface::BuildPoseMessage(const std::deque<SmplFrame>& frames)
{
  const std::string n = std::to_string(frames.size());
  // Field order MUST match the binary append order below (parser reads sequentially).
  // shapes are row-major [N, ...]; matches pack_pose_message() (protocol v3).
  const std::string fields = "[{\"name\":\"smpl_pose\",\"dtype\":\"f32\",\"shape\":[" + n +
                             ",21,3]},"
                             "{\"name\":\"smpl_joints\",\"dtype\":\"f32\",\"shape\":[" +
                             n +
                             ",24,3]},"
                             "{\"name\":\"body_quat_w\",\"dtype\":\"f32\",\"shape\":[" +
                             n +
                             ",4]},"
                             "{\"name\":\"joint_pos\",\"dtype\":\"f32\",\"shape\":[" +
                             n +
                             ",29]},"
                             "{\"name\":\"joint_vel\",\"dtype\":\"f32\",\"shape\":[" +
                             n +
                             ",29]},"
                             "{\"name\":\"frame_index\",\"dtype\":\"i64\",\"shape\":[" +
                             n + "]}]";

  auto append = [](std::vector<uint8_t>& v, const void* data, size_t nbytes) {
    const auto* p = static_cast<const uint8_t*>(data);
    v.insert(v.end(), p, p + nbytes);
  };

  std::vector<uint8_t> msg;
  const std::string topic = "pose";
  msg.insert(msg.end(), topic.begin(), topic.end());
  const auto hdr = BuildZmqHeader(fields, /*version=*/3);
  msg.insert(msg.end(), hdr.begin(), hdr.end());

  // Little-endian binary payload (x86 is natively LE), frames outer (row-major [N, ...]).
  for (const auto& f : frames) {
    for (float v : f.smpl_pose) {
      append(msg, &v, 4);
    }
  }
  for (const auto& f : frames) {
    for (float v : f.smpl_joints) {
      append(msg, &v, 4);
    }
  }
  for (const auto& f : frames) {
    for (float v : f.body_quat_w) {
      append(msg, &v, 4);
    }
  }
  for (const auto& f : frames) {
    for (float v : f.joint_pos) {
      append(msg, &v, 4);
    }
  }
  for (const auto& f : frames) {
    for (float v : f.joint_vel) {
      append(msg, &v, 4);
    }
  }
  for (const auto& f : frames) {
    const int64_t idx = f.frame_index;
    append(msg, &idx, 8);
  }
  return msg;
}

} // namespace g1_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(g1_hardware::GearSonicInterface)
