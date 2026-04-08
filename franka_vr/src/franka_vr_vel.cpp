#include <atomic>
#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <mutex>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/utils/logger.hpp>
#include "franka_vr/srv/set_step_size.hpp"
#include "franka_vr/srv/set_target_pose.hpp"
#include <Eigen/Geometry>

using namespace moveit_servo;

namespace
{
constexpr auto K_BASE_FRAME = "fr3_link0";
constexpr auto K_TIP_FRAME = "fr3_hand";
constexpr double POSITION_TOLERANCE = 0.002;     // 位置容差（米）
constexpr double ORIENTATION_TOLERANCE = 0.05;  // 角度容差（弧度）
constexpr double LINEAR_VEL_HARD_LIMIT = 0.52;   // m/s
constexpr double ANGULAR_VEL_HARD_LIMIT = 1.28;  // rad/s
constexpr double TARGET_STALE_TIMEOUT_S = 0.20;  // 超时后清零，防止陈旧目标持续积分
}  // namespace

static Eigen::Vector3d linear_step_size{ 0.39, 0.39, 0.39 };
static Eigen::AngleAxisd x_step_size(1.05, Eigen::Vector3d::UnitX());
static Eigen::AngleAxisd y_step_size(1.05, Eigen::Vector3d::UnitY());
static Eigen::AngleAxisd z_step_size(1.05, Eigen::Vector3d::UnitZ());
static geometry_msgs::msg::Pose target_pose;
static bool target_set = false;
static uint64_t target_sequence = 0;
static int64_t target_update_time_ns = 0;
static uint64_t target_topic_message_count = 0;
static geometry_msgs::msg::Pose previous_target_pose;
static int64_t previous_target_update_time_ns = 0;
static bool have_previous_target_pose = false;
static std::mutex target_pose_guard;
static moveit_msgs::msg::ServoStatus latest_servo_status;
static bool servo_status_received = false;
static uint64_t servo_status_message_count = 0;
static std::mutex servo_status_guard;

double compute_shortest_orientation_error(const geometry_msgs::msg::Pose& previous,
                                          const geometry_msgs::msg::Pose& current,
                                          tf2::Quaternion& q_prev,
                                          tf2::Quaternion& q_curr)
{
  tf2::fromMsg(previous.orientation, q_prev);
  tf2::fromMsg(current.orientation, q_curr);
  q_prev.normalize();
  q_curr.normalize();

  if (q_prev.dot(q_curr) < 0.0)
  {
    q_curr = tf2::Quaternion(-q_curr.x(), -q_curr.y(), -q_curr.z(), -q_curr.w());
  }

  return q_prev.angleShortestPath(q_curr);
}

Eigen::Vector3d clamp_vector_norm(const Eigen::Vector3d& vector, double max_norm)
{
  if (max_norm <= 0.0)
  {
    return Eigen::Vector3d::Zero();
  }

  const double norm = vector.norm();
  if (norm <= max_norm || norm < 1e-9)
  {
    return vector;
  }

  return vector * (max_norm / norm);
}

Eigen::Isometry3d pose_msg_to_eigen(const geometry_msgs::msg::Pose& pose_msg)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() =
      Eigen::Vector3d(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);

  Eigen::Quaterniond quat(pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y,
                          pose_msg.orientation.z);
  if (quat.norm() < 1e-8)
  {
    quat = Eigen::Quaterniond::Identity();
  }
  else
  {
    quat.normalize();
  }
  pose.linear() = quat.toRotationMatrix();
  return pose;
}

Eigen::Vector3d compute_orientation_error_vector(const geometry_msgs::msg::Pose& current,
                                                 const geometry_msgs::msg::Pose& target)
{
  Eigen::Quaterniond q_current(current.orientation.w, current.orientation.x, current.orientation.y,
                               current.orientation.z);
  Eigen::Quaterniond q_target(target.orientation.w, target.orientation.x, target.orientation.y,
                              target.orientation.z);

  if (q_current.norm() < 1e-8)
  {
    q_current = Eigen::Quaterniond::Identity();
  }
  else
  {
    q_current.normalize();
  }

  if (q_target.norm() < 1e-8)
  {
    q_target = Eigen::Quaterniond::Identity();
  }
  else
  {
    q_target.normalize();
  }

  if (q_current.dot(q_target) < 0.0)
  {
    q_target.coeffs() *= -1.0;
  }

  const Eigen::Quaterniond q_error = q_target * q_current.inverse();
  Eigen::AngleAxisd angle_axis(q_error);
  if (std::abs(angle_axis.angle()) < 1e-9 || !std::isfinite(angle_axis.angle()))
  {
    return Eigen::Vector3d::Zero();
  }

  return angle_axis.axis() * angle_axis.angle();
}

void set_step_size_callback(const rclcpp::Node::SharedPtr& node,
                            const std::shared_ptr<franka_vr::srv::SetStepSize::Request> request,
                            std::shared_ptr<franka_vr::srv::SetStepSize::Response> response)
{
  try
  {
    linear_step_size = Eigen::Vector3d{ request->linear_x, request->linear_y, request->linear_z };
    x_step_size = Eigen::AngleAxisd(request->angular_x, Eigen::Vector3d::UnitX());
    y_step_size = Eigen::AngleAxisd(request->angular_y, Eigen::Vector3d::UnitY());
    z_step_size = Eigen::AngleAxisd(request->angular_z, Eigen::Vector3d::UnitZ());
    response->success = true;
    response->message = "Step sizes updated successfully";
    RCLCPP_INFO(node->get_logger(), "Updated step sizes: linear[%.3f, %.3f, %.3f], angular[%.3f, %.3f, %.3f]",
                linear_step_size.x(), linear_step_size.y(), linear_step_size.z(), x_step_size.angle(),
                y_step_size.angle(), z_step_size.angle());
  }
  catch (const std::exception& e)
  {
    response->success = false;
    response->message = std::string("Failed to update step sizes: ") + e.what();
    RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
  }
}

void set_target_pose_callback(const rclcpp::Node::SharedPtr& node,
                              const std::shared_ptr<franka_vr::srv::SetTargetPose::Request> request,
                              std::shared_ptr<franka_vr::srv::SetTargetPose::Response> response)
{
  std::lock_guard<std::mutex> lock(target_pose_guard);
  target_pose = request->target_pose;
  target_set = true;
  ++target_sequence;
  target_update_time_ns = node->get_clock()->now().nanoseconds();
  response->success = true;
  response->message = "Target pose set successfully";
  RCLCPP_INFO(node->get_logger(), "Target pose set: [x: %.3f, y: %.3f, z: %.3f]", target_pose.position.x,
              target_pose.position.y, target_pose.position.z);
}

void set_target_pose_topic_callback(const rclcpp::Node::SharedPtr& node,
                                    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(target_pose_guard);
  const int64_t update_time_ns = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
                                     node->get_clock()->now().nanoseconds() :
                                     rclcpp::Time(msg->header.stamp).nanoseconds();

  double input_dt_s = 0.0;
  double input_dpos_norm_m = 0.0;
  double input_dtheta_rad = 0.0;
  if (have_previous_target_pose && previous_target_update_time_ns > 0 && update_time_ns > previous_target_update_time_ns)
  {
    input_dt_s = (update_time_ns - previous_target_update_time_ns) * 1e-9;
    const Eigen::Vector3d delta_position(
        msg->pose.position.x - previous_target_pose.position.x,
        msg->pose.position.y - previous_target_pose.position.y,
        msg->pose.position.z - previous_target_pose.position.z);
    input_dpos_norm_m = delta_position.norm();
    tf2::Quaternion q_prev, q_curr;
    input_dtheta_rad =
        compute_shortest_orientation_error(previous_target_pose, msg->pose, q_prev, q_curr);
  }

  target_pose = msg->pose;
  target_set = true;
  ++target_sequence;
  ++target_topic_message_count;
  target_update_time_ns = update_time_ns;
  previous_target_pose = msg->pose;
  previous_target_update_time_ns = update_time_ns;
  have_previous_target_pose = true;
  RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                       "Received target_pose messages=%llu seq=%llu pos=[%.3f, %.3f, %.3f]",
                       static_cast<unsigned long long>(target_topic_message_count),
                       static_cast<unsigned long long>(target_sequence),
                       target_pose.position.x, target_pose.position.y, target_pose.position.z);
  if (have_previous_target_pose && input_dt_s > 0.0)
  {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                         "Input diag: dt=%.4f s, dpos=%.4f m, dtheta=%.4f rad",
                         input_dt_s, input_dpos_norm_m, input_dtheta_rad);
  }
}

void servo_status_callback(const rclcpp::Node::SharedPtr& node,
                           const moveit_msgs::msg::ServoStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(servo_status_guard);
  const bool code_changed = !servo_status_received || msg->code != latest_servo_status.code;
  latest_servo_status = *msg;
  servo_status_received = true;
  ++servo_status_message_count;

  if (code_changed || msg->code != moveit_msgs::msg::ServoStatus::NO_WARNING)
  {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                         "Servo status topic: code=%d message_count=%llu message='%s'",
                         static_cast<int>(msg->code),
                         static_cast<unsigned long long>(servo_status_message_count),
                         msg->message.c_str());
  }
}

Eigen::Vector3d compute_position_error(const geometry_msgs::msg::Pose& current, const geometry_msgs::msg::Pose& target)
{
  return Eigen::Vector3d(target.position.x - current.position.x, target.position.y - current.position.y,
                         target.position.z - current.position.z);
}

double compute_orientation_error(const geometry_msgs::msg::Pose& current, const geometry_msgs::msg::Pose& target)
{
  tf2::Quaternion q_current, q_target;
  tf2::fromMsg(current.orientation, q_current);
  tf2::fromMsg(target.orientation, q_target);
  return q_current.angleShortestPath(q_target);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");
  moveit::setNodeLoggerName(demo_node->get_name());

  auto step_size_service = demo_node->create_service<franka_vr::srv::SetStepSize>(
      "set_step_size", std::bind(&set_step_size_callback, demo_node, std::placeholders::_1, std::placeholders::_2));
  auto target_pose_service = demo_node->create_service<franka_vr::srv::SetTargetPose>(
      "set_target_pose", std::bind(&set_target_pose_callback, demo_node, std::placeholders::_1, std::placeholders::_2));
  auto target_pose_subscription = demo_node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_pose", rclcpp::SystemDefaultsQoS(),
      [demo_node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        set_target_pose_topic_callback(demo_node, msg);
      });

  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  auto servo_status_subscription = demo_node->create_subscription<moveit_msgs::msg::ServoStatus>(
      servo_params.status_topic, rclcpp::SystemDefaultsQoS(),
      [demo_node](const moveit_msgs::msg::ServoStatus::SharedPtr msg) {
        servo_status_callback(demo_node, msg);
      });

  auto trajectory_outgoing_cmd_pub = demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params.command_out_topic, rclcpp::SystemDefaultsQoS());

  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(demo_node, servo_params);
  Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

  std::this_thread::sleep_for(std::chrono::seconds(3));

  auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params.move_group_name);

  servo.setCommandType(CommandType::TWIST);

  std::deque<KinematicState> joint_cmd_rolling_window;
  KinematicState current_state = servo.getCurrentRobotState(true);
  updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());
  servo.resetSmoothing(current_state);

  auto pose_tracker = [&]() {
    KinematicState joint_state;
    bool was_tracking_active = false;
    uint64_t last_seen_target_sequence = 0;
    uint64_t trajectory_publish_count = 0;
    Eigen::VectorXd last_published_positions;
    rclcpp::WallRate tracking_rate(1 / servo_params.publish_period);
    std::deque<KinematicState> joint_cmd_rolling_window;
    KinematicState current_state = servo.getCurrentRobotState(true);
    updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());
    servo.resetSmoothing(current_state);

    while (rclcpp::ok())
    {
      geometry_msgs::msg::Pose local_target_pose;
      bool local_target_set = false;
      uint64_t local_target_sequence = 0;
      int64_t local_target_update_time_ns = 0;
      {
        std::lock_guard<std::mutex> lock(target_pose_guard);
        local_target_set = target_set;
        if (local_target_set)
        {
          local_target_pose = target_pose;
          local_target_sequence = target_sequence;
          local_target_update_time_ns = target_update_time_ns;
        }
      }

      if (local_target_set && local_target_sequence != last_seen_target_sequence)
      {
        RCLCPP_INFO_THROTTLE(demo_node->get_logger(), *demo_node->get_clock(), 1000,
                             "Tracker saw new target sequence=%llu update_time_ns=%lld",
                             static_cast<unsigned long long>(local_target_sequence),
                             static_cast<long long>(local_target_update_time_ns));
        last_seen_target_sequence = local_target_sequence;
      }

      const bool target_is_fresh =
          local_target_set && local_target_update_time_ns > 0 &&
          ((demo_node->now().nanoseconds() - local_target_update_time_ns) * 1e-9 <= TARGET_STALE_TIMEOUT_S);

      if (local_target_set && !target_is_fresh)
      {
        {
          std::lock_guard<std::mutex> lock(target_pose_guard);
          target_set = false;
        }
        local_target_set = false;
      }

      if (!local_target_set)
      {
        if (was_tracking_active)
        {
          current_state = servo.getCurrentRobotState(false);
          servo.resetSmoothing(current_state);
          was_tracking_active = false;
          RCLCPP_INFO_THROTTLE(demo_node->get_logger(), *demo_node->get_clock(), 1000,
                               "Target stream inactive; pose tracking paused");
        }
        tracking_rate.sleep();
        continue;
      }

      auto actual_robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
      if (!actual_robot_state)
      {
        tracking_rate.sleep();
        continue;
      }

      const geometry_msgs::msg::Pose current_pose =
          tf2::toMsg(actual_robot_state->getGlobalLinkTransform(K_TIP_FRAME));
      const Eigen::Vector3d position_error = compute_position_error(current_pose, local_target_pose);
      const Eigen::Vector3d orientation_error = compute_orientation_error_vector(current_pose, local_target_pose);
      const double position_error_norm = position_error.norm();
      const double orientation_error_norm = orientation_error.norm();

      const Eigen::Vector3d linear_gain = linear_step_size.cwiseAbs();
      const Eigen::Vector3d angular_gain(std::abs(x_step_size.angle()), std::abs(y_step_size.angle()),
                                         std::abs(z_step_size.angle()));

      Eigen::Vector3d linear_vel =
          clamp_vector_norm(linear_gain.cwiseProduct(position_error), LINEAR_VEL_HARD_LIMIT);
      Eigen::Vector3d angular_vel =
          clamp_vector_norm(angular_gain.cwiseProduct(orientation_error), ANGULAR_VEL_HARD_LIMIT);

      if (position_error_norm < POSITION_TOLERANCE)
      {
        linear_vel.setZero();
      }
      if (orientation_error_norm < ORIENTATION_TOLERANCE)
      {
        angular_vel.setZero();
      }

      TwistCommand target_twist{ K_BASE_FRAME,
                                 { linear_vel.x(), linear_vel.y(), linear_vel.z(), angular_vel.x(), angular_vel.y(),
                                   angular_vel.z() } };

      joint_state = servo.getNextJointState(robot_state, target_twist);
      StatusCode status = servo.getStatus();

      if (status != StatusCode::INVALID)
      {
        updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());
        if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
        {
          trajectory_outgoing_cmd_pub->publish(msg.value());
          ++trajectory_publish_count;
          double joint_pos_step_norm = 0.0;
          if (last_published_positions.size() == joint_state.positions.size())
          {
            joint_pos_step_norm = (joint_state.positions - last_published_positions).norm();
          }
          last_published_positions = joint_state.positions;

          RCLCPP_INFO_THROTTLE(demo_node->get_logger(), *demo_node->get_clock(), 1000,
                               "Published joint trajectory count=%llu points=%zu target_pos=[%.3f, %.3f, %.3f]",
                               static_cast<unsigned long long>(trajectory_publish_count), msg->points.size(),
                               local_target_pose.position.x, local_target_pose.position.y, local_target_pose.position.z);

          moveit_msgs::msg::ServoStatus local_servo_status;
          bool local_servo_status_received = false;
          {
            std::lock_guard<std::mutex> servo_status_lock(servo_status_guard);
            local_servo_status = latest_servo_status;
            local_servo_status_received = servo_status_received;
          }

          RCLCPP_INFO_THROTTLE(
              demo_node->get_logger(), *demo_node->get_clock(), 1000,
              "Tracking diag: pos_err=%.4f m, rot_err=%.4f rad, cmd_lin=%.4f m/s, cmd_ang=%.4f rad/s, joint_vel_norm=%.4f, joint_step_norm=%.4f, local_status=%s%s%s",
              position_error_norm, orientation_error_norm, linear_vel.norm(), angular_vel.norm(),
              joint_state.velocities.norm(), joint_pos_step_norm, servo.getStatusMessage().c_str(),
              local_servo_status_received ? " | topic_status=" : "",
              local_servo_status_received ? local_servo_status.message.c_str() : "");
        }
        if (!joint_cmd_rolling_window.empty())
        {
          robot_state->setJointGroupPositions(joint_model_group, joint_cmd_rolling_window.back().positions);
          robot_state->setJointGroupVelocities(joint_model_group, joint_cmd_rolling_window.back().velocities);
        }
        was_tracking_active = true;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(demo_node->get_logger(), *demo_node->get_clock(), 1000,
                             "Servo rejected twist target: pos_err=%.4f rot_err=%.4f cmd_lin=%.4f cmd_ang=%.4f",
                             position_error_norm, orientation_error_norm, linear_vel.norm(), angular_vel.norm());
      }

      if (status != StatusCode::NO_WARNING && status != StatusCode::INVALID)
      {
        const auto status_it = SERVO_STATUS_CODE_MAP.find(status);
        const std::string status_text =
            status_it != SERVO_STATUS_CODE_MAP.end() ? status_it->second : "Unknown servo status";
        RCLCPP_WARN_THROTTLE(demo_node->get_logger(), *demo_node->get_clock(), 1000,
                             "Servo warning: %s | pos_err=%.4f rot_err=%.4f cmd_lin=%.4f cmd_ang=%.4f",
                             status_text.c_str(), position_error_norm, orientation_error_norm, linear_vel.norm(),
                             angular_vel.norm());
      }

      tracking_rate.sleep();
    }
  };

  std::thread tracker_thread(pose_tracker);
  RCLCPP_INFO_STREAM(demo_node->get_logger(), servo.getStatusMessage());
  rclcpp::spin(demo_node);

  if (tracker_thread.joinable())
    tracker_thread.join();

  rclcpp::shutdown();
  return 0;
}
