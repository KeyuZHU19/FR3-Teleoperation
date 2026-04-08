#include <atomic>
#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
constexpr double LINEAR_VEL_HARD_LIMIT = 0.28;   // m/s
constexpr double ANGULAR_VEL_HARD_LIMIT = 0.75;  // rad/s
constexpr double TARGET_STALE_TIMEOUT_S = 0.20;  // 超时后清零，防止陈旧目标持续积分
constexpr double MIN_TARGET_DT_S = 0.010;        // 避免服务/回调抖动导致的极小dt
constexpr double MAX_TARGET_DELTA_POSITION_M = 0.012;  // 单帧最大平移增量
constexpr double MAX_TARGET_DELTA_ROTATION_RAD = 0.12; // 单帧最大旋转增量
constexpr double MAX_TARGET_JUMP_POSITION_M = 0.08;    // 目标跳变阈值（欧式距离）
constexpr double MAX_TARGET_JUMP_Z_M = 0.05;           // 目标跳变阈值（Z轴）
constexpr double MAX_TARGET_JUMP_ROTATION_RAD = 0.80;  // 目标跳变阈值（姿态）
constexpr double LINEAR_ACCEL_LIMIT = 0.7;             // m/s^2
constexpr double ANGULAR_ACCEL_LIMIT = 2.0;            // rad/s^2
constexpr double ZERO_TWIST_EPS = 1e-4;
constexpr double DIAGNOSTIC_LOG_PERIOD_S = 1.0;
}  // namespace

static Eigen::Vector3d linear_step_size{ 0.20, 0.20, 0.20 };
static Eigen::AngleAxisd x_step_size(0.60, Eigen::Vector3d::UnitX());
static Eigen::AngleAxisd y_step_size(0.60, Eigen::Vector3d::UnitY());
static Eigen::AngleAxisd z_step_size(0.60, Eigen::Vector3d::UnitZ());
static geometry_msgs::msg::Pose target_pose;
static bool target_set = false;
static uint64_t target_sequence = 0;
static int64_t target_update_time_ns = 0;
static uint64_t target_topic_message_count = 0;
static std::mutex target_pose_guard;

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

Eigen::Vector3d apply_acceleration_limit(const Eigen::Vector3d& current,
                                         const Eigen::Vector3d& target,
                                         double accel_limit,
                                         double dt_s)
{
  if (dt_s <= 0.0 || accel_limit <= 0.0)
  {
    return target;
  }

  const double max_delta = accel_limit * dt_s;
  Eigen::Vector3d next = current;
  for (int i = 0; i < 3; ++i)
  {
    const double delta = std::clamp(target[i] - current[i], -max_delta, max_delta);
    next[i] += delta;
  }
  return next;
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
  target_pose = msg->pose;
  target_set = true;
  ++target_sequence;
  ++target_topic_message_count;
  target_update_time_ns = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
                              node->get_clock()->now().nanoseconds() :
                              rclcpp::Time(msg->header.stamp).nanoseconds();
  RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                       "Received target_pose messages=%llu seq=%llu pos=[%.3f, %.3f, %.3f]",
                       static_cast<unsigned long long>(target_topic_message_count),
                       static_cast<unsigned long long>(target_sequence),
                       target_pose.position.x, target_pose.position.y, target_pose.position.z);
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

  auto trajectory_outgoing_cmd_pub = demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params.command_out_topic, rclcpp::SystemDefaultsQoS());

  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(demo_node, servo_params);
  Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

  std::this_thread::sleep_for(std::chrono::seconds(3));

  auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params.move_group_name);

  servo.setCommandType(CommandType::POSE);

  std::deque<KinematicState> joint_cmd_rolling_window;
  KinematicState current_state = servo.getCurrentRobotState(true);
  updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());
  servo.resetSmoothing(current_state);

  auto pose_tracker = [&]() {
    KinematicState joint_state;
    bool was_tracking_active = false;
    uint64_t last_seen_target_sequence = 0;
    uint64_t trajectory_publish_count = 0;
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

      PoseCommand target_pose_command;
      target_pose_command.frame_id = K_BASE_FRAME;
      target_pose_command.pose = pose_msg_to_eigen(local_target_pose);

      joint_state = servo.getNextJointState(robot_state, target_pose_command);
      StatusCode status = servo.getStatus();

      if (status != StatusCode::INVALID)
      {
        updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());
        if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
        {
          trajectory_outgoing_cmd_pub->publish(msg.value());
          ++trajectory_publish_count;
          RCLCPP_INFO_THROTTLE(demo_node->get_logger(), *demo_node->get_clock(), 1000,
                               "Published joint trajectory count=%llu points=%zu target_pos=[%.3f, %.3f, %.3f]",
                               static_cast<unsigned long long>(trajectory_publish_count), msg->points.size(),
                               local_target_pose.position.x, local_target_pose.position.y, local_target_pose.position.z);
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
                             "Servo rejected pose target: pos=[%.3f, %.3f, %.3f]",
                             local_target_pose.position.x, local_target_pose.position.y, local_target_pose.position.z);
      }

      if (status != StatusCode::NO_WARNING && status != StatusCode::INVALID)
      {
        const auto status_it = SERVO_STATUS_CODE_MAP.find(status);
        const std::string status_text =
            status_it != SERVO_STATUS_CODE_MAP.end() ? status_it->second : "Unknown servo status";
        RCLCPP_WARN_THROTTLE(demo_node->get_logger(), *demo_node->get_clock(), 1000,
                             "Servo warning: %s | pose target=[%.3f, %.3f, %.3f]",
                             status_text.c_str(), local_target_pose.position.x, local_target_pose.position.y,
                             local_target_pose.position.z);
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
