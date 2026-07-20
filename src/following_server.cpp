// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
//
// This file has been modified from the original navigation2 "main" branch
// source to build against the ROS 2 Jazzy API surface (nav2_util instead of
// nav2_ros_common), to use a self-contained FollowObject action instead of
// nav2_msgs, and to replace opennav_docking::Controller::
// computeRotateToHeadingCommand() (not available in Jazzy) with a local
// accel-limited P controller. See README.md for details.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <stdexcept>

#include "angles/angles.h"
#include "opennav_following/following_server.hpp"
#include "opennav_docking_core/docking_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace opennav_following
{

FollowingServer::FollowingServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("following_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());
}

nav2_util::CallbackReturn
FollowingServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());
  auto node = shared_from_this();

  // Declare and get all parameters
  nav2_util::declare_parameter_if_not_declared(
    node, "controller_frequency", rclcpp::ParameterValue(50.0));
  node->get_parameter("controller_frequency", controller_frequency_);
  nav2_util::declare_parameter_if_not_declared(
    node, "detection_timeout", rclcpp::ParameterValue(2.0));
  node->get_parameter("detection_timeout", detection_timeout_);
  nav2_util::declare_parameter_if_not_declared(
    node, "rotate_to_object_timeout", rclcpp::ParameterValue(10.0));
  node->get_parameter("rotate_to_object_timeout", rotate_to_object_timeout_);
  nav2_util::declare_parameter_if_not_declared(
    node, "static_object_timeout", rclcpp::ParameterValue(-1.0));
  node->get_parameter("static_object_timeout", static_object_timeout_);
  nav2_util::declare_parameter_if_not_declared(
    node, "linear_tolerance", rclcpp::ParameterValue(0.15));
  node->get_parameter("linear_tolerance", linear_tolerance_);
  nav2_util::declare_parameter_if_not_declared(
    node, "angular_tolerance", rclcpp::ParameterValue(0.15));
  node->get_parameter("angular_tolerance", angular_tolerance_);
  nav2_util::declare_parameter_if_not_declared(
    node, "max_retries", rclcpp::ParameterValue(3));
  node->get_parameter("max_retries", max_retries_);
  nav2_util::declare_parameter_if_not_declared(
    node, "base_frame", rclcpp::ParameterValue(std::string("base_link")));
  node->get_parameter("base_frame", base_frame_);
  nav2_util::declare_parameter_if_not_declared(
    node, "fixed_frame", rclcpp::ParameterValue(std::string("odom")));
  node->get_parameter("fixed_frame", fixed_frame_);
  nav2_util::declare_parameter_if_not_declared(
    node, "desired_distance", rclcpp::ParameterValue(1.0));
  node->get_parameter("desired_distance", desired_distance_);
  nav2_util::declare_parameter_if_not_declared(
    node, "skip_orientation", rclcpp::ParameterValue(true));
  node->get_parameter("skip_orientation", skip_orientation_);
  nav2_util::declare_parameter_if_not_declared(
    node, "search_by_rotating", rclcpp::ParameterValue(false));
  node->get_parameter("search_by_rotating", search_by_rotating_);
  nav2_util::declare_parameter_if_not_declared(
    node, "search_angle", rclcpp::ParameterValue(M_PI_2));
  node->get_parameter("search_angle", search_angle_);
  nav2_util::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  node->get_parameter("transform_tolerance", transform_tolerance_);
  nav2_util::declare_parameter_if_not_declared(
    node, "odom_topic", rclcpp::ParameterValue(std::string("odom")));
  node->get_parameter("odom_topic", odom_topic_);
  nav2_util::declare_parameter_if_not_declared(
    node, "odom_duration", rclcpp::ParameterValue(0.3));
  node->get_parameter("odom_duration", odom_duration_);
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.use_collision_detection", rclcpp::ParameterValue(false));
  node->get_parameter("controller.use_collision_detection", use_collision_detection_);
  nav2_util::declare_parameter_if_not_declared(
    node, "filter_coef", rclcpp::ParameterValue(0.1));
  node->get_parameter("filter_coef", filter_coef_);
  // opennav_docking::Controller does not expose a rotate-in-place command in this
  // ROS distribution, so the search rotation uses its own small accel-limited P controller.
  nav2_util::declare_parameter_if_not_declared(
    node, "rotate_angular_velocity", rclcpp::ParameterValue(0.5));
  node->get_parameter("rotate_angular_velocity", rotate_angular_velocity_);
  nav2_util::declare_parameter_if_not_declared(
    node, "rotate_angular_acceleration", rclcpp::ParameterValue(1.0));
  node->get_parameter("rotate_angular_acceleration", rotate_angular_acceleration_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  vel_publisher_ = std::make_unique<nav2_util::TwistPublisher>(
    node, "cmd_vel", rclcpp::SystemDefaultsQoS());
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create odom subscriber for backward blind docking
  odom_sub_ = std::make_unique<nav2_util::OdomSmoother>(node, odom_duration_, odom_topic_);

  // Create the action server for dynamic following
  following_action_server_ = std::make_unique<FollowingActionServer>(
    node, "follow_object",
    std::bind(&FollowingServer::followObject, this),
    nullptr, std::chrono::milliseconds(500),
    true);

  // Create the controller
  // Note: Collision detection is not supported in following server so we force it off
  // and warn if the user has it enabled (from launch file or parameter file)
  controller_ =
    std::make_unique<opennav_docking::Controller>(node, tf2_buffer_, fixed_frame_,
      base_frame_);

  if (use_collision_detection_) {
    RCLCPP_ERROR(
      get_logger(),
      "Collision detection is not supported in the following server. Please disable "
      "the controller.use_collision_detection parameter.");
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Setup filter
  filter_ = std::make_unique<opennav_docking::PoseFilter>(filter_coef_, detection_timeout_);

  // And publish the filtered pose
  filtered_dynamic_pose_pub_ =
    create_publisher<geometry_msgs::msg::PoseStamped>(
    "filtered_dynamic_pose", rclcpp::SystemDefaultsQoS());

  // Initialize static object detection variables
  static_timer_initialized_ = false;
  static_object_start_time_ = rclcpp::Time(0);

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&FollowingServer::dynamicParametersCallback, this, _1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FollowingServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());

  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_, this, true);
  vel_publisher_->on_activate();
  filtered_dynamic_pose_pub_->on_activate();
  following_action_server_->activate();

  // Create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FollowingServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  following_action_server_->deactivate();
  vel_publisher_->on_deactivate();
  filtered_dynamic_pose_pub_->on_deactivate();

  tf2_listener_.reset();

  // Destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FollowingServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  dyn_params_handler_.reset();
  tf2_buffer_.reset();
  following_action_server_.reset();
  controller_.reset();
  vel_publisher_.reset();
  filtered_dynamic_pose_pub_.reset();
  odom_sub_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FollowingServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult
FollowingServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    // If we are trying to change the parameter of a plugin we can just skip it at this point
    // as they handle parameter changes themselves and don't need to lock the mutex
    if (param_name.find('.') != std::string::npos) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (parameter.as_double() <= 0.0 &&
        (param_name == "controller_frequency" || param_name == "detection_timeout" ||
        param_name == "rotate_to_object_timeout"))
      {
        RCLCPP_WARN(
          get_logger(), "The value of parameter '%s' is incorrectly set to %f, "
          "it should be >0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_double());
        result.successful = false;
        continue;
      } else if (parameter.as_double() < 0.0 && param_name != "static_object_timeout") {
        RCLCPP_WARN(
          get_logger(), "The value of parameter '%s' is incorrectly set to %f, "
          "it should be >=0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_double());
        result.successful = false;
        continue;
      }

      if (param_name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (param_name == "detection_timeout") {
        detection_timeout_ = parameter.as_double();
      } else if (param_name == "rotate_to_object_timeout") {
        rotate_to_object_timeout_ = parameter.as_double();
      } else if (param_name == "static_object_timeout") {
        static_object_timeout_ = parameter.as_double();
      } else if (param_name == "linear_tolerance") {
        linear_tolerance_ = parameter.as_double();
      } else if (param_name == "angular_tolerance") {
        angular_tolerance_ = parameter.as_double();
      } else if (param_name == "desired_distance") {
        desired_distance_ = parameter.as_double();
      } else if (param_name == "transform_tolerance") {
        transform_tolerance_ = parameter.as_double();
      } else if (param_name == "search_angle") {
        search_angle_ = parameter.as_double();
      } else if (param_name == "rotate_angular_velocity") {
        rotate_angular_velocity_ = parameter.as_double();
      } else if (param_name == "rotate_angular_acceleration") {
        rotate_angular_acceleration_ = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == "base_frame") {
        base_frame_ = parameter.as_string();
      } else if (param_name == "fixed_frame") {
        fixed_frame_ = parameter.as_string();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == "skip_orientation") {
        skip_orientation_ = parameter.as_bool();
      } else if (param_name == "search_by_rotating") {
        search_by_rotating_ = parameter.as_bool();
      }
    }
  }

  return result;
}

template<typename ActionT>
void FollowingServer::getPreemptedGoalIfRequested(
  typename std::shared_ptr<const typename ActionT::Goal> goal,
  const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename ActionT>
bool FollowingServer::checkAndWarnIfCancelled(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was cancelled. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

template<typename ActionT>
bool FollowingServer::checkAndWarnIfPreempted(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server->is_preempt_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was preempted. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

void FollowingServer::followObject()
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = following_action_server_->get_current_goal();
  auto result = std::make_shared<FollowObject::Result>();

  if (!following_action_server_ || !following_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled<FollowObject>(following_action_server_, "follow_object")) {
    following_action_server_->terminate_all();
    return;
  }

  getPreemptedGoalIfRequested<FollowObject>(goal, following_action_server_);
  num_retries_ = 0;
  static_timer_initialized_ = false;

  // Reset the last detected dynamic pose timestamp so we start fresh for this action
  detected_dynamic_pose_.header.stamp = rclcpp::Time(0);

  try {
    auto pose_topic = goal->pose_topic;
    auto target_frame = goal->tracked_frame;
    if (target_frame.empty()) {
      if (pose_topic.empty()) {
        RCLCPP_ERROR(
          get_logger(),
          "Both pose topic and target frame are empty. Cannot follow object.");
        result->error_code = FollowObject::Result::FAILED_TO_DETECT_OBJECT;
        result->error_msg = "No pose topic or target frame provided.";
        following_action_server_->terminate_all(result);
        return;
      } else {
        mutex_.unlock();
        RCLCPP_INFO(get_logger(), "Subscribing to pose topic: %s", pose_topic.c_str());
        dynamic_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          pose_topic,
          rclcpp::QoS(rclcpp::KeepLast(1)),  // Only want the most recent pose
          [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose) {
            detected_dynamic_pose_ = *pose;
          });
        mutex_.lock();
      }
    } else {
      RCLCPP_INFO(get_logger(), "Following frame: %s instead of pose", target_frame.c_str());
    }

    // Following control loop: while not timeout, run controller
    geometry_msgs::msg::PoseStamped object_pose;
    rclcpp::Duration max_duration = goal->max_duration;
    while (rclcpp::ok()) {
      try {
        // Check if we have run out of time
        if (this->now() - action_start_time_ > max_duration && max_duration.seconds() > 0.0) {
          RCLCPP_INFO(get_logger(), "Exceeded max duration. Stopping.");
          result->total_elapsed_time = this->now() - action_start_time_;
          result->num_retries = num_retries_;
          publishZeroVelocity();
          following_action_server_->succeeded_current(result);
          dynamic_pose_sub_.reset();
          return;
        }

        // Approach the object using control law
        if (approachObject(object_pose, target_frame)) {
          // Initialize static timer on first entry
          if (!static_timer_initialized_) {
            static_object_start_time_ = this->now();
            static_timer_initialized_ = true;
          }

          // We have reached the object, maintain position
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Reached object. Stopping until goal is moved again.");
          publishFollowingFeedback(FollowObject::Feedback::STOPPING);
          publishZeroVelocity();

          // Stop if the object has been static for some time
          if (static_object_timeout_ > 0.0) {
            auto static_duration = this->now() - static_object_start_time_;
            if (static_duration.seconds() > static_object_timeout_) {
              RCLCPP_INFO(
                get_logger(),
                "Object has been static for %.2f seconds (timeout: %.2f), stopping.",
                static_duration.seconds(), static_object_timeout_);
              result->total_elapsed_time = this->now() - action_start_time_;
              result->num_retries = num_retries_;
              publishZeroVelocity();
              following_action_server_->succeeded_current(result);
              return;
            }
          }
        } else {
          // Cancelled, preempted, or shutting down (recoverable errors throw DockingException)
          static_timer_initialized_ = false;
          result->total_elapsed_time = this->now() - action_start_time_;
          publishZeroVelocity();
          following_action_server_->terminate_all(result);
          dynamic_pose_sub_.reset();
          return;
        }
      } catch (opennav_docking_core::DockingException & e) {
        if (++num_retries_ > max_retries_) {
          RCLCPP_ERROR(get_logger(), "Failed to follow, all retries have been used");
          throw;
        }
        RCLCPP_WARN(get_logger(), "Following failed, will retry: %s", e.what());

        // Perform an in-place rotation to find the object again
        if (search_by_rotating_) {
          RCLCPP_INFO(get_logger(), "Rotating to find object again");
          if (!rotateToObject(object_pose, target_frame)) {
            // Cancelled, preempted, or shutting down
            publishZeroVelocity();
            following_action_server_->terminate_all(result);
            return;
          }
        } else {
          RCLCPP_INFO(get_logger(), "Using last known heading to find object again");
        }
      }
      loop_rate.sleep();
    }
  } catch (const tf2::TransformException & e) {
    result->error_msg = std::string("Transform error: ") + e.what();
    RCLCPP_ERROR(get_logger(), "%s", result->error_msg.c_str());
    result->error_code = FollowObject::Result::TF_ERROR;
  } catch (opennav_docking_core::FailedToDetectDock & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), "%s", result->error_msg.c_str());
    result->error_code = FollowObject::Result::FAILED_TO_DETECT_OBJECT;
  } catch (opennav_docking_core::FailedToControl & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), "%s", result->error_msg.c_str());
    result->error_code = FollowObject::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::DockingException & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), "%s", result->error_msg.c_str());
    result->error_code = FollowObject::Result::UNKNOWN;
  } catch (std::exception & e) {
    result->error_msg = e.what();
    RCLCPP_ERROR(get_logger(), "%s", result->error_msg.c_str());
    result->error_code = FollowObject::Result::UNKNOWN;
  }

  // Stop the robot and report
  result->total_elapsed_time = this->now() - action_start_time_;
  result->num_retries = num_retries_;
  publishZeroVelocity();
  following_action_server_->terminate_current(result);
  dynamic_pose_sub_.reset();
}

bool FollowingServer::approachObject(
  geometry_msgs::msg::PoseStamped & object_pose, const std::string & target_frame)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  while (rclcpp::ok()) {
    // Update the iteration start time, used for get robot position, transformation and control
    iteration_start_time_ = this->now();

    publishFollowingFeedback(FollowObject::Feedback::CONTROLLING);

    // Stop if cancelled/preempted
    if (checkAndWarnIfCancelled<FollowObject>(following_action_server_, "follow_object") ||
      checkAndWarnIfPreempted<FollowObject>(following_action_server_, "follow_object"))
    {
      return false;
    }

    // Get the tracking pose from topic or frame
    getTrackingPose(object_pose, target_frame);

    // Get the pose at the distance we want to maintain from the object
    // Stop and report success if goal is reached
    auto target_pose = getPoseAtDistance(object_pose, desired_distance_);
    if (isGoalReached(target_pose)) {
      return true;
    }

    // The control law can get jittery when close to the end when atan2's can explode.
    // Thus, we reduce the desired distance by a small amount so that the robot never
    // gets to the end of the spiral before its at the desired distance to stop the
    // following procedure.
    const double backward_projection = 0.25;
    const double effective_distance = desired_distance_ - backward_projection;
    target_pose = getPoseAtDistance(object_pose, effective_distance);

    // ... and transform the target_pose into base_frame
    try {
      tf2_buffer_->transform(
        target_pose, target_pose, base_frame_,
          tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Failed to transform target pose: %s", ex.what());
      return false;
    }

    // If the object is behind the robot, we reverse the control
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!nav2_util::getCurrentPose(
        robot_pose, *tf2_buffer_, target_pose.header.frame_id, base_frame_,
        transform_tolerance_,
        iteration_start_time_))
    {
      RCLCPP_WARN(get_logger(), "Failed to get current robot pose");
      return false;
    }

    // Compute and publish controls
    auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
    command->header.stamp = now();
    if (!controller_->computeVelocityCommand(target_pose.pose, command->twist, true, false)) {
      throw opennav_docking_core::FailedToControl("Failed to get control");
    }
    vel_publisher_->publish(std::move(command));

    loop_rate.sleep();
  }
  return false;
}

bool FollowingServer::rotateToObject(
  geometry_msgs::msg::PoseStamped & object_pose, const std::string & target_frame)
{
  const double dt = 1.0 / controller_frequency_;

  // Compute initial robot heading
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf2_buffer_, object_pose.header.frame_id, base_frame_,
      transform_tolerance_,
      iteration_start_time_))
  {
    RCLCPP_WARN(get_logger(), "Failed to get current robot pose");
    return false;
  }
  double initial_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // Search angles: left offset, then right offset from initial heading
  std::vector<double> angles = {initial_yaw + search_angle_,
    initial_yaw - search_angle_};

  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(rotate_to_object_timeout_);

  // Iterate over target angles
  for (const double & target_angle : angles) {
    // Create a target pose oriented at target_angle
    auto target_pose = object_pose;
    target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(target_angle);

    // Rotate towards target_angle while checking for detection
    while (rclcpp::ok()) {
      // Update the iteration start time, used for get robot position, transformation and control
      iteration_start_time_ = this->now();

      publishFollowingFeedback(FollowObject::Feedback::RETRY);

      // Stop if cancelled/preempted
      if (checkAndWarnIfCancelled<FollowObject>(following_action_server_, "follow_object") ||
        checkAndWarnIfPreempted<FollowObject>(following_action_server_, "follow_object"))
      {
        return false;
      }

      // Get current robot pose
      if (!nav2_util::getCurrentPose(
          robot_pose, *tf2_buffer_, object_pose.header.frame_id, base_frame_,
          transform_tolerance_,
          iteration_start_time_))
      {
        RCLCPP_WARN(get_logger(), "Failed to get current robot pose");
        return false;
      }

      double angular_distance_to_heading = angles::shortest_angular_distance(
        tf2::getYaw(robot_pose.pose.orientation), target_angle);

      // If we are close enough to the target orientation, break and try next angle
      if (fabs(angular_distance_to_heading) < angular_tolerance_) {
        break;
      }

      // While rotating, check if we can get the tracking pose (object detected)
      try {
        if (getTrackingPose(object_pose, target_frame)) {
          return true;
        }
      } catch (opennav_docking_core::FailedToDetectDock & e) {
        // No detection yet, continue rotating
      }

      // Simple accel-limited proportional controller to rotate towards target_angle.
      // (opennav_docking::Controller has no rotate-in-place command in this ROS
      // distribution, so we don't delegate to it here.)
      const double current_angular_vel = odom_sub_->getTwist().angular.z;
      const double desired_angular_vel = std::clamp(
        2.0 * angular_distance_to_heading,
        -rotate_angular_velocity_, rotate_angular_velocity_);
      const double max_vel_step = rotate_angular_acceleration_ * dt;
      const double command_angular_vel = current_angular_vel +
        std::clamp(desired_angular_vel - current_angular_vel, -max_vel_step, max_vel_step);

      auto command = std::make_unique<geometry_msgs::msg::TwistStamped>();
      command->header = robot_pose.header;
      command->twist.angular.z = command_angular_vel;

      vel_publisher_->publish(std::move(command));

      if (this->now() - start > timeout) {
        throw opennav_docking_core::FailedToControl("Timed out rotating to object");
      }

      loop_rate.sleep();
    }
  }

  // If we exhausted all search angles and did not detect the object, fail
  throw opennav_docking_core::FailedToControl("Failed to rotate to object");
}

void FollowingServer::publishZeroVelocity()
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.frame_id = base_frame_;
  cmd_vel->header.stamp = now();
  vel_publisher_->publish(std::move(cmd_vel));
}

void FollowingServer::publishFollowingFeedback(uint16_t state)
{
  auto feedback = std::make_shared<FollowObject::Feedback>();
  feedback->state = state;
  feedback->following_time = iteration_start_time_ - action_start_time_;
  feedback->num_retries = num_retries_;
  following_action_server_->publish_feedback(feedback);
}

bool FollowingServer::getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
{
  // Get current detections and transform to frame
  geometry_msgs::msg::PoseStamped detected = detected_dynamic_pose_;

  // If we haven't received any detection yet, wait up to detection_timeout_ for one to arrive.
  if (detected.header.stamp == builtin_interfaces::msg::Time{}) {
    auto start = this->now();
    auto timeout = rclcpp::Duration::from_seconds(detection_timeout_);
    rclcpp::Rate wait_rate(controller_frequency_);
    while (this->now() - start < timeout) {
      // Check if a new detection arrived
      if (detected_dynamic_pose_.header.stamp != builtin_interfaces::msg::Time{}) {
        detected = detected_dynamic_pose_;
        break;
      }
      wait_rate.sleep();
    }
    if (detected.header.stamp == builtin_interfaces::msg::Time{}) {
      RCLCPP_WARN(this->get_logger(), "No detection received within timeout period");
      return false;
    }
  }

  // Validate that external pose is new enough
  auto timeout = rclcpp::Duration::from_seconds(detection_timeout_);
  if (this->now() - detected.header.stamp > timeout) {
    RCLCPP_WARN(this->get_logger(), "Lost detection or did not detect: timeout exceeded");
    return false;
  }

  // Transform detected pose into fixed frame
  if (detected.header.frame_id != fixed_frame_) {
    try {
      tf2_buffer_->transform(
        detected, detected, fixed_frame_,
          tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to transform detected object pose: %s", ex.what());
      return false;
    }
  }

  // The control law can oscillate if the orientation in the perception
  // is not set correctly or has a lot of noise.
  // Then, we skip the target orientation by pointing it
  // in the same orientation than the vector from the robot to the object.
  if (skip_orientation_) {
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!nav2_util::getCurrentPose(
        robot_pose, *tf2_buffer_, detected.header.frame_id, base_frame_,
        transform_tolerance_,
        iteration_start_time_))
    {
      RCLCPP_WARN(get_logger(), "Failed to get current robot pose");
      return false;
    }
    double dx = detected.pose.position.x - robot_pose.pose.position.x;
    double dy = detected.pose.position.y - robot_pose.pose.position.y;
    double angle_to_target = std::atan2(dy, dx);
    detected.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(angle_to_target);
  }

  // Filter the detected pose
  auto pose_filtered = filter_->update(detected);
  filtered_dynamic_pose_pub_->publish(pose_filtered);

  pose = pose_filtered;
  return true;
}

bool FollowingServer::getFramePose(
  geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id)
{
  try {
    // Get the transform from the target frame to the fixed frame
    auto transform = tf2_buffer_->lookupTransform(
      fixed_frame_, frame_id, iteration_start_time_,
        tf2::durationFromSec(transform_tolerance_));

    // Convert transform to pose
    pose.header.frame_id = fixed_frame_;
    pose.header.stamp = transform.header.stamp;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to get transform for frame %s: %s", frame_id.c_str(), ex.what());
    return false;
  }

  // Filter the detected pose
  auto filtered_pose = filter_->update(pose);
  filtered_dynamic_pose_pub_->publish(filtered_pose);

  pose = filtered_pose;
  return true;
}

bool FollowingServer::getTrackingPose(
  geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id)
{
  // Use frame tracking if we have a target frame, otherwise use topic tracking
  if (!frame_id.empty()) {
    if (!getFramePose(pose, frame_id)) {
      throw opennav_docking_core::FailedToDetectDock(
              "Failed to get pose in target frame: " + frame_id);
    }
  } else {
    // Use the traditional pose detection from topic
    if (!getRefinedPose(pose)) {
      throw opennav_docking_core::FailedToDetectDock("Failed object detection");
    }
  }
  return true;
}

geometry_msgs::msg::PoseStamped FollowingServer::getPoseAtDistance(
  const geometry_msgs::msg::PoseStamped & pose, double distance)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf2_buffer_, pose.header.frame_id, base_frame_,
      transform_tolerance_,
      iteration_start_time_))
  {
    RCLCPP_WARN(get_logger(), "Failed to get current robot pose");
    // Return original pose as fallback
    return pose;
  }
  double dx = pose.pose.position.x - robot_pose.pose.position.x;
  double dy = pose.pose.position.y - robot_pose.pose.position.y;
  const double dist = std::hypot(dx, dy);
  geometry_msgs::msg::PoseStamped forward_pose = pose;
  forward_pose.pose.position.x -= distance * (dx / dist);
  forward_pose.pose.position.y -= distance * (dy / dist);
  return forward_pose;
}

bool FollowingServer::isGoalReached(const geometry_msgs::msg::PoseStamped & goal_pose)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf2_buffer_, goal_pose.header.frame_id, base_frame_,
      transform_tolerance_,
      iteration_start_time_))
  {
    RCLCPP_WARN(get_logger(), "Failed to get current robot pose");
    return false;
  }
  const double dist = std::hypot(
    robot_pose.pose.position.x - goal_pose.pose.position.x,
    robot_pose.pose.position.y - goal_pose.pose.position.y);
  const double yaw = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(goal_pose.pose.orientation));
  return dist < linear_tolerance_ && abs(yaw) < angular_tolerance_;
}

}  // namespace opennav_following

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(opennav_following::FollowingServer)
