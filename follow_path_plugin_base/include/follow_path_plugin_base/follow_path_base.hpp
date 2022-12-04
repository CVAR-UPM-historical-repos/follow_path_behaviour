/*!*******************************************************************************************
 *  \file       follow_path_base.hpp
 *  \brief      follow_path_base header file
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef FOLLOW_PATH_BASE_HPP
#define FOLLOW_PATH_BASE_HPP

#include <rclcpp_action/rclcpp_action.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/platform_status.hpp"
#include "motion_reference_handlers/hover_motion.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace follow_path_base {

struct follow_path_plugin_params {
  double default_follow_path_max_speed = 0.0;
  double follow_path_threshold         = 0.0;
};

class FollowPathBase {
public:
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>;

  FollowPathBase(){};
  virtual ~FollowPathBase(){};

  void initialize(as2::Node *node_ptr,
                  const std::shared_ptr<as2::tf::TfHandler> tf_handler,
                  follow_path_plugin_params &params) {
    node_ptr_             = node_ptr;
    params_               = params;
    hover_motion_handler_ = std::make_shared<as2::motionReferenceHandlers::HoverMotion>(node_ptr_);
    ownInit();
  }

  virtual Eigen::Vector3d getTargetPosition() = 0;

  virtual void state_callback(geometry_msgs::msg::PoseStamped &pose_msg,
                              geometry_msgs::msg::TwistStamped &twist_msg) {
    actual_pose_ = pose_msg;

    feedback_.actual_speed = Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y,
                                             twist_msg.twist.linear.z)
                                 .norm();

    if (goal_accepted_) {
      feedback_.actual_distance_to_next_waypoint =
          (getTargetPosition() - Eigen::Vector3d(actual_pose_.pose.position.x,
                                                 actual_pose_.pose.position.y,
                                                 actual_pose_.pose.position.z))
              .norm();
    }

    distance_measured_ = true;
    return;
  }

  void platform_info_callback(const as2_msgs::msg::PlatformInfo::SharedPtr msg) {
    platform_state_ = msg->status.state;
    return;
  }

  virtual bool on_deactivate(const std::shared_ptr<std::string> &message) = 0;
  virtual bool on_pause(const std::shared_ptr<std::string> &message)      = 0;
  virtual bool on_resume(const std::shared_ptr<std::string> &message)     = 0;

  virtual bool on_activate(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) {
    as2_msgs::action::FollowPath::Goal goal_candidate = *goal;
    if (!processGoal(goal_candidate)) return false;

    if (own_activate(std::make_shared<as2_msgs::action::FollowPath::Goal>(goal_candidate))) {
      goal_ = goal_candidate;
      return true;
    }
    return false;
  }

  virtual bool on_modify(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) {
    as2_msgs::action::FollowPath::Goal goal_candidate = *goal;
    if (!processGoal(goal_candidate)) return false;

    if (own_modify(std::make_shared<as2_msgs::action::FollowPath::Goal>(goal_candidate))) {
      goal_ = goal_candidate;
      return true;
    }
    return false;
  }

  virtual as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal,
      std::shared_ptr<as2_msgs::action::FollowPath::Feedback> &feedback_msg,
      std::shared_ptr<as2_msgs::action::FollowPath::Result> &result_msg) {
    goal_accepted_                       = true;
    as2_behavior::ExecutionStatus status = own_run();

    feedback_msg = std::make_shared<as2_msgs::action::FollowPath::Feedback>(feedback_);
    result_msg   = std::make_shared<as2_msgs::action::FollowPath::Result>(result_);
    return status;
  }

  virtual void on_excution_end(const as2_behavior::ExecutionStatus &state) {
    distance_measured_ = false;
    goal_accepted_     = false;
    own_execution_end(state);
    return;
  }

protected:
  virtual void ownInit(){};

  virtual bool own_activate(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) {
    return true;
  }

  virtual as2_behavior::ExecutionStatus own_run() = 0;

  virtual bool own_modify(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) {
    return true;
  }

  virtual void own_execution_end(const as2_behavior::ExecutionStatus &state) = 0;

  inline void sendHover() {
    hover_motion_handler_->sendHover();
    return;
  };

  inline float getActualYaw() {
    return as2::frame::getYawFromQuaternion(actual_pose_.pose.orientation);
  };

protected:
  as2::Node *node_ptr_;

  as2_msgs::action::FollowPath::Goal goal_;
  as2_msgs::action::FollowPath::Feedback feedback_;
  as2_msgs::action::FollowPath::Result result_;

  int platform_state_;
  follow_path_plugin_params params_;
  geometry_msgs::msg::PoseStamped actual_pose_;
  bool distance_measured_ = false;
  bool goal_accepted_     = false;

  std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> hover_motion_handler_ = nullptr;

private:
  bool processGoal(as2_msgs::action::FollowPath::Goal &_goal) {
    if (platform_state_ != as2_msgs::msg::PlatformStatus::FLYING) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Behavior reject, platform is not flying");
      return false;
    }

    if (!distance_measured_) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Behavior reject, there is no localization");
      return false;
    }

    return true;
  }

};  // FollowPathBase class
}  // namespace follow_path_base

#endif  // FOLLOW_PATH_BASE_HPP