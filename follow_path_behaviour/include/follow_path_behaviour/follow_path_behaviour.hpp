#ifndef FOLLOW_PATH_BEHAVIOUR_HPP
#define FOLLOW_PATH_BEHAVIOUR_HPP

#include "as2_core/as2_basic_behaviour.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include <as2_msgs/action/follow_path.hpp>

#include <pluginlib/class_loader.hpp>
#include "follow_path_plugin_base/follow_path_base.hpp"

class FollowPathBehaviour : public as2::BasicBehaviour<as2_msgs::action::FollowPath>
{
public:
    using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>;

    FollowPathBehaviour() : as2::BasicBehaviour<as2_msgs::action::FollowPath>(as2_names::actions::behaviours::followpath)
    {
        this->declare_parameter("default_follow_path_plugin");
        this->declare_parameter("follow_path_threshold");

        loader_ = std::make_shared<pluginlib::ClassLoader<follow_path_base::FollowPathBase>>("follow_path_plugin_base", "follow_path_base::FollowPathBase");

        try
        {
            follow_path_ = loader_->createSharedInstance(this->get_parameter("default_follow_path_plugin").as_string());
            follow_path_->initialize(this, this->get_parameter("follow_path_threshold").as_double());
            RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED: %s", this->get_parameter("default_follow_path_plugin").as_string());
        }
        catch (pluginlib::PluginlibException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n", ex.what());
        }

        RCLCPP_INFO(this->get_logger(), "Follow Path Behaviour ready!");
    };

    rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal)
    {
        if (goal->trajectory_waypoints.poses.size() == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory waypoints are empty");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return follow_path_->onAccepted(goal);
    }

    rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
    {
        return follow_path_->onCancel(goal_handle);
    }

    void onExecute(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
    {
        if (follow_path_->onExecute(goal_handle))
        {
            RCLCPP_INFO(this->get_logger(), "Follow Path succeeded");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Follow Path canceled");
        }
    }

private:
    std::shared_ptr<pluginlib::ClassLoader<follow_path_base::FollowPathBase>> loader_;
    std::shared_ptr<follow_path_base::FollowPathBase> follow_path_;
};

#endif // FOLLOW_PATH_BEHAVIOUR_HPP