#ifndef FOLLOW_PATH_BASE_HPP
#define FOLLOW_PATH_BASE_HPP

#include "as2_core/node.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include <as2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Dense>
#include <deque>

namespace follow_path_base
{
    class FollowPathBase
    {
    public:
        using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>;

        void initialize(as2::Node *node_ptr, float goal_threshold)
        {
            node_ptr_ = node_ptr;
            goal_threshold_ = goal_threshold;
            odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
                node_ptr_->generate_global_name(as2_names::topics::self_localization::odom), as2_names::topics::self_localization::qos,
                std::bind(&FollowPathBase::odomCb, this, std::placeholders::_1));

            this->ownInit(node_ptr_);
        };

        virtual rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) = 0;
        virtual rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle) = 0;
        virtual bool onExecute(const std::shared_ptr<GoalHandleFollowPath> goal_handle) = 0;

        virtual ~FollowPathBase(){};

    protected:
        FollowPathBase(){};

        // To initialize needed publisher for each plugin
        virtual void ownInit(as2::Node *node_ptr){};

    private:
        // TODO: if onExecute is done with timer no atomic attributes needed
        void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
            this->current_pose_x_ = msg->pose.pose.position.x;
            this->current_pose_y_ = msg->pose.pose.position.y;
            this->current_pose_z_ = msg->pose.pose.position.z;

            this->actual_speed_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                                                  msg->twist.twist.linear.y,
                                                  msg->twist.twist.linear.z).norm();
        };

    protected:
        as2::Node *node_ptr_;
        float goal_threshold_;

        std::atomic<float> current_pose_x_;
        std::atomic<float> current_pose_y_;
        std::atomic<float> current_pose_z_;

        std::atomic<float> actual_speed_;

        std::deque<Eigen::Vector3d> waypoints_;

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    }; // FollowPathBase class

}  // follow_path_base namespace

#endif // FOLLOW_PATH_BASE_HPP