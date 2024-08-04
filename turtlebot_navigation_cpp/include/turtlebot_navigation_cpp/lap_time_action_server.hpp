#ifndef LAP_TIME_ACTION_SERVER_HPP_
#define LAP_TIME_ACTION_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/measure_lap_time.hpp"
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <memory>
#include <limits>

class LapTimeActionServer : public rclcpp::Node
{
public:
    using MeasureLapTime = custom_interfaces::action::MeasureLapTime;
    using GoalHandleMeasureLapTime = rclcpp_action::ServerGoalHandle<MeasureLapTime>;

    explicit LapTimeActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    rclcpp_action::Server<MeasureLapTime>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

    geometry_msgs::msg::Pose start_pose_;
    geometry_msgs::msg::Pose current_pose_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MeasureLapTime::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void execute(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle);
    bool check_lap_complete();
    double calculate_distance(const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2);
};

#endif  // LAP_TIME_ACTION_SERVER_HPP_