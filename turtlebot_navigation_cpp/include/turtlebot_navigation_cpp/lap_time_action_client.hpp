#ifndef LAP_TIME_ACTION_CLIENT_HPP_
#define LAP_TIME_ACTION_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "custom_interfaces/action/measure_lap_time.hpp"

class LapTimeActionClient : public rclcpp::Node
{
public:
    using MeasureLapTime = custom_interfaces::action::MeasureLapTime;
    using GoalHandleMeasureLapTime = rclcpp_action::ClientGoalHandle<MeasureLapTime>;

    LapTimeActionClient();

private:
    void wall_found_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void send_goal();
    void goal_response_callback(std::shared_future<GoalHandleMeasureLapTime::SharedPtr> future);
    void feedback_callback(GoalHandleMeasureLapTime::SharedPtr, const std::shared_ptr<const MeasureLapTime::Feedback> feedback);
    void result_callback(const GoalHandleMeasureLapTime::WrappedResult & result);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    rclcpp_action::Client<MeasureLapTime>::SharedPtr action_client_;
};

#endif  // LAP_TIME_ACTION_CLIENT_HPP_