#include "turtlebot_navigation_cpp/lap_time_action_client.hpp"

using namespace std::placeholders;

LapTimeActionClient::LapTimeActionClient()
: Node("lap_time_action_client")
{
    action_client_ = rclcpp_action::create_client<MeasureLapTime>(this, "measure_lap_time");
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "wall_found", 10, std::bind(&LapTimeActionClient::wall_found_callback, this, _1));

    // Wait for the action server to be available
    while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }
}

void LapTimeActionClient::wall_found_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "WALL FOUND: SENDING GOAL");
        send_goal();
    }
}

void LapTimeActionClient::send_goal()
{
    auto goal_msg = MeasureLapTime::Goal();

    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<MeasureLapTime>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](std::shared_ptr<GoalHandleMeasureLapTime> goal_handle) {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal rejected :(");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted :)");
        }
    };
    send_goal_options.feedback_callback = std::bind(&LapTimeActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&LapTimeActionClient::result_callback, this, _1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
}

void LapTimeActionClient::goal_response_callback(std::shared_future<GoalHandleMeasureLapTime::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal rejected :(");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted :)");
    }
}

void LapTimeActionClient::feedback_callback(GoalHandleMeasureLapTime::SharedPtr, const std::shared_ptr<const MeasureLapTime::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Received feedback: %f seconds elapsed", feedback->elapsed_time);
}

void LapTimeActionClient::result_callback(const GoalHandleMeasureLapTime::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Result: Lap completed in %f seconds", result.result->total_time);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Unknown result code");
            break;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LapTimeActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}