#include "turtlebot_navigation_cpp/lap_time_action_server.hpp"
using namespace std::chrono_literals;
LapTimeActionServer::LapTimeActionServer(const rclcpp::NodeOptions & options)
: Node("lap_time_action_server", options)
{
    action_server_ = rclcpp_action::create_server<MeasureLapTime>(
        this,
        "measure_lap_time",
        std::bind(&LapTimeActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&LapTimeActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&LapTimeActionServer::handle_accepted, this, std::placeholders::_1));

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&LapTimeActionServer::odom_callback, this, std::placeholders::_1));

    start_pose_.position.x = std::numeric_limits<double>::quiet_NaN();
    current_pose_.position.x = std::numeric_limits<double>::quiet_NaN();
}

rclcpp_action::GoalResponse LapTimeActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MeasureLapTime::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LapTimeActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LapTimeActionServer::handle_accepted(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle)
{
    std::thread{std::bind(&LapTimeActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void LapTimeActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pose_ = msg->pose.pose;
}

void LapTimeActionServer::execute(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal...");

    if (std::isnan(start_pose_.position.x)) {
        start_pose_ = current_pose_;
    }

    auto start_time = this->now();

    while (rclcpp::ok()) {
        if (this->check_lap_complete() && ((this->now() - start_time).seconds() > 10.0)) {
            auto total_time = (this->now() - start_time).seconds();
            auto result = std::make_shared<MeasureLapTime::Result>();
            result->total_time = total_time;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Lap completed in %f seconds", total_time);
            return;
        }

        auto elapsed_time = (this->now() - start_time).seconds();
        auto feedback = std::make_shared<MeasureLapTime::Feedback>();
        feedback->elapsed_time = elapsed_time;
        goal_handle->publish_feedback(feedback);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool LapTimeActionServer::check_lap_complete()
{
    if (std::isnan(start_pose_.position.x) || std::isnan(current_pose_.position.x)) {
        return false;
    }

    double distance = this->calculate_distance(start_pose_, current_pose_);
    return distance < 0.5; 
}

double LapTimeActionServer::calculate_distance(const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2)
{
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LapTimeActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}