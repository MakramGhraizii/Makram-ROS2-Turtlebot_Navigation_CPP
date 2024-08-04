#include "turtlebot_navigation_cpp/robot_driver.hpp"

using std::placeholders::_1;

RobotDriver::RobotDriver()
: Node("robot_driver"),
  finding_wall_(true),
  distance_to_wall_(0.4),
  wall_distance_tolerance_(0.02),
  distance_threshold_(1.25)
{
    //publisher to control robot speed with the /smd_vel topic
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    //subscription to take the 360 degree ranges of the robot in the /scan topic, and run the listener_callback at every published scan
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&RobotDriver::listener_callback, this, _1));
    
    // publisher to tell when the wall is found (aka start the race)
    wall_found_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "wall_found", 10, std::bind(&RobotDriver::wall_found_callback, this, _1));

    find_wall_client_ = this->create_client<custom_interfaces::srv::FindWall>("find_wall");

    call_find_wall_service();

}

void RobotDriver::wall_found_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Wall found signal received.");
        finding_wall_ = false;
    }
}

void RobotDriver::call_find_wall_service()
{
    while (!find_wall_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for WallFinder service...");
    }

    auto request = std::make_shared<custom_interfaces::srv::FindWall::Request>();
    auto future = find_wall_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Wall found by WallFinder service started.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Wall not found by WallFinder service ? never.");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call WallFinder service.");
    }
}

void RobotDriver::listener_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!finding_wall_) {
        auto twist = geometry_msgs::msg::Twist();
        follow_wall_logic(msg, twist);
        publisher_->publish(twist);

    } 
}

float RobotDriver::getFrontRegion(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    // Define the front region angle range (e.g., ±20 degrees around the front)
    float front_angle = 20.0;
    float front_angle_range = front_angle * M_PI / 180.0;  // Convert degrees to radians

    // Calculate the indices for the desired angle range
    int start_index = static_cast<int>((msg->angle_min - front_angle_range) / msg->angle_increment);
    int end_index = static_cast<int>(( msg->angle_min + front_angle_range) / msg->angle_increment);

    int num_ranges = static_cast<int>(msg->ranges.size());
    start_index = (start_index + num_ranges) % num_ranges;
    end_index = (end_index + num_ranges) % num_ranges;

     // Collect the distances in the front region
    std::vector<float> front_ranges;
    if (start_index < end_index) {
        front_ranges.insert(front_ranges.end(), msg->ranges.begin() + start_index, msg->ranges.begin() + end_index + 1);
    } else {
        front_ranges.insert(front_ranges.end(), msg->ranges.begin() + start_index, msg->ranges.end());
        front_ranges.insert(front_ranges.end(), msg->ranges.begin(), msg->ranges.begin() + end_index + 1);
    }

    // Check if ranges array is not empty and indices are valid
    if (front_ranges.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid ranges in LaserScan data");
        return std::numeric_limits<float>::infinity();
    }

     // Get the minimum distance in the front region
    float front_region = *std::min_element(front_ranges.begin(), front_ranges.end());

    return front_region;
}

float RobotDriver::getRightRegion(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    // Define the right region angle range (e.g., from 60 to 90 degrees)
    float start_angle = -90.0;  // Right 90 degrees
    float end_angle = -60.0;    // Right 60 degrees
    float start_angle_rad = start_angle * M_PI / 180.0;  // Convert degrees to radians
    float end_angle_rad = end_angle * M_PI / 180.0;      // Convert degrees to radians

    // Calculate the indices for the desired angle range
    int start_index = static_cast<int>((start_angle_rad - msg->angle_min) / msg->angle_increment);
    int end_index = static_cast<int>((end_angle_rad - msg->angle_min) / msg->angle_increment);

    int num_ranges = static_cast<int>(msg->ranges.size());
    start_index = (start_index + num_ranges) % num_ranges;
    end_index = (end_index + num_ranges) % num_ranges;

    RCLCPP_INFO(this->get_logger(), "Start index: %d", start_index);
    RCLCPP_INFO(this->get_logger(), "End index: %d", end_index);

    // Collect the distances in the right region
    std::vector<float> right_ranges;
    if (start_index < end_index) {
        right_ranges.insert(right_ranges.end(), msg->ranges.begin() + start_index, msg->ranges.begin() + end_index + 1);
    } else {
        right_ranges.insert(right_ranges.end(), msg->ranges.begin() + start_index, msg->ranges.end());
        right_ranges.insert(right_ranges.end(), msg->ranges.begin(), msg->ranges.begin() + end_index + 1);
    }

    // Check if ranges array is not empty and indices are valid
    if (right_ranges.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid ranges in LaserScan data");
        return std::numeric_limits<float>::infinity();
    }

    // Get the minimum distance in the right region
    float right_region = *std::min_element(right_ranges.begin(), right_ranges.end());

    return right_region;
}

void RobotDriver::follow_wall_logic(const sensor_msgs::msg::LaserScan::SharedPtr msg, geometry_msgs::msg::Twist &twist)
{
    auto ranges = msg->ranges;

    float front_region = getFrontRegion(msg);

    float right_region = getRightRegion(msg);

    // Define a maximum angular velocity
    const float max_angular_velocity = 1;

    // Calculate the proportional control for angular velocity based on front region distance
    if (front_region < distance_threshold_) {
        float proximity_factor = front_region / distance_threshold_;

        twist.linear.x = 0.8 * proximity_factor;
        twist.angular.z = max_angular_velocity * (1 - proximity_factor);
    } else {
        if (right_region < distance_to_wall_ - wall_distance_tolerance_) {
            float proximity_factor = right_region / (distance_to_wall_ - wall_distance_tolerance_);
            twist.linear.x = 0.8;
            twist.angular.z = max_angular_velocity * (1 - proximity_factor) ;  // Too close to wall, turn left
        } else if (right_region > distance_to_wall_ + wall_distance_tolerance_) {
            float proximity_factor = right_region / (distance_to_wall_ + wall_distance_tolerance_);
            twist.linear.x = 0.8;
            twist.angular.z = -max_angular_velocity * (proximity_factor - 1);  // Too far from wall, turn right
        } else {
            twist.linear.x = 0.8;
            twist.angular.z = 0.0;  // Move forward while maintaining distance
        }
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}