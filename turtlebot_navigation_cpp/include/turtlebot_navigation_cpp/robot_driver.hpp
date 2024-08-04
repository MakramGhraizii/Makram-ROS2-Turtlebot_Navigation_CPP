#ifndef ROBOT_DRIVER_HPP_
#define ROBOT_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <custom_interfaces/srv/find_wall.hpp>
#include "std_msgs/msg/bool.hpp"

class RobotDriver : public rclcpp::Node
{
public:
    RobotDriver();
    
private:
    void listener_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void follow_wall_logic(const sensor_msgs::msg::LaserScan::SharedPtr msg, geometry_msgs::msg::Twist &twist);
    float getFrontRegion(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    float getRightRegion(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void wall_found_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void call_find_wall_service();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wall_found_subscription_;
    rclcpp::Client<custom_interfaces::srv::FindWall>::SharedPtr client_;
    rclcpp::Client<custom_interfaces::srv::FindWall>::SharedPtr find_wall_client_;

    bool finding_wall_;
    float distance_to_wall_;
    float wall_distance_tolerance_;
    float distance_threshold_;
};

#endif  // ROBOT_DRIVER_HPP_ 