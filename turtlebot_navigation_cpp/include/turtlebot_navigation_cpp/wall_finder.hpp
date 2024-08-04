#ifndef WALL_FINDER
#define WALL_FINDER

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <custom_interfaces/srv/find_wall.hpp>


class WallFinder : public rclcpp::Node
{
public:
    WallFinder();
    
private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void find_wall_logic(const sensor_msgs::msg::LaserScan::SharedPtr msg, geometry_msgs::msg::Twist &twist);
    float getFrontRegion(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void find_wall_service_callback(const std::shared_ptr<custom_interfaces::srv::FindWall::Request> request,
                                    std::shared_ptr<custom_interfaces::srv::FindWall::Response> response);


    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wall_found_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Service<custom_interfaces::srv::FindWall>::SharedPtr find_wall_service_;

    float find_wall_distance;
    bool finding_wall_;
};

#endif  // WALL_FINDER