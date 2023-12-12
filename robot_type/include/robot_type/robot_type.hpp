#ifndef ROBOT_TYPE_HPP
#define ROBOT_TYPE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/twist.hpp>

class robot_type : public rclcpp::Node
{
public:
    robot_type();
    ~robot_type();

    void CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr CmdVelSub;
    double wheelbase = 0.9;  // Replace with your robot's wheelbase in meters
    double wheel_radius = 0.207;  // Replace with your robot's wheel radius in meters

    int getRpm(float linear_vel, float radius);
};

#endif  // ROBOT_TYPE_HPP
