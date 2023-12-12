#ifndef ROBOT_TYPE_HPP
#define ROBOT_TYPE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
class robot_type : public rclcpp::Node
{
public:
    robot_type();
    ~robot_type();

    void CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr CmdVelSub;

//      ----    
//     | || |   | 
//     |    |   | whell_L
//    ||    ||  ~
//      ----
//      wheelbase

    float whell_L = 1.2; // Replace with your robot's whell_L in meters
    float wheelbase = 0.9;  // Replace with your robot's wheelbase in meters
    float wheel_radius = 0.207;  // Replace with your robot's wheel radius in meters

    int getRpm(float linear_vel);
    int diffDrive(float linear_x, float angular_z);
    int ackermannDrive(float linear_x, float angular_z);
    int tricycleDrive(float linear_x, float angular_z);
};

#endif  // ROBOT_TYPE_HPP
