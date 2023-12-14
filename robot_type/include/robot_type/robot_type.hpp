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
    
     // Omni config
        //     | ^
        //     | L
        //     | ~
        //     /\   
        //    /  \  
        //   /    \ //
    float L = 0.5; 

    // Other config
    // \-----------\
    //       |         
    //       |
    //       |  Wb
    //       |
    //       |
    // |-----------| 
    //       Wt       

    float Wb = 1.1; // m
    float Wt = 0.8; // m
    float wheel_radius = 0.207;  //r Replace with your robot's wheel radius in meters

    float steering_angle;
    float drive_velocity;

    float drive_velocity_left;
    float drive_velocity_right;
    float angular_velocity;

    float steering_angle_r;
    float steering_angle_l;

    float R; // turning radius

    int getRpm(float linear_vel);
    std::tuple<float, float, int> polar_from_cart(float x,float y, float angular_z);
    int diffDrive(float linear_x, float angular_z);
    int ackermannDrive(float linear_x, float angular_z);
    int tricycleDrive_type1(float linear_x, float angular_z);
    int tricycleDrive_type2_bicycle(float linear_x, float angular_z);
    int four_wheel_drive(float linear_x, float angular_z);
    int omniDrive(float linear_x, float linear_y, float angular_z);
    int mechDrive(float linear_x, float linear_y, float angular_z);
    int four_steer_drive(float linear_x, float linear_y, float angular_z);


};

#endif  // ROBOT_TYPE_HPP
