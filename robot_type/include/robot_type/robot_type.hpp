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

    // y
    // |
    // |
    //  ----- x
    // The robot is 15 cm in width and 20 cm in length.
    float lx = 0.15; // m
    float ly = 0.2; //m

        //     | ^
        //     | L
        //     | ~
        //     /\   
        //    /  \  
        //   /    \ //
    float L = 0.5; //


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

    float steering_angle;
    float drive_velocity;

    float drive_velocity_left;
    float drive_velocity_right;
    float angular_velocity;

    float steering_angle_r;
    float steering_angle_l;


    float R; // turning radius

    int getRpm(float linear_vel);
    int diffDrive(float linear_x, float angular_z);
    int ackermannDrive(float linear_x, float angular_z);
    int tricycleDrive_type1(float linear_x, float angular_z);
    int tricycleDrive_type2(float linear_x, float angular_z);
    int four_wheel_drive(float linear_x, float angular_z);
    int omniDrive(float linear_x, float linear_y, float angular_z);
    int mechDrive(float linear_x, float linear_y, float angular_z);


};

#endif  // ROBOT_TYPE_HPP
