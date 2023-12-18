#ifndef AMR_CONTROL_HPP
#define AMR_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
class amr_control : public rclcpp::Node
{
public:
    amr_control();
    ~amr_control();

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
    double L = 0.5; 

    // Other config
    // \-----------\
    //       |         
    //       |
    //       |  Wb
    //       |
    //       |
    // |-----------| 
    //       Wt       

    double Wb = 1.1; // m
    double Wt = 0.8; // m
    double wheel_radius = 0.207;  //r Replace with your robot's wheel radius in meters

    double steering_angle;
    double drive_velocity;

    double drive_velocity_left;
    double drive_velocity_right;
    double angular_velocity;

    double steering_angle_r;
    double steering_angle_l;

    double R; // turning radius

    int getRpm(double linear_vel);
    std::tuple<bool, double, double, int> polar_from_cart(double x,double y, double angular_z);
    int diffDrive(double linear_x, double angular_z);
    int ackermannDrive(double linear_x, double angular_z);
    int tricycleDrive_type1(double linear_x, double angular_z);
    int tricycleDrive_type2_bicycle(double linear_x, double angular_z);
    int four_wheel_drive(double linear_x, double angular_z);
    int omniDrive(double linear_x, double linear_y, double angular_z);
    int mechDrive(double linear_x, double linear_y, double angular_z);
    int four_steer_drive(double linear_x, double linear_y, double angular_z);
    int two_steer_drive(double linear_x, double linear_y, double angular_z);


};

#endif  // amr_control_HPP
