#ifndef AMR_ODOM_HPP
#define AMR_ODOM_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
class amr_odom : public rclcpp::Node
{
public:
    amr_odom();
    ~amr_odom();

    void CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg);

public:
    double get_x() const { return x_; }  // return x position [m]
    double get_y() const { return y_; }  // return y position [m]
    double get_heading() const { return heading_; } // return heading angle [rad]

    double get_linear_x() const { return linear_x_; }  // return linear velocity x [m/s]
    double get_linear_y() const { return linear_y_; }  // return linear velocity y [m/s]
    double get_angular() const { return angular_; }  // angular velocity [rad/s]

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

    // pose_state && vel_state are the robot body frame informations.

    // pose_state = [pose_x, pose_y, orien_z]; || [pose_x, pose_y, steer_angle];
    // vel_state = [linear_x, linear_y, angular_z];

    double x_, y_, heading_, linear_x_, linear_y_, angular_;

    double steering_angle;
    double drive_velocity;

    double drive_velocity_left;
    double drive_velocity_right;
    double angular_velocity;

    double steering_angle_r;
    double steering_angle_l;

    


};

#endif  // amr_odom_HPP
