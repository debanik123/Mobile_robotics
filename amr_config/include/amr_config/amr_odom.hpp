#ifndef AMR_ODOM_HPP
#define AMR_ODOM_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
class amr_odom : public rclcpp::Node
{
public:
    amr_odom();
    ~amr_odom();

public:
    double get_x() const { return x_; }  // return x position [m]
    double get_y() const { return y_; }  // return y position [m]
    double get_heading() const { return heading_; } // return heading angle [rad]

    double get_linear_x() const { return linear_x_; }  // return linear velocity x [m/s]
    double get_linear_y() const { return linear_y_; }  // return linear velocity y [m/s]
    double get_angular_z() const { return angular_z_; }  // angular velocity [rad/s]

    int odom_update();

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
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

    double x_ = 0.0;
    double y_ = 0.0;
    double heading_ = 0.0;

    double linear_x_ = 0.0;
    double linear_y_ = 0.0;
    double angular_z_ = 0.0;
    

    double steering_angle;
    double drive_velocity;

    double drive_velocity_left;
    double drive_velocity_right;
    double angular_velocity;

    double steering_angle_r;
    double steering_angle_l;

    std::vector<double> pose_covarience{};
    


};

#endif  // amr_odom_HPP
