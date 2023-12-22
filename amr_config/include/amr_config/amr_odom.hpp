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

    double get_linear_x() const { return linear_; }  // return linear velocity x [m/s]
    double get_linear_y() const { return linear_y_; }  // return linear velocity y [m/s]
    double get_angular_z() const { return angular_; }  // angular velocity [rad/s]

    void resetOdometry();
    
    void updateOpenLoop(double linear, double angular, const rclcpp::Time & time);
    void integrateRungeKutta2(double linear, double angular);
    void integrateExact(double linear, double angular);

    void getVelocities_two_steer_drive(double rpm1, double th1, double rpm2, double th2, rclcpp::Time & time);
    void getVelocities(double rpm1, double th1, double rpm2, double th2, double rpm3, double th3, double rpm4, double th4, rclcpp::Time & time);
    void getVelocities_Omni(double rpm1, double rpm2, double rpm3, rclcpp::Time & time);
    void getVelocities_Mecanum(double rpm1, double rpm2, double rpm3, double rpm4, rclcpp::Time & time);
    void getVelocities_Tri(double steering_angle, double rpm, rclcpp::Time & time);
    void getVelocities_ACKtype2(double steering_angle_l, double steering_angle_r, double rpm_l, double rpm_r, rclcpp::Time & time);
    void getVelocities(double steering_angle, double rpm_l, double rpm_r, rclcpp::Time & time);
    void getVelocities(double rpm_l, double rpm_r, rclcpp::Time & time);
    void getVelocities(double rpm_fl, double rpm_bl, double rpm_fr, double rpm_br, rclcpp::Time & time);

    double getVel_from_rpm(double rpm);
    std::tuple<double, double> Cartesian_from_polar(double ro, double th);
    int odom_update();

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Current timestamp:
    rclcpp::Time timestamp_;
    
    /*
        Omni config
            | ^
            | L
            | ~
            /\   
           /  \  
          /    \ 
    

        Other config
        \----------\
            |         
            |
            |  Wb
            |
            |
        |-----------| 
            Wt       

    */
    
    double L = 0.5; 
    double Wb = 1.1; // m
    double Wt = 0.8; // m
    double wheel_radius = 0.207;  //r Replace with your robot's wheel radius in meters

    // pose_state && vel_state are the robot body frame informations.

    // pose_state = [pose_x, pose_y, orien_z]; || [pose_x, pose_y, steer_angle];
    // vel_state = [linear_x, linear_y, angular_z];

    double x_ = 0.0;
    double y_ = 0.0;
    double heading_ = 0.0;

    double linear_ = 0.0;
    double linear_y_ = 0.0;
    double angular_ = 0.0;


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
