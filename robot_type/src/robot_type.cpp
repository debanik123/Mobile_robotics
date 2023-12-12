#include "robot_type/robot_type.hpp"

robot_type::robot_type()
: Node("robot_type_node")
{
    // Initialize your class, if needed
    CmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&robot_type::CmdVelCb, this, std::placeholders::_1));
}

void robot_type::CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    float linear_x = msg->linear.x;
    float linear_y = msg->linear.x;
    float angular_z = msg->angular.z;

    // int diff_ = diffDrive(linear_x, angular_z);
    // int ackerDrive_ = ackermannDrive(linear_x, angular_z);
    int triDrive_ = tricycleDrive(linear_x, angular_z);

    RCLCPP_INFO(get_logger(), "Received Twist message: linear_x = %f, angular_z = %f", linear_x, angular_z);
}

int robot_type::ackermannDrive(float linear_x, float angular_z)
{
    float steering_angle = atan2(whell_L * angular_z, linear_x);
    if (std::abs(steering_angle) < 1e-6) {
        RCLCPP_INFO(get_logger(), "Steering angle is close to zero. Setting default values.");
        int diff_ = diffDrive(linear_x, angular_z);

    }
    else
    {
        float turning_radius = whell_L / tan(steering_angle);
        float left_wheel_vel = linear_x * (turning_radius - whell_L / 2.0) / turning_radius;
        float right_wheel_vel = linear_x * (turning_radius + whell_L / 2.0) / turning_radius;
        
        float angular_velocity = linear_x * tan(steering_angle) / whell_L;

        int left_wheel_rpm = getRpm(left_wheel_vel);
        int right_wheel_rpm = getRpm(right_wheel_vel);

        // Print information for illustration
        RCLCPP_INFO(get_logger(), "Steering Angle: %f radians", steering_angle);
        RCLCPP_INFO(get_logger(), "Turning Radius: %f meters", turning_radius);
        RCLCPP_INFO(get_logger(), "Left Wheel Velocity: %f m/s, Right Wheel Velocity: %f m/s", left_wheel_vel, right_wheel_vel);
        RCLCPP_INFO(get_logger(), "Angular Velocity (omega): %f rad/s", angular_velocity);
        RCLCPP_INFO(get_logger(), "Left Wheel RPM: %d, Right Wheel RPM: %d", left_wheel_rpm, right_wheel_rpm);
    }

    

    return 0;

}

int robot_type::tricycleDrive(float linear_x, float angular_z)
{
    float steering_angle_ack = atan2(whell_L * angular_z, linear_x);
    // float steering_angle_tri = asin(whell_L * angular_z/ linear_x);
    float drive_vel = hypot(whell_L * angular_z, linear_x);
    int drive_vel_rpm = getRpm(drive_vel);

    RCLCPP_INFO(get_logger(), "Steering Angle (atan2): %f radians", steering_angle_ack);
    // RCLCPP_INFO(get_logger(), "Steering Angle (asin): %f radians", steering_angle_tri);
    RCLCPP_INFO(get_logger(), "Drive Velocity: %f m/s", drive_vel);
    RCLCPP_INFO(get_logger(), "Drive Velocity (RPM): %d RPM", drive_vel_rpm);

    return 0;
}

int robot_type::diffDrive(float linear_x, float angular_z)
{
    float left_wheel_vel = linear_x - (angular_z * wheelbase / 2.0);
    float right_wheel_vel = linear_x + (angular_z * wheelbase / 2.0);

    int left_wheel_rpm = getRpm(left_wheel_vel);
    int right_wheel_rpm = getRpm(right_wheel_vel);

    RCLCPP_INFO(get_logger(), "left_wheel_rpm: %d, right_wheel_rpm: %d", left_wheel_rpm, right_wheel_rpm);
    return 0;
}


int robot_type::getRpm(float linear_vel)
{
    int rpm = static_cast<int>((linear_vel * 60) / (2 * M_PI * wheel_radius));
    rpm = std::min(255, std::max(-255, rpm));
    return rpm;
}

robot_type::~robot_type()
{
    // Cleanup, if needed
    CmdVelSub.reset();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_type>());
  rclcpp::shutdown();
  return 0;
}