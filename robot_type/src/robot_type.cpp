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

    int diff_ = diffDrive(linear_x, angular_z, wheelbase, wheel_radius);
    RCLCPP_INFO(get_logger(), "Received Twist message: linear_x = %f, angular_z = %f", linear_x, angular_z);
}

int robot_type::diffDrive(float linear_x, float angular_z, float wheelbase, float wheel_radius)
{
    float left_wheel_vel = linear_x - (angular_z * wheelbase / 2.0);
    float right_wheel_vel = linear_x + (angular_z * wheelbase / 2.0);

    int left_wheel_rpm = getRpm(left_wheel_vel, wheel_radius);
    int right_wheel_rpm = getRpm(right_wheel_vel, wheel_radius);

    RCLCPP_INFO(get_logger(), "left_wheel_rpm: %d, right_wheel_rpm: %d", left_wheel_rpm, right_wheel_rpm);
    return 0;
}


int robot_type::getRpm(float linear_vel, float wheel_radius)
{
    int rpm = static_cast<int>((linear_vel * 60) / (2 * M_PI * wheel_radius));
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