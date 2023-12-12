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
    float angular_z = msg->angular.z;
    RCLCPP_INFO(get_logger(), "Received Twist message: linear_x = %f, angular_z = %f", linear_x, angular_z);
}

int robot_type::getRpm(float linear_vel, float radius)
{
    int rpm = static_cast<int>((linear_vel * 60) / (2 * M_PI * radius));
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