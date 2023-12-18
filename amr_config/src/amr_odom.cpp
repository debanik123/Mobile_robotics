#include "amr_config/amr_odom.hpp"

amr_odom::amr_odom()
: Node("amr_odom")
{
    // Initialize your class, if needed
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
}


amr_odom::~amr_odom()
{
    // Cleanup, if needed
    odom_pub_.reset();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_odom>());
  rclcpp::shutdown();
  return 0;
}