#include "amr_config/amr_odom.hpp"

amr_odom::amr_odom()
: Node("amr_odom")
{
    // Initialize your class, if needed
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
}

int amr_odom::odom_update()
{
    rclcpp::Time now = this->get_clock()->now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = get_x();
    odom.pose.pose.position.y = get_y();
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, get_heading());

    odom.pose.pose.orientation.x = quaternion[0];
    odom.pose.pose.orientation.y = quaternion[1];
    odom.pose.pose.orientation.z = quaternion[2];
    odom.pose.pose.orientation.w = quaternion[3];

    odom.twist.twist.linear.x = get_linear_x();
    odom.twist.twist.linear.y = get_linear_y();
    odom.twist.twist.angular.z = get_angular_z();

    odom_pub_->publish(odom);


    return 0;
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