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

void amr_odom::integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos(direction);
    y_       += linear * sin(direction);
    heading_ += angular;
  }


  void amr_odom::integrateExact(double linear, double angular)
  {
    if (fabs(angular) < 1e-6)
      integrateRungeKutta2(linear, angular);
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double r = linear/angular;
      heading_ += angular;
      x_       +=  r * (sin(heading_) - sin(heading_old));
      y_       += -r * (cos(heading_) - cos(heading_old));
    }
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