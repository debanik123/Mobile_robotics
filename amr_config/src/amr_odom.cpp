#include "amr_config/amr_odom.hpp"

amr_odom::amr_odom()
: Node("amr_odom")
{
    // Initialize your class, if needed
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
}

void amr_odom::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
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

void amr_odom::getVelocities_Mecanum(double rpm_fl, double rpm_bl, double rpm_fr, double rpm_br, rclcpp::Time & time)
{
  double linear_rpm = (rpm_fr+rpm_br+rpm_fl+rpm_bl)/4.0;
  // double angular_rpm = (-rpm_fr+rpm_br+rpm_fl-rpm_bl)/(4.0*d);

}

void amr_odom::getVelocities_Tri(double steering_angle, double rpm, rclcpp::Time & time)
{
  double linear_rpm = rpm;
  double body_linear_vel =  getVel_from_rpm(linear_rpm);
  double body_angular_vel =  (body_linear_vel * tan(steering_angle)) / Wb;

  linear_ = body_linear_vel;
  angular_ = body_angular_vel;

  updateOpenLoop(linear_, angular_, time);

}

void amr_odom::getVelocities(double steering_angle, double rpm_l, double rpm_r, rclcpp::Time & time)
{
  double linear_rpm = (rpm_r + rpm_l)/2.0;
  double body_linear_vel =  getVel_from_rpm(linear_rpm);
  double body_angular_vel =  (body_linear_vel * tan(steering_angle)) / Wb;

  linear_ = body_linear_vel;
  angular_ = body_angular_vel;

  updateOpenLoop(linear_, angular_, time);

}

void amr_odom::getVelocities_ACKtype2(double steering_angle_l, double steering_angle_r, double rpm_l, double rpm_r, rclcpp::Time & time)
{
  double steering_angle = (steering_angle_l + steering_angle_r)/2.0;
  double linear_rpm = (rpm_r + rpm_l)/2.0;
  double body_linear_vel =  getVel_from_rpm(linear_rpm);
  double body_angular_vel =  (body_linear_vel * tan(steering_angle)) / Wb;

  linear_ = body_linear_vel;
  angular_ = body_angular_vel;

  updateOpenLoop(linear_, angular_, time);

}

void amr_odom::getVelocities(double rpm_l, double rpm_r, rclcpp::Time & time)
{
  double d = Wt/2.0;
  double linear_rpm = (rpm_r + rpm_l)/2.0;
  double angular_rpm = (rpm_r - rpm_l)/(2.0*d);

  double body_linear_vel =  getVel_from_rpm(linear_rpm);
  double body_angular_vel  = getVel_from_rpm(angular_rpm);

  linear_ = body_linear_vel;
  angular_ = body_angular_vel;

  updateOpenLoop(linear_, angular_, time);

}

void amr_odom::getVelocities(double rpm_fl, double rpm_bl, double rpm_fr, double rpm_br, rclcpp::Time & time)
{
  double d = Wt/2.0;
  double linear_rpm = (rpm_fr+rpm_br+rpm_fl+rpm_bl)/4.0;
  double angular_rpm = (rpm_fr+rpm_br-rpm_fl-rpm_bl)/(4.0*d);

  double body_linear_vel =  getVel_from_rpm(linear_rpm);
  double body_angular_vel  = getVel_from_rpm(angular_rpm);

  linear_ = body_linear_vel;
  angular_ = body_angular_vel;

  updateOpenLoop(linear_, angular_, time);

}

double amr_odom::getVel_from_rpm(double rpm)
{
  double linear_vel = (2 * M_PI * wheel_radius* rpm)/60.0;
  return linear_vel;
}


void amr_odom::updateOpenLoop(double linear, double angular, const rclcpp::Time & time)
{
  /// Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrateExact(linear * dt, angular * dt);
}

void amr_odom::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

void amr_odom::integrateExact(double linear, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
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