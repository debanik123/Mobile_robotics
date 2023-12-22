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


void amr_odom::getVelocities_two_steer_drive(double rpm1, double th1, double rpm2, double th2, rclcpp::Time & time)
{
  // RCLCPP_INFO(get_logger(), "Two_steer_drive Config");
  double l = Wb/2.0;
  double v1 = getVel_from_rpm(rpm1);
  double v2 = getVel_from_rpm(rpm2);

  auto [v1x, v1y] = Cartesian_from_polar(v1, th1);
  auto [v2x, v2y] = Cartesian_from_polar(v2, th2);

  double linear_vel_x = (v1x+v2x)/2.0;
  double linear_vel_y = (v1y+v2y)/2.0;
  double angular_z = (v1y-v2y)/(2.0*l);
}

void amr_odom::getVelocities_four_steer_drive(double rpm1, double th1, double rpm2, double th2, double rpm3, double th3, double rpm4, double th4, rclcpp::Time & time)
{
  // RCLCPP_INFO(get_logger(), "Four_steer_drive Config");
  double d = Wt/2.0;
  double l = Wb/2.0;
  double t = 4*(d*d + l*l);

  double sig1 = l/t;
  double sig2 = d/t;

  double v1 = getVel_from_rpm(rpm1);
  double v2 = getVel_from_rpm(rpm2);
  double v3 = getVel_from_rpm(rpm3);
  double v4 = getVel_from_rpm(rpm4);

  auto [v1x, v1y] = Cartesian_from_polar(v1, th1);
  auto [v2x, v2y] = Cartesian_from_polar(v2, th2);
  auto [v3x, v3y] = Cartesian_from_polar(v3, th3);
  auto [v4x, v4y] = Cartesian_from_polar(v4, th4);

  double linear_vel_x = (v1x+v2x+v3x+v4x);
  double linear_vel_y = (v1y+v2y+v3y+v4y);
  double angular_z = -sig2*v1x + sig1*v1y - sig2*v2x - sig1*v2y + sig2*v3x - sig1*v3y + sig2*v4x + sig1*v4y;

  // linear_ = body_linear_vel;
  // angular_ = body_angular_vel;

  // updateOpenLoop(linear_, angular_, time);
}


void amr_odom::getVelocities_Omni(double rpm1, double rpm2, double rpm3, rclcpp::Time & time)
{
  // RCLCPP_INFO(get_logger(), "OmniDrive Config");
  double linear_x_rpm = (0.0*rpm1-rpm2+rpm3)/sqrt(3);
  double linear_y_rpm = (2.0*rpm1-rpm2-rpm3)/3.0;
  double angular_rpm =  (rpm1+rpm2+rpm3)/(3*L);

  double linear_x_vel =  getVel_from_rpm(linear_x_rpm);
  double linear_y_vel =  getVel_from_rpm(linear_y_rpm);
  double angular_vel = getVel_from_rpm(angular_rpm);

}

void amr_odom::getVelocities_Mecanum(double rpm1, double rpm2, double rpm3, double rpm4, rclcpp::Time & time)
{
  // RCLCPP_INFO(get_logger(), "MechDrive Config");
  double rot = (Wt + Wb)/2.0; // [lx, ly]

  double linear_x_rpm = (rpm1+rpm2+rpm3+rpm4)/4.0;
  double linear_y_rpm = (-rpm1+rpm2+rpm3-rpm4)/(4.0);
  double angular_rpm = (-rpm1+rpm2-rpm3+rpm4)/(4.0*rot);

  double linear_x_vel =  getVel_from_rpm(linear_x_rpm);
  double linear_y_vel =  getVel_from_rpm(linear_y_rpm);
  double angular_vel = getVel_from_rpm(angular_rpm);
}

void amr_odom::getVelocities_hex_mechDrive(double rpm1, double rpm2, double rpm3, double rpm4, double rpm5, double rpm6, rclcpp::Time & time)
{
  // RCLCPP_INFO(get_logger(), "Hex MechDrive Config");
  double phi = (sqrt(3)+1);
  double phi2 = (1.0/(4*R_hex*(sqrt(3)+3)));

  double linear_x_rpm = (rpm1/4.0)+(rpm2/8.0)+(rpm3/8.0)+(rpm4/4.0)+(rpm5/8.0)+(rpm6/8.0);
  double linear_y_rpm = -(rpm1/4.0)+(rpm2/8.0)+(rpm3/8.0)-(rpm4/4.0)+(rpm5/8.0)+(rpm6/8.0);
  double angular_rpm = phi2*(-phi*rpm1 + phi*rpm2 - phi*rpm3 + phi*rpm4 - 2.0*rpm5 + 2.0*rpm6);

  double linear_x_vel = getVel_from_rpm(linear_x_rpm);
  double linear_y_vel = getVel_from_rpm(linear_y_rpm);
  double angular_vel = getVel_from_rpm(angular_rpm);
}


void amr_odom::getVelocities_Tri(double steering_angle, double rpm, rclcpp::Time & time)
{
  // RCLCPP_INFO(get_logger(), "TricycleDrive_type2_bicycle Config");
  double linear_rpm = rpm;
  double body_linear_vel =  getVel_from_rpm(linear_rpm);
  double body_angular_vel =  (body_linear_vel * tan(steering_angle)) / Wb;

  linear_ = body_linear_vel;
  angular_ = body_angular_vel;

  updateOpenLoop(linear_, angular_, time);

}

void amr_odom::getVelocities_tricycleDrive_type1(double steering_angle, double rpm_l, double rpm_r, rclcpp::Time & time)
{
  // RCLCPP_INFO(get_logger(), "TricycleDrive Config with diff");
  double linear_rpm = (rpm_r + rpm_l)/2.0;
  double body_linear_vel =  getVel_from_rpm(linear_rpm);
  double body_angular_vel =  (body_linear_vel * tan(steering_angle)) / Wb;

  linear_ = body_linear_vel;
  angular_ = body_angular_vel;

  updateOpenLoop(linear_, angular_, time);
}

void amr_odom::getVelocities_ACKtype(double steering_angle_l, double steering_angle_r, double rpm_l, double rpm_r, rclcpp::Time & time)
{
  // RCLCPP_INFO(get_logger(), "Ackermann Config");
  double steering_angle = (steering_angle_l + steering_angle_r)/2.0;
  double linear_rpm = (rpm_r + rpm_l)/2.0;
  double body_linear_vel =  getVel_from_rpm(linear_rpm);
  double body_angular_vel =  (body_linear_vel * tan(steering_angle)) / Wb;

  linear_ = body_linear_vel;
  angular_ = body_angular_vel;

  updateOpenLoop(linear_, angular_, time);
}

void amr_odom::getVelocities_DiffDrive(double rpm_l, double rpm_r, rclcpp::Time & time)
{
  // RCLCPP_INFO(get_logger(), "DiffDrive Config");
  double d = Wt/2.0;
  double linear_rpm = (rpm_r + rpm_l)/2.0;
  double angular_rpm = (rpm_r - rpm_l)/(2.0*d);

  double body_linear_vel =  getVel_from_rpm(linear_rpm);
  double body_angular_vel  = getVel_from_rpm(angular_rpm);

  linear_ = body_linear_vel;
  angular_ = body_angular_vel;

  updateOpenLoop(linear_, angular_, time);

}

void amr_odom::getVelocities_Four_wheel_drive(double rpm_fl, double rpm_bl, double rpm_fr, double rpm_br, rclcpp::Time & time)
{
  // RCLCPP_INFO(get_logger(), "Four_wheel_drive Config");
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

std::tuple<double, double> amr_odom::Cartesian_from_polar(double ro, double th)
{
  double x = ro*cos(th);
  double y = ro*sin(th);
  return std::make_tuple(x,y);
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