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
    // int triDrive_ = tricycleDrive(linear_x, angular_z);
    // int forDrive_ = four_wheel_drive(linear_x, angular_z);
    // int forDrive_ = mechDrive(linear_x, linear_y, angular_z);
    int omniDrive_ = omniDrive(linear_x, linear_y, angular_z);

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

int robot_type::four_wheel_drive(float linear_x, float angular_z)
{
    float left_FW_wheel_vel = linear_x - (angular_z * wheelbase / 2.0);
    float left_BW_wheel_vel = linear_x - (angular_z * wheelbase / 2.0);
    float right_FW_wheel_vel = linear_x + (angular_z * wheelbase / 2.0);
    float right_BW_wheel_vel = linear_x + (angular_z * wheelbase / 2.0);

    float left_FW_wheel_rpm = getRpm(left_FW_wheel_vel);
    float left_BW_wheel_rpm = getRpm(left_BW_wheel_vel);
    float right_FW_wheel_rpm = getRpm(right_FW_wheel_vel);
    float right_BW_wheel_rpm = getRpm(right_BW_wheel_vel);

    RCLCPP_INFO(get_logger(), "Left Front Wheel Velocity: %f m/s", left_FW_wheel_vel);
    RCLCPP_INFO(get_logger(), "Left Back Wheel Velocity: %f m/s", left_BW_wheel_vel);
    RCLCPP_INFO(get_logger(), "Right Front Wheel Velocity: %f m/s", right_FW_wheel_vel);
    RCLCPP_INFO(get_logger(), "Right Back Wheel Velocity: %f m/s", right_BW_wheel_vel);

    RCLCPP_INFO(get_logger(), "Left Front Wheel RPM: %f RPM", left_FW_wheel_rpm);
    RCLCPP_INFO(get_logger(), "Left Back Wheel RPM: %f RPM", left_BW_wheel_rpm);
    RCLCPP_INFO(get_logger(), "Right Front Wheel RPM: %f RPM", right_FW_wheel_rpm);
    RCLCPP_INFO(get_logger(), "Right Back Wheel RPM: %f RPM", right_BW_wheel_rpm);

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

int robot_type::omniDrive(float linear_x, float linear_y, float angular_z)
{
    float rot = L*angular_z;

    float FO_wheel_vel = linear_y + rot;
    float LO_wheel_vel = -linear_x*sin(M_PI/3.0)-linear_y*cos(M_PI/3.0)+ rot;
    float RO_wheel_vel = linear_x*sin(M_PI/3.0)-linear_y*cos(M_PI/3.0)+ rot;

    int FO_wheel_rpm = getRpm(FO_wheel_vel);
    int LO_wheel_rpm = getRpm(LO_wheel_vel);
    int RO_wheel_rpm = getRpm(RO_wheel_vel);

    RCLCPP_INFO(get_logger(), "FO_wheel_vel: %f", FO_wheel_vel);
    RCLCPP_INFO(get_logger(), "LO_wheel_vel: %f", LO_wheel_vel);
    RCLCPP_INFO(get_logger(), "RO_wheel_vel: %f", RO_wheel_vel);

    RCLCPP_INFO(get_logger(), "FO_wheel_rpm: %d", FO_wheel_rpm);
    RCLCPP_INFO(get_logger(), "LO_wheel_rpm: %d", LO_wheel_rpm);
    RCLCPP_INFO(get_logger(), "RO_wheel_rpm: %d", RO_wheel_rpm);

}

int robot_type::mechDrive(float linear_x, float linear_y, float angular_z)
{
    float rot = lx + ly;
    // velocity
    float front_left_vel = (linear_x - linear_y - rot * angular_z);
    float front_right_vel = (linear_x + linear_y + rot * angular_z);
    float back_left_vel = (linear_x + linear_y - rot * angular_z);
    float back_right_vel = (linear_x - linear_y + rot * angular_z);

    int front_left_rpm = getRpm(front_left_vel);
    int front_right_rpm = getRpm(front_right_vel);
    int back_left_rpm = getRpm(back_left_vel);
    int back_right_rpm = getRpm(back_right_vel);

    RCLCPP_INFO(get_logger(), "Front Left Velocity: %f", front_left_vel);
    RCLCPP_INFO(get_logger(), "Front Right Velocity: %f", front_right_vel);
    RCLCPP_INFO(get_logger(), "Back Left Velocity: %f", back_left_vel);
    RCLCPP_INFO(get_logger(), "Back Right Velocity: %f", back_right_vel);
    
    RCLCPP_INFO(get_logger(), "Front Left RPM: %d", front_left_rpm);
    RCLCPP_INFO(get_logger(), "Front Right RPM: %d", front_right_rpm);
    RCLCPP_INFO(get_logger(), "Back Left RPM: %d", back_left_rpm);
    RCLCPP_INFO(get_logger(), "Back Right RPM: %d", back_right_rpm);
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