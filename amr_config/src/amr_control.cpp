#include "amr_config/amr_control.hpp"

amr_control::amr_control()
: Node("amr_control_node")
{
    // Initialize your class, if needed
    CmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&amr_control::CmdVelCb, this, std::placeholders::_1));
}

void amr_control::CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double linear_x = msg->linear.x;
    double linear_y = msg->linear.x;
    double angular_z = msg->angular.z;

    // int diff_ = diffDrive(linear_x, angular_z);
    int ackerDrive_ = ackermannDrive(linear_x, angular_z);
    // int triDrive_ty1 = tricycleDrive_type1(linear_x, angular_z);
    // int triDrive_ty2 = tricycleDrive_type2_bicycle(linear_x, angular_z);
    // int forDrive_ = four_wheel_drive(linear_x, angular_z);
    // int mechDrive_ = mechDrive(linear_x, linear_y, angular_z);
    // int omniDrive_ = omniDrive(linear_x, linear_y, angular_z);
    // int four_steer_drive_ = four_steer_drive(linear_x, linear_y, angular_z);
    // int two_steer_drive_ = two_steer_drive(linear_x, linear_y, angular_z);
    

    // RCLCPP_INFO(get_logger(), "Received Twist message: linear_x = %f, angular_z = %f", linear_x, angular_z);
}

int amr_control::ackermannDrive(double linear_x, double angular_z)
{
    RCLCPP_INFO(get_logger(), "Ackermann Config");

    std::vector<double> traction_commands;
    std::vector<double> steering_commands;
    std::vector<int> traction_commands_rpm;

    double v_x = linear_x;
    double v_y = angular_z*Wb;

    auto [is_xy, drive_velocity, steering_angle, rpm] = polar_from_cart(v_x, v_y, angular_z);

    traction_commands = {drive_velocity, drive_velocity};
    steering_commands = {steering_angle, steering_angle};
    traction_commands_rpm = {rpm, rpm};

    if(is_xy)
    {
        R = Wb / tan(steering_angle);

        drive_velocity_right = drive_velocity*(1+(Wt/2.0*R));
        drive_velocity_left = drive_velocity*(1-(Wt/2.0*R));

        double numerator = 2 * Wb * std::sin(steering_angle);
        double denominator_first_member = 2 * Wb * std::cos(steering_angle);
        double denominator_second_member = Wt * std::sin(steering_angle);

        steering_angle_r = std::atan2(numerator, denominator_first_member - denominator_second_member);
        steering_angle_l = std::atan2(numerator, denominator_first_member + denominator_second_member);

        int right_wheel_rpm = getRpm(drive_velocity_right);
        int left_wheel_rpm = getRpm(drive_velocity_left);

        angular_velocity = drive_velocity*tan(steering_angle) / Wb;

        traction_commands = {drive_velocity_right, drive_velocity_left};
        steering_commands = {steering_angle_r, steering_angle_l};
        traction_commands_rpm = {right_wheel_rpm, left_wheel_rpm};
        
        // Print information for illustration
    }

    RCLCPP_INFO(get_logger(), "Turning Radius: %f meters", R);
    RCLCPP_INFO(get_logger(), "Left Wheel Velocity: %f m/s, Right Wheel Velocity: %f m/s", traction_commands.at(1), traction_commands.at(0));
    RCLCPP_INFO(get_logger(), "Left steering_angle: %f radians, Right steering_angle: %f radians", steering_commands.at(1), steering_commands.at(0));
    RCLCPP_INFO(get_logger(), "Left Wheel RPM: %d, Right Wheel RPM: %d", traction_commands_rpm.at(1), traction_commands_rpm.at(0));
    // RCLCPP_INFO(get_logger(), "Angular Velocity (omega): %f rad/s", angular_velocity);
    
    return 0;

}

int amr_control::tricycleDrive_type1(double linear_x, double angular_z) // three wheel (2 drive power wheel and one steer)
{
    RCLCPP_INFO(get_logger(), "TricycleDrive_type1 Config");
    std::vector<double> traction_commands;
    std::vector<double> steering_commands;
    std::vector<int> traction_commands_rpm;
    
    double v_x = linear_x;
    double v_y = angular_z*Wb;

    auto [is_xy, drive_velocity, steering_angle, rpm] = polar_from_cart(v_x, v_y, angular_z);

    // drive_velocity_right = drive_velocity;
    // drive_velocity_left = drive_velocity;
    traction_commands = {drive_velocity, drive_velocity};
    steering_commands = {steering_angle};
    traction_commands_rpm = {rpm, rpm};
    

    if(is_xy)
    {
        R = Wb / tan(steering_angle);

        drive_velocity_right = drive_velocity*(1+(Wt/2.0*R));
        drive_velocity_left = drive_velocity*(1-(Wt/2.0*R));

        int right_wheel_rpm = getRpm(drive_velocity_right);
        int left_wheel_rpm = getRpm(drive_velocity_left);

        angular_velocity = drive_velocity*tan(steering_angle) / Wb;

        traction_commands = {drive_velocity_right, drive_velocity_left};
        steering_commands = {steering_angle};
        traction_commands_rpm = {right_wheel_rpm, left_wheel_rpm};
        
    }
    // Print information for illustration
    RCLCPP_INFO(get_logger(), "Turning Radius: %f meters", R);
    RCLCPP_INFO(get_logger(), "Left Wheel Velocity: %f m/s, Right Wheel Velocity: %f m/s", traction_commands.at(1), traction_commands.at(0));
    RCLCPP_INFO(get_logger(), "Left Wheel RPM: %d, Right Wheel RPM: %d", traction_commands_rpm.at(1), traction_commands_rpm.at(0));
    // RCLCPP_INFO(get_logger(), "Angular Velocity (omega): %f rad/s", angular_velocity);
    return 0;
}

int amr_control::tricycleDrive_type2_bicycle(double linear_x, double angular_z) // three wheel (2 drive wheel two power)
{
    RCLCPP_INFO(get_logger(), "TricycleDrive_type2_bicycle Config");

    std::vector<double> traction_commands;
    std::vector<double> steering_commands;
    std::vector<int> traction_commands_rpm;

    double v_x = linear_x;
    double v_y = angular_z*Wb;

    auto [is_xy, drive_velocity, steering_angle, rpm] = polar_from_cart(v_x, v_y, angular_z);

    traction_commands = {drive_velocity};
    steering_commands = {steering_angle};
    traction_commands_rpm = {rpm};

}


int amr_control::four_wheel_drive(double linear_x, double angular_z)
{
    RCLCPP_INFO(get_logger(), "Four_wheel_drive Config");
    double left_FW_wheel_vel = linear_x - (angular_z * Wt / 2.0);
    double left_BW_wheel_vel = linear_x - (angular_z * Wt / 2.0);
    double right_FW_wheel_vel = linear_x + (angular_z * Wt / 2.0);
    double right_BW_wheel_vel = linear_x + (angular_z * Wt / 2.0);

    double left_FW_wheel_rpm = getRpm(left_FW_wheel_vel);
    double left_BW_wheel_rpm = getRpm(left_BW_wheel_vel);
    double right_FW_wheel_rpm = getRpm(right_FW_wheel_vel);
    double right_BW_wheel_rpm = getRpm(right_BW_wheel_vel);

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

int amr_control::diffDrive(double linear_x, double angular_z)
{
    RCLCPP_INFO(get_logger(), "DiffDrive Config");
    double left_wheel_vel = linear_x - (angular_z * Wt / 2.0);
    double right_wheel_vel = linear_x + (angular_z * Wt / 2.0);

    int left_wheel_rpm = getRpm(left_wheel_vel);
    int right_wheel_rpm = getRpm(right_wheel_vel);

    RCLCPP_INFO(get_logger(), "left_wheel_rpm: %d, right_wheel_rpm: %d", left_wheel_rpm, right_wheel_rpm);
    return 0;
}

int amr_control::omniDrive(double linear_x, double linear_y, double angular_z)
{
    RCLCPP_INFO(get_logger(), "OmniDrive Config");
    double rot = L*angular_z;

    double FO_wheel_vel = linear_y + rot;
    double LO_wheel_vel = -linear_x*sin(M_PI/3.0)-linear_y*cos(M_PI/3.0)+ rot;
    double RO_wheel_vel = linear_x*sin(M_PI/3.0)-linear_y*cos(M_PI/3.0)+ rot;

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

int amr_control::mechDrive(double linear_x, double linear_y, double angular_z)
{
    RCLCPP_INFO(get_logger(), "MechDrive Config");
    double rot = Wt/2.0 + Wb/2.0; // [lx, ly]
    // velocity
    double front_left_vel = (linear_x - linear_y - rot * angular_z);
    double front_right_vel = (linear_x + linear_y + rot * angular_z);
    double back_left_vel = (linear_x + linear_y - rot * angular_z);
    double back_right_vel = (linear_x - linear_y + rot * angular_z);

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
int amr_control::four_steer_drive(double linear_x, double linear_y, double angular_z)
{
    RCLCPP_INFO(get_logger(), "Four_steer_drive Config");
    double d = Wt/2.0;
    double l = Wb/2.0;
    
    double v_x1 = linear_x - d*angular_z;
    double v_y1 = linear_y + l*angular_z;

    double v_x2 = linear_x - d*angular_z;
    double v_y2 = linear_y - l*angular_z;

    double v_x3 = linear_x + d*angular_z;
    double v_y3 = linear_y - l*angular_z;

    double v_x4 = linear_x + d*angular_z;
    double v_y4 = linear_y + l*angular_z;

    // 8x1 states vectors
    auto [is_xy1, v1, th1, rpm1] = polar_from_cart(v_x1, v_y1, angular_z);
    auto [is_xy2, v2, th2, rpm2] = polar_from_cart(v_x2, v_y2, angular_z);
    auto [is_xy3, v3, th3, rpm3] = polar_from_cart(v_x3, v_y3, angular_z);
    auto [is_xy4, v4, th4, rpm4] = polar_from_cart(v_x4, v_y4, angular_z);

    // // Print the variables
    RCLCPP_INFO(get_logger(), "v1: %f, th1: %f, rpm1: %d", v1, th1, rpm1);
    RCLCPP_INFO(get_logger(), "v2: %f, th2: %f, rpm2: %d", v2, th2, rpm2);
    RCLCPP_INFO(get_logger(), "v3: %f, th3: %f, rpm3: %d", v3, th3, rpm3);
    RCLCPP_INFO(get_logger(), "v4: %f, th4: %f, rpm4: %d", v4, th4, rpm4);

}

int amr_control::two_steer_drive(double linear_x, double linear_y, double angular_z)
{
    RCLCPP_INFO(get_logger(), "Two_steer_drive Config");
    double d = 0.0; // no track distance 
    double l = Wb/2.0;

    double v_x1 = linear_x;
    double v_y1 = linear_y + l*angular_z;

    double v_x2 = linear_x;
    double v_y2 = linear_y - l*angular_z;

    // 8x1 states vectors
    auto [is_xy1, v1, th1, rpm1] = polar_from_cart(v_x1, v_y1, angular_z);
    auto [is_xy2, v2, th2, rpm2] = polar_from_cart(v_x2, v_y2, angular_z);

    // // Print the variables
    RCLCPP_INFO(get_logger(), "v1: %f, th1: %f, rpm1: %d", v1, th1, rpm1);
    RCLCPP_INFO(get_logger(), "v2: %f, th2: %f, rpm2: %d", v2, th2, rpm2);
}

std::tuple<bool, double, double, int> amr_control::polar_from_cart(double x,double y, double angular_z)
{
    double steering_angle;
    double drive_velocity;
    int drive_rpm;
    bool is_xy;

    if (x==0.0 && y == 0.0)
    {
        steering_angle = 0.0;
        drive_velocity = 0.0;
        drive_rpm = getRpm(drive_velocity);
        is_xy = 0;
        
    }
    if (x==0.0 && y != 0.0)
    {
        steering_angle = angular_z > 0 ? M_PI_2 : -M_PI_2;
        drive_velocity = hypot(x,y);
        drive_rpm = getRpm(drive_velocity);
        is_xy = 0;
    }
    if (x!=0.0 && y == 0.0)
    {
        steering_angle = 0.0;
        drive_velocity = hypot(x,y);
        drive_rpm = getRpm(drive_velocity);
        is_xy = 0;
    }
    if(x!=0.0 && y != 0.0)
    {
        steering_angle = atan(y/x);
        drive_velocity = hypot(x,y);
        drive_rpm = getRpm(drive_velocity);
        is_xy = 1;
    }

    
    RCLCPP_INFO(get_logger(), "Steering Angle: %f radians", steering_angle);
    RCLCPP_INFO(get_logger(), "Drive Velocity: %f m/s", drive_velocity);
    RCLCPP_INFO(get_logger(), "Drive Velocity (RPM): %d RPM", drive_rpm);
    return std::make_tuple(is_xy, steering_angle, drive_velocity, drive_rpm);

}

int amr_control::getRpm(double linear_vel)
{
    int rpm = static_cast<int>((linear_vel * 60) / (2 * M_PI * wheel_radius));
    rpm = std::min(255, std::max(-255, rpm));
    return rpm;
}

amr_control::~amr_control()
{
    // Cleanup, if needed
    CmdVelSub.reset();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_control>());
  rclcpp::shutdown();
  return 0;
}