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

int robot_type::ackermannDrive(float linear_x, float angular_z)
{
    RCLCPP_INFO(get_logger(), "Ackermann Config");

    std::vector<float> traction_commands;
    std::vector<float> steering_commands;
    std::vector<int> traction_commands_rpm;

    float v_x = linear_x;
    float v_y = angular_z*Wb;

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

int robot_type::tricycleDrive_type1(float linear_x, float angular_z) // three wheel (2 drive power wheel and one steer)
{
    RCLCPP_INFO(get_logger(), "TricycleDrive_type1 Config");
    std::vector<float> traction_commands;
    std::vector<float> steering_commands;
    std::vector<int> traction_commands_rpm;
    
    float v_x = linear_x;
    float v_y = angular_z*Wb;

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

int robot_type::tricycleDrive_type2_bicycle(float linear_x, float angular_z) // three wheel (2 drive wheel two power)
{
    RCLCPP_INFO(get_logger(), "TricycleDrive_type2_bicycle Config");

    std::vector<float> traction_commands;
    std::vector<float> steering_commands;
    std::vector<int> traction_commands_rpm;

    float v_x = linear_x;
    float v_y = angular_z*Wb;

    auto [is_xy, drive_velocity, steering_angle, rpm] = polar_from_cart(v_x, v_y, angular_z);

    traction_commands = {drive_velocity};
    steering_commands = {steering_angle};
    traction_commands_rpm = {rpm};

}


int robot_type::four_wheel_drive(float linear_x, float angular_z)
{
    RCLCPP_INFO(get_logger(), "Four_wheel_drive Config");
    float left_FW_wheel_vel = linear_x - (angular_z * Wt / 2.0);
    float left_BW_wheel_vel = linear_x - (angular_z * Wt / 2.0);
    float right_FW_wheel_vel = linear_x + (angular_z * Wt / 2.0);
    float right_BW_wheel_vel = linear_x + (angular_z * Wt / 2.0);

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
    RCLCPP_INFO(get_logger(), "DiffDrive Config");
    float left_wheel_vel = linear_x - (angular_z * Wt / 2.0);
    float right_wheel_vel = linear_x + (angular_z * Wt / 2.0);

    int left_wheel_rpm = getRpm(left_wheel_vel);
    int right_wheel_rpm = getRpm(right_wheel_vel);

    RCLCPP_INFO(get_logger(), "left_wheel_rpm: %d, right_wheel_rpm: %d", left_wheel_rpm, right_wheel_rpm);
    return 0;
}

int robot_type::omniDrive(float linear_x, float linear_y, float angular_z)
{
    RCLCPP_INFO(get_logger(), "OmniDrive Config");
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
    RCLCPP_INFO(get_logger(), "MechDrive Config");
    float rot = Wt/2.0 + Wb/2.0; // [lx, ly]
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
int robot_type::four_steer_drive(float linear_x, float linear_y, float angular_z)
{
    RCLCPP_INFO(get_logger(), "Four_steer_drive Config");
    float d = Wt/2.0;
    float l = Wb/2.0;
    
    float v_x1 = linear_x - d*angular_z;
    float v_y1 = linear_y + l*angular_z;

    float v_x2 = linear_x - d*angular_z;
    float v_y2 = linear_y - l*angular_z;

    float v_x3 = linear_x + d*angular_z;
    float v_y3 = linear_y - l*angular_z;

    float v_x4 = linear_x + d*angular_z;
    float v_y4 = linear_y + l*angular_z;

    // 8x1 states vectors
    auto [is_xy1, v1, th1, rpm1] = polar_from_cart(v_x1, v_y1, angular_z);
    auto [is_xy2, v2, th2, rpm2] = polar_from_cart(v_x2, v_y2, angular_z);
    auto [is_xy3, v3, th3, rpm3] = polar_from_cart(v_x3, v_y3, angular_z);
    auto [is_xy4, v4, th4, rpm4] = polar_from_cart(v_x4, v_y4, angular_z);

    // // Print the variables
    // RCLCPP_INFO(get_logger(), "v1: %f, th1: %f, rpm1: %d", v1, th1, rpm1);
    // RCLCPP_INFO(get_logger(), "v2: %f, th2: %f, rpm2: %d", v2, th2, rpm2);
    // RCLCPP_INFO(get_logger(), "v3: %f, th3: %f, rpm3: %d", v3, th3, rpm3);
    // RCLCPP_INFO(get_logger(), "v4: %f, th4: %f, rpm4: %d", v4, th4, rpm4);

}

int robot_type::two_steer_drive(float linear_x, float linear_y, float angular_z)
{
    RCLCPP_INFO(get_logger(), "Two_steer_drive Config");
    float d = 0.0; // no track distance 
    float l = Wb/2.0;

    float v_x1 = linear_x;
    float v_y1 = linear_y + l*angular_z;

    float v_x2 = linear_x;
    float v_y2 = linear_y - l*angular_z;

    // 8x1 states vectors
    auto [is_xy1, v1, th1, rpm1] = polar_from_cart(v_x1, v_y1, angular_z);
    auto [is_xy2, v2, th2, rpm2] = polar_from_cart(v_x2, v_y2, angular_z);

    // // Print the variables
    RCLCPP_INFO(get_logger(), "v1: %f, th1: %f, rpm1: %d", v1, th1, rpm1);
    RCLCPP_INFO(get_logger(), "v2: %f, th2: %f, rpm2: %d", v2, th2, rpm2);
}

std::tuple<bool, float, float, int> robot_type::polar_from_cart(float x,float y, float angular_z)
{
    float steering_angle;
    float drive_velocity;
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