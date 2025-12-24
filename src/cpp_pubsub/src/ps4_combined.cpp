#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>
using namespace std;

class ttlcode : public rclcpp::Node
{
public:
    ttlcode() : Node("dual_ps4_node")
    {
        pub_drive = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);
        pub_arm = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel2", 10);

        sub_drive = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy1", 10, std::bind(&ttlcode::joy1, this, std::placeholders::_1));

        sub_arm = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy2", 10, std::bind(&ttlcode::joy2, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Dual PS4 control node started!");
    }

private:
    void joy1(sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto v_drive = geometry_msgs::msg::Twist();
        v_drive.linear.x = msg->axes[1];   // left stick vertical
        v_drive.angular.z = msg->axes[3];  // right stick horizontal
        pub_drive->publish(v_drive);
    }

    void joy2(sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto v_arm = geometry_msgs::msg::Twist();

        v_arm.linear.x  = msg->axes[1]; // Gripper
        v_arm.linear.y  = msg->axes[4]; // Actuator
        v_arm.linear.z  = msg->axes[7]; // Wrist
        v_arm.angular.x = msg->axes[6]; // Elbow

        // joint5: square = -1, circle = +1
        bool square = msg->buttons[3];
        bool circle = msg->buttons[1];
        v_arm.angular.y = square ? -1.0 : (circle ? 1.0 : 0.0);

        // joint4: cross (X) = -1, triangle = +1
        bool cross = msg->buttons[0];
        bool triangle = msg->buttons[2];
        v_arm.angular.z = cross ? (-0.15) : (triangle ? (0.15) : 0.0);

        pub_arm->publish(v_arm);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_drive;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_arm;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_drive;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_arm;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ttlcode>());
    rclcpp::shutdown();
    return 0;
}
