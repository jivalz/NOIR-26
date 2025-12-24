#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
// #include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"

#include <iostream>

class gps : public rclcpp::Node
{
public:
    gps() : Node("gps_data_publisher")
    {
        sub1 = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10, std::bind(&gps::latlon, this, std::placeholders::_1));

        sub2 = this->create_subscription<std_msgs::msg::Int32>(
            "/imu_data", 10, std::bind(&gps::imu_callback, this, std::placeholders::_1));

        timeryaw = this->create_wall_timer(
            std::chrono::milliseconds(999),
            std::bind(&gps::timer_yaw_publish, this));

        pub = this->create_publisher<geometry_msgs::msg::Twist>("/gps_data", 10);
    }

private:

    std_msgs::msg::Int32 last_imu;   // store latest IMU

    void latlon(const sensor_msgs::msg::NavSatFix::SharedPtr gpsmsg)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = gpsmsg->latitude;
        msg.linear.y = gpsmsg->longitude;

        pub->publish(msg);

        printf("Latitude: %.6f  Longitude: %.6f\n", msg.linear.x, msg.linear.y);
    }

    void imu_callback(const std_msgs::msg::Int32::SharedPtr imumsg)
    {
        last_imu = *imumsg;    // store newest IMU
    }

    void timer_yaw_publish()
    {
        auto msg = geometry_msgs::msg::Twist();

        msg.linear.z = last_imu.data;

        pub->publish(msg);

        printf("Yaw (slow): %.6f\n", msg.linear.z);
    }


    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub1;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timeryaw;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gps>());
    rclcpp::shutdown();
    return 0;
}
