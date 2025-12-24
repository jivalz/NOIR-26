#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <limits>
#include <cmath>

class LidarMinDistance : public rclcpp::Node
{
public:
    LidarMinDistance() : Node("lidar_min_distance")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/ldlidar_node/scan",
            10,
            std::bind(&LidarMinDistance::scanCallback, this, std::placeholders::_1)
        );
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float min_distance = std::numeric_limits<float>::infinity();
        float angle_at_min = 0.0;

        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            float d = msg->ranges[i];
            // float tol = 2.0;

            if (d < min_distance)
            {
                min_distance = d;
                angle_at_min = msg->angle_min + i * msg->angle_increment;
            }
        }

        // Convert rad → deg
        float angle_deg = angle_at_min * 180.0 / M_PI;
        if (angle_deg > 328.0)
        angle_deg = angle_deg - 360.0;

        // if (angle_deg < 0) angle_deg += 360.0;

        printf("Min distance = %.3f cm at angle = %.1f deg \n",min_distance*100, angle_deg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMinDistance>());
    rclcpp::shutdown();
    return 0;
}
