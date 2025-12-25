#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <limits>

class ObjectAttraction : public rclcpp::Node
{
public:
    ObjectAttraction() : Node("object_attraction")
    {
        RCLCPP_INFO(get_logger(), "Object Attraction Node Started");

        /* -------- SUBSCRIBER -------- */
        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/ldlidar_node/scan", 10,
            std::bind(&ObjectAttraction::lidar_callback, this, std::placeholders::_1));

        /* -------- PUBLISHER -------- */
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);
    }

private:
    /* -------- ROS HANDLES -------- */
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    /* -------- PARAMETERS -------- */
    const float ATTRACTION_THRESHOLD = 2.0;    // Detect objects within 2 meters
    const float MIN_SAFE_DISTANCE = 0.2;       // Stop approaching at 20cm
    const float MAX_LINEAR_SPEED = 0.3;        // Maximum forward speed
    const float MAX_ANGULAR_SPEED = 0.3;       // Maximum turn speed

    /* -------- UTILS -------- */
    double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    /* -------- CALLBACK -------- */
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;

        float closest_dist = msg->range_max;
        int closest_idx = -1;
        int total = msg->ranges.size();

        // Find the closest object within attraction threshold
        for (size_t i = 0; i < total; i++)
        {
            float d = msg->ranges[i];
            if (d > 0.05 && d < closest_dist && d < ATTRACTION_THRESHOLD)
            {
                closest_dist = d;
                closest_idx = i;
            }
        }

        // If object detected within range
        if (closest_idx != -1)
        {
            // Calculate angle to object
            float angle_per_scan = (msg->angle_max - msg->angle_min) / total;
            float object_angle = msg->angle_min + closest_idx * angle_per_scan;

            // If too close, stop
            if (closest_dist < MIN_SAFE_DISTANCE)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Too close! Distance: %.2fm - STOPPED", closest_dist);
            }
            else
            {
                // Turn toward the object
                cmd.angular.z = clamp(object_angle * 2.0, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

                // Move forward with speed proportional to distance
                double speed_factor = (closest_dist - MIN_SAFE_DISTANCE) / 
                                    (ATTRACTION_THRESHOLD - MIN_SAFE_DISTANCE);
                cmd.linear.x = clamp(speed_factor * MAX_LINEAR_SPEED, 0.05, MAX_LINEAR_SPEED);

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "Attracted to object | Distance: %.2fm | Angle: %.1f° | Speed: %.2f",
                    closest_dist, object_angle * 180.0 / M_PI, cmd.linear.x);
            }
        }
        else
        {
            // No object detected - stop or rotate to search
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.3;  // Slow rotation to search for objects
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "No object detected within 2m - Searching...");
        }

        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectAttraction>());
    rclcpp::shutdown();
    return 0;
}
