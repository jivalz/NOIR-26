#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>
using namespace std;

class ttlcode : public rclcpp::Node
{
public:
    ttlcode() : Node("lidar")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("ldlidar_node/scan", 10, std::bind(&ttlcode::obs, this, std::placeholders::_1));
    }

private:
    void obs(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto tms = geometry_msgs::msg::Twist();
        double detection_threshold = 2.0;  // Detect obstacles within 2m
        double stop_threshold = 0.3;       // Stop when 0.3m away
        double linvel = 0.15;              // Linear velocity
        double rotvel = 0.5;               // Rotation velocity for searching
        double kpz = 0.5;                  // Angular gain for turning towards obstacle
        
        int stid = ((-75.0 - msg->angle_min) / msg->angle_increment);
        int endid = ((75.0 - msg->angle_min) / msg->angle_increment);
        stid = std::max(0, stid);
        endid = std::min(static_cast<int>(msg->ranges.size()) - 1, endid);
        int mid = stid + (endid - stid) / 2;
        
        float ld = msg->range_max;
        float rd = msg->range_max;
        
        // Find minimum distance on left side
        for (int i = stid; i <= mid; i++)
        {
            float instdis = msg->ranges[i];
            if (instdis > 0.0 && instdis < ld)
            {
                ld = instdis;
            }
        }
        
        // Find minimum distance on right side
        for (int j = mid + 1; j <= endid; j++)
        {
            float instdis = msg->ranges[j];
            if (instdis > 0.0 && instdis < rd)
            {
                rd = instdis;
            }
        }
        
        float closest_distance = std::min(ld, rd);
        
        // Check if obstacle is detected within detection threshold
        bool obstacle_detected = (closest_distance < detection_threshold);
        
        if (!obstacle_detected)
        {
            // No obstacle detected - rotate 360 to search
            tms.linear.x = 0.0;
            tms.angular.z = rotvel;
            cout << "No obstacle detected. Searching..." << endl;
        }
        else if (closest_distance <= stop_threshold)
        {
            // Close enough - stop
            tms.linear.x = 0.0;
            tms.angular.z = 0.0;
            cout << "Reached obstacle. Stopping at distance: " << closest_distance << "m" << endl;
        }
        else
        {
            // Obstacle detected but not close enough - move towards it
            tms.linear.x = linvel;
            
            // Turn towards the closer obstacle
            if (ld < rd)
            {
                // Obstacle on left - turn left (positive angular z)
                tms.angular.z = kpz * (detection_threshold - ld);
            }
            else
            {
                // Obstacle on right - turn right (negative angular z)
                tms.angular.z = -kpz * (detection_threshold - rd);
            }
            
            cout << "Moving towards obstacle. Left: " << ld << "m, Right: " << rd << "m" << endl;
        }
        
        pub_->publish(tms);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ttlcode>());
    rclcpp::shutdown();
    return 0;
}