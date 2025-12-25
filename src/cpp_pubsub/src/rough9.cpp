#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"

#include <geographic_msgs/msg/geo_point.hpp>
#include <geodesy/utm.h>
#include <cmath>
#include <iostream>
#include <limits>

class gps_controller_cin : public rclcpp::Node
{
public:
    gps_controller_cin() : Node("gps_controller_cin")
    {
        /* -------- GOAL INPUT (ONCE) -------- */
        std::cout << "Enter goal latitude  : ";
        std::cin >> goal_geo_.latitude;
        std::cout << "Enter goal longitude : ";
        std::cin >> goal_geo_.longitude;

        goal_geo_.altitude = 0.0;

        geodesy::UTMPoint goal_utm;
        geodesy::fromMsg(goal_geo_, goal_utm);
        goal_e_ = goal_utm.easting;
        goal_n_ = goal_utm.northing;

        RCLCPP_INFO(get_logger(), "Goal locked, starting navigation");

        /* -------- SUBSCRIBERS -------- */
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/fix", 10,
            std::bind(&gps_controller_cin::gps_callback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<std_msgs::msg::Int32>("/imu_data", 10,
            std::bind(&gps_controller_cin::imu_callback, this, std::placeholders::_1));

        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/ldlidar_node/scan", 10,
            std::bind(&gps_controller_cin::lidar_callback, this, std::placeholders::_1));

        /* -------- PUBLISHERS -------- */
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);
    }

private:
    /* -------- ROS HANDLES -------- */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    /* -------- STATE -------- */
    geographic_msgs::msg::GeoPoint goal_geo_;
    double goal_e_, goal_n_;
    double imu_heading_rad_{0.0};

    float left_min_{10.0};
    float right_min_{10.0};
    float front_min_{10.0};
    bool obstacle_{false};

    // State machine for cone detection
    enum class State {
        NAVIGATE_TO_GOAL,
        ROTATING_TO_SCAN,
        APPROACHING_CONE
    };
    State current_state_{State::NAVIGATE_TO_GOAL};
    
    double scan_start_heading_{0.0};
    double cone_heading_{0.0};
    float cone_distance_{0.0};
    bool cone_detected_{false};

    /* -------- PARAMETERS -------- */
    const float OBSTACLE_THRESHOLD = 1.0;      // Detection distance
    const float CRITICAL_DISTANCE = 0.3;       // Emergency stop distance
    const float AVOIDANCE_GAIN = 1.3;          // How aggressively to avoid
    const float GOAL_GAIN = 1.0;               // How much to bias toward goal
    const float CONE_SEARCH_RADIUS = 2.0;      // Start cone search within 2m
    const float CONE_REACH_DISTANCE = 0.3;     // Stop when this close to cone
    const float ROTATION_SPEED = 0.5;          // Angular velocity for 360° scan

    /* -------- UTILS -------- */
    double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    /* -------- CALLBACKS -------- */

    void imu_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        imu_heading_rad_ = msg->data * M_PI / 180.0;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        left_min_  = msg->range_max;
        right_min_ = msg->range_max;
        front_min_ = msg->range_max;

        int total = msg->ranges.size();
        int third = total / 3;

        // Left sector (0 to 1/3)
        for (int i = 0; i < third; i++)
        {
            float d = msg->ranges[i];
            if (d > 0.05 && d < left_min_){
                left_min_ = d;
            }
        }

        // Front sector (1/3 to 2/3)
        for (int i = third; i < 2 * third; i++)
        {
            float d = msg->ranges[i];
            if (d > 0.05 && d < front_min_){
                front_min_ = d;
            }
        }

        // Right sector (2/3 to end)
        for (size_t i = 2 * third; i < total; i++)
        {
            float d = msg->ranges[i];
            if (d > 0.05 && d < right_min_)
            {
                right_min_ = d;
            }
        }

        obstacle_ = (left_min_ < OBSTACLE_THRESHOLD || right_min_ < OBSTACLE_THRESHOLD || front_min_ < OBSTACLE_THRESHOLD);
        
        // During rotation scan, detect closest obstacle (cone)
        if (current_state_ == State::ROTATING_TO_SCAN && obstacle_)
        {
            if (!cone_detected_ || front_min_ < cone_distance_)
            {
                cone_detected_ = true;
                cone_distance_ = front_min_;
                cone_heading_ = imu_heading_rad_;
                RCLCPP_INFO(get_logger(), "Cone detected at %.2fm, heading %.1f°", 
                           cone_distance_, cone_heading_ * 180.0 / M_PI);
            }
        }
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;

        /* -------- CURRENT POSITION -------- */
        geographic_msgs::msg::GeoPoint curr_geo;
        curr_geo.latitude  = msg->latitude;
        curr_geo.longitude = msg->longitude;
        curr_geo.altitude  = msg->altitude;

        geodesy::UTMPoint curr_utm;
        geodesy::fromMsg(curr_geo, curr_utm);

        double dx = goal_e_ - curr_utm.easting;
        double dy = goal_n_ - curr_utm.northing;
        double dist = std::hypot(dx, dy);

        /* -------- GOAL HEADING -------- */
        double desired_heading = atan2(dx, dy);
        double angle_error = normalize_angle(-(desired_heading - imu_heading_rad_));

        /* -------- STATE MACHINE -------- */
        
        if (current_state_ == State::NAVIGATE_TO_GOAL)
        {
            // Check if within 2m of goal - start cone search
            if (dist < CONE_SEARCH_RADIUS)
            {
                current_state_ = State::ROTATING_TO_SCAN;
                scan_start_heading_ = imu_heading_rad_;
                cone_detected_ = false;
                cone_distance_ = 100.0;
                RCLCPP_INFO(get_logger(), "Within 2m of goal. Starting 360° scan for cone...");
                cmd.linear.x = 0.0;
                cmd.angular.z = ROTATION_SPEED;
                cmd_pub_->publish(cmd);
                return;
            }

            // Normal navigation with obstacle avoidance
            if (obstacle_)
            {
                // Calculate repulsive forces from obstacles
                double avoidance_angle = 0.0;
                
                if (front_min_ < OBSTACLE_THRESHOLD) {
                    if (left_min_ < right_min_) {
                        avoidance_angle = -1.0;  // Turn right
                    } else {
                        avoidance_angle = 1.0;   // Turn left
                    }
                } else {
                    double left_repulsion = (OBSTACLE_THRESHOLD - left_min_) / OBSTACLE_THRESHOLD;
                    double right_repulsion = (OBSTACLE_THRESHOLD - right_min_) / OBSTACLE_THRESHOLD;
                    avoidance_angle = (right_repulsion - left_repulsion);
                }

                double goal_weight = GOAL_GAIN;
                double avoidance_weight = AVOIDANCE_GAIN;
                
                if (front_min_ < CRITICAL_DISTANCE || 
                    left_min_ < CRITICAL_DISTANCE || 
                    right_min_ < CRITICAL_DISTANCE) {
                    goal_weight = 0.2;
                    avoidance_weight = 2.0;
                    cmd.linear.x = 0.05;
                } else {
                    cmd.linear.x = 0.15;
                }

                double goal_turn = clamp(angle_error / M_PI, -1.0, 1.0);
                cmd.angular.z = (goal_weight * goal_turn + 
                                avoidance_weight * avoidance_angle);
                cmd.angular.z = clamp(cmd.angular.z, -1.0, 1.0);

                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Avoiding | L:%.2f F:%.2f R:%.2f | Goal angle:%.1f° Turn:%.2f",
                    left_min_, front_min_, right_min_, 
                    angle_error * 180.0 / M_PI, cmd.angular.z);
            }
            else
            {
                double ang = clamp(angle_error / M_PI, -1.0, 1.0);
                
                if(dist > 5.0){
                    cmd.linear.x = 1.0;
                } else {
                    cmd.linear.x = clamp(dist / 5.0, 0.0, 1.0);
                }
                cmd.angular.z = ang;
            }
        }
        else if (current_state_ == State::ROTATING_TO_SCAN)
        {
            // Continue rotating until 360° complete
            double rotated = normalize_angle(imu_heading_rad_ - scan_start_heading_);
            
            if (std::abs(rotated) >= 2.0 * M_PI - 0.2)  // Nearly 360°
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                
                if (cone_detected_)
                {
                    current_state_ = State::APPROACHING_CONE;
                    RCLCPP_INFO(get_logger(), "360° scan complete. Approaching cone at %.2fm", cone_distance_);
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "360° scan complete. No cone detected. Stopping.");
                    cmd_pub_->publish(cmd);
                    rclcpp::shutdown();
                    return;
                }
            }
            else
            {
                // Keep rotating
                cmd.linear.x = 0.0;
                cmd.angular.z = ROTATION_SPEED;
            }
        }
        else if (current_state_ == State::APPROACHING_CONE)
        {
            // Turn towards cone and approach
            double cone_angle_error = normalize_angle(cone_heading_ - imu_heading_rad_);
            
            // Check if reached cone
            if (front_min_ < CONE_REACH_DISTANCE)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                RCLCPP_INFO(get_logger(), "Cone reached! Mission complete.");
                cmd_pub_->publish(cmd);
                rclcpp::shutdown();
                return;
            }
            
            // Move towards cone
            if (std::abs(cone_angle_error) > 0.1)
            {
                // Need to adjust heading
                cmd.linear.x = 0.1;
                cmd.angular.z = clamp(cone_angle_error * 2.0, -0.5, 0.5);
            }
            else
            {
                // Heading is good, move forward
                cmd.linear.x = 0.3;
                cmd.angular.z = 0.0;
            }
            
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "Approaching cone | Distance: %.2fm | Angle error: %.1f°",
                front_min_, cone_angle_error * 180.0 / M_PI);
        }
        
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gps_controller_cin>());
    rclcpp::shutdown();
    return 0;
}