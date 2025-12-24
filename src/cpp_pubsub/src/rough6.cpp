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

/*ros2 run nmea_navsat_driver nmea_serial_driver   --ros-args -p port:=/dev/ttyUSB1 -p baud:=9600 -p use_gnss:=true //GPS
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 //magn or imu
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB2 // drive 
  ros2 launch ldlidar_node ldlidar_rviz2.launch.py //lidar
*/


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
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/fix", 10,std::bind(&gps_controller_cin::gps_callback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<std_msgs::msg::Int32>("/imu_data", 10,std::bind(&gps_controller_cin::imu_callback, this, std::placeholders::_1));

        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/ldlidar_node/scan", 10,std::bind(&gps_controller_cin::lidar_callback, this, std::placeholders::_1));

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
    bool obstacle_{false};

    /* -------- UTILS -------- */
    double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
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

        int mid = msg->ranges.size() / 2;

        for (int i = 0; i < mid; i++)
        {
            float d = msg->ranges[i];
            if (d > 0.05 && d < left_min_){
                left_min_ = d;
            }
        }

        for (size_t i = mid; i < msg->ranges.size(); i++)
        {
            float d = msg->ranges[i];
            if (d > 0.05 && d < right_min_)
            {
                right_min_ = d;
            }
        }

        obstacle_ = (left_min_ < 0.5 || right_min_ < 0.5);
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
        double angle_error = atan2(
            sin(desired_heading - imu_heading_rad_),
            cos(desired_heading - imu_heading_rad_));

        /* -------- GOAL REACHED -------- */
        if (dist < 0.3)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Goal Reached");
            cmd_pub_->publish(cmd);
            return;
        }

        /* -------- OBSTACLE AVOIDANCE -------- */
        if (obstacle_)
        {
            cmd.linear.x = 0.1;

            if (left_min_ < right_min_){
                cmd.angular.z = -0.6;
            }
            else{
                cmd.angular.z = 0.6;
            }

            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,"Obstacle detected | L: %.2f R: %.2f",left_min_, right_min_);
        }
        else
        {
            /* -------- GO TO GOAL -------- */
            double ang = clamp(angle_error / M_PI, -1.0, 1.0);
            if(dist > 5.0){
                cmd.linear.x = 1.0;
            }else{
                cmd.linear.x = clamp(dist / 5.0, 0.0, 1.0);
            }
            cmd.angular.z = ang;
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
