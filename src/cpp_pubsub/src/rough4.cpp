#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/int32.hpp"

#include <geographic_msgs/msg/geo_point.hpp>
#include <geodesy/utm.h>
#include <cmath>

class gps_controller : public rclcpp::Node
{
public:
    gps_controller() : Node("gps_controller")
    {
        // -------- PARAMETERS --------
        this->declare_parameter<double>("goal_lat", 0.0);
        this->declare_parameter<double>("goal_lon", 0.0);
        this->declare_parameter<double>("max_lin_dist", 5.0);
        this->declare_parameter<double>("reach_thresh", 0.3);

        // -------- SUBSCRIBERS --------
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10,
            std::bind(&gps_controller::gps_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/imu_data", 10,
            std::bind(&gps_controller::imu_callback, this, std::placeholders::_1));

        // -------- PUBLISHERS --------
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);
        enu_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/gps/enu", 10);

        RCLCPP_INFO(this->get_logger(), "GPS Controller Node Started");
    }

private:
    // -------- ROS HANDLES --------
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr enu_pub_;

    // -------- STATE --------
    bool origin_set_ = false;
    double goal_e_, goal_n_;
    std_msgs::msg::Int32 last_imu_;

    // -------- UTIL --------
    double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    // -------- CALLBACKS --------
    void imu_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        last_imu_ = *msg;   // degrees (0–360)
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        double goal_lat = this->get_parameter("goal_lat").as_double();
        double goal_lon = this->get_parameter("goal_lon").as_double();
        double max_lin_dist = this->get_parameter("max_lin_dist").as_double();
        double reach_thresh = this->get_parameter("reach_thresh").as_double();

        if (goal_lat == 0.0 || goal_lon == 0.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for goal_lat & goal_lon parameters");
            return;
        }

        // -------- CURRENT POSITION --------
        geographic_msgs::msg::GeoPoint curr_geo;
        curr_geo.latitude  = msg->latitude;
        curr_geo.longitude = msg->longitude;
        curr_geo.altitude  = msg->altitude;

        geodesy::UTMPoint curr_utm;
        geodesy::fromMsg(curr_geo, curr_utm);

        // -------- SET GOAL ORIGIN --------
        if (!origin_set_) {
            geographic_msgs::msg::GeoPoint goal_geo;
            goal_geo.latitude  = goal_lat;
            goal_geo.longitude = goal_lon;
            goal_geo.altitude  = 0.0;

            geodesy::UTMPoint goal_utm;
            geodesy::fromMsg(goal_geo, goal_utm);

            goal_e_ = goal_utm.easting;
            goal_n_ = goal_utm.northing;
            origin_set_ = true;

            RCLCPP_INFO(get_logger(), "Goal UTM set");
        }

        // -------- CONTROL --------
        double dx = goal_e_ - curr_utm.easting;
        double dy = goal_n_ - curr_utm.northing;
        double dist = std::hypot(dx, dy);

        double desired_heading = atan2(dx, dy);
        double imu_heading = last_imu_.data * M_PI / 180.0;

        double angle_error = atan2(
            sin(desired_heading - imu_heading),
            cos(desired_heading - imu_heading)
        );

        // -------- NORMALIZE [-1, 1] --------
        double lin = dist / max_lin_dist;
        double ang = angle_error / M_PI;

        lin = clamp(lin, -1.0, 1.0);
        ang = clamp(ang, -1.0, 1.0);

        geometry_msgs::msg::Twist cmd;

        if (dist < reach_thresh) {
            cmd.linear.x  = 0.0;
            cmd.angular.z = 0.0;
            RCLCPP_INFO(get_logger(), "Reached Goal");
        } else {
            cmd.linear.x  = lin;
            cmd.angular.z = ang;
        }

        // -------- DEBUG ENU --------
        geometry_msgs::msg::Twist enu;
        enu.linear.x = dx;
        enu.linear.y = dy;
        enu.angular.x = angle_error;
        enu.angular.y = dist;

        enu_pub_->publish(enu);
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gps_controller>());
    rclcpp::shutdown();
    return 0;
}
