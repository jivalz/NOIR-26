#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <iostream>
#include <cmath>

class gps : public rclcpp::Node
{
public:
    gps() : Node("gps_data_publisher")
    {
        sub_fix = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10, std::bind(&gps::gps_callback, this, std::placeholders::_1));

        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&gps::imu_callback, this, std::placeholders::_1));

        // timer every 200 ms to run go-to-goal
        timer_nav = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&gps::go_to_goal, this));

        pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);

        // Set reference origin (your launch point)
        ref_lat = 21.167085;
        ref_lon = 72.785789;

        ask_goal();
    }

private:
    // ------------------------------------------
    // VARIABLES
    // ------------------------------------------
    double ref_lat, ref_lon;           // reference (origin)
    double cur_lat = 0, cur_lon = 0;   // current GPS
    double goal_lat = 0, goal_lon = 0; // goal GPS

    double yaw = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
    rclcpp::TimerBase::SharedPtr timer_nav;

    // ------------------------------------------
    // ASK USER FOR GOAL
    // ------------------------------------------
    void ask_goal()
    {
        std::cout << "Enter GOAL latitude: ";
        std::cin >> goal_lat;

        std::cout << "Enter GOAL longitude: ";
        std::cin >> goal_lon;

        std::cout << "Goal Set: " << goal_lat << ", " << goal_lon << std::endl;
    }

    // ------------------------------------------
    // latlon → local X,Y (ENU)
    // ------------------------------------------
    void latlon_to_xy(double lat, double lon, double &x, double &y)
    {
        double R = 6378137.0;

        double dLat = (lat - ref_lat) * M_PI / 180;
        double dLon = (lon - ref_lon) * M_PI / 180;

        x = R * dLon * cos(ref_lat * M_PI / 180.0);
        y = R * dLat;
    }

    // ------------------------------------------
    // GPS CALLBACK
    // ------------------------------------------
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        cur_lat = msg->latitude;
        cur_lon = msg->longitude;
    }

    // ------------------------------------------
    // IMU CALLBACK
    // ------------------------------------------
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // yaw extracted from angular velocity z is wrong for heading
        // here assume orientation not available → keep your method
        yaw = msg->angular_velocity.z;
    }

    // ------------------------------------------
    // MOVE FUNCTION FROM YOU
    // ------------------------------------------
    void move(double cx, double cy, double theta)
    {
        auto twist = geometry_msgs::msg::Twist();

        double gx, gy;
        latlon_to_xy(goal_lat, goal_lon, gx, gy);

        double dist = std::sqrt(std::pow(gx - cx, 2) + std::pow(gy - cy, 2));
        double angle = std::atan2((gy - cy), (gx - cx));

        if (dist > 0.1)
        {
            twist.linear.x = 1.0 * dist;
            twist.angular.z = (angle - theta) * 4.0;
        }
        else
        {
            std::cout << "Goal reached! Enter new goal.\n";
            ask_goal();
        }

        pub_cmd->publish(twist);
    }

    // ------------------------------------------
    // GO TO GOAL LOOP (runs every 200ms)
    // ------------------------------------------
    void go_to_goal()
    {
        double cx, cy;
        latlon_to_xy(cur_lat, cur_lon, cx, cy);

        move(cx, cy, yaw);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gps>());
    rclcpp::shutdown();
    return 0;
}
