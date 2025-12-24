#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>
using namespace std;

class ttlcode : public rclcpp::Node
{
    public:
    ttlcode() : Node("ps4")
    {
        pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1",10);
        sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy",10,std::bind(&ttlcode::joy,this,std::placeholders::_1));
    }
    private:
    void joy(sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto v = geometry_msgs::msg::Twist();
        v.linear.x = msg->axes[1] ;
        v.angular.z = msg->axes[3];
        pub->publish(v);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
    double x,y;
};
int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ttlcode>());
    rclcpp::shutdown();
}