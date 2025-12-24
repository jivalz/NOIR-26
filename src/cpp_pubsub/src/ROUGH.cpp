#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/int32.hpp"

#include <iostream>
#include <cmath>

using namespace std;

class gtg : public rclcpp::Node
{
    public:
    gtg(): Node("rado_iter1")
    {
        sub_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>("/fix",10,std::bind(&gtg::gps_callback,this,std::placeholders::_1));
        sub_mag = this->create_subscription<std_msgs::msg::Int32>("/imu_data",10,std::bind(&gtg::mag_callback,this,std::placeholders::_1));
        timer = this->create_wall_timer(std::chrono::milliseconds(200),std::bind(&gtg::gotogoal,this));
        pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1",10);
        r_lat = 21.167085;
        r_long = 72.785789;
    }
    private:

    double kp_lin = 1.0;
    double kp_ang = 4.0;

    double r_lat, r_long;
    double curr_lat = 0.0, curr_long = 0.0;
    double goal_lat = 0.0, goal_long = 0.0;
    double heading = 0.0;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_mag;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;

    void req_goal()
    {
        cout<<"Enter goal latitude: ";
        cin>>goal_lat;
        cout<<"Enter goal longitude: ";
        cin>>goal_long;

        cout<<"Goal Set to: "<<goal_lat<<", "<<goal_long<<endl;
    }

    void latlon_to_xy(double lat,double lon,double &x, double &y)
    {
        double R = 6378137.0;
        double dLat = (lat-r_lat) * M_PI / 180;
        double dLong = (lon - r_long) * M_PI / 180;
        x = R * dLong * cos(r_lat * M_PI / 180);
        y = R * dLat;
    }

    void mag_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        heading = msg->data;
    }
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        curr_lat = msg->latitude;
        curr_long = msg->longitude;
    }
    void move(double cx, double cy, double theta)
    {
        auto twmsg = geometry_msgs::msg::Twist();
        double gx,gy;
        latlon_to_xy(goal_lat,goal_long,gx,gy);
        double dis =  sqrt(pow(gx-cx,2)+pow(gy-cy,2));
        double angle = atan2(gy-cy,gx-cx);

        if(dis > 0.1)
        {
            twmsg.linear.x = kp_lin * dis;
            twmsg.angular.z = kp_ang*(angle-theta);
        }else{
            cout<<"<<<<Goal reached, Enter new goal>>>>"<<endl;
            req_goal();
        }
        pub->publish(twmsg);
    }
    void gotogoal()
    {
        double cx,cy;
        latlon_to_xy(curr_lat,curr_long,cx,cy);
        move(cx,cy,heading);
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gtg>());
    rclcpp::shutdown();
    return 0;
}