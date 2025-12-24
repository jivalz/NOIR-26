#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geodesy/utm.h>

/*ros2 run nmea_navsat_driver nmea_serial_driver   --ros-args -p port:=/dev/ttyUSB1 -p baud:=9600 -p use_gnss:=true //GPS
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 //magn or imu
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB2 // drive 
*/


class gps : public rclcpp::Node
{
public:
    gps() : Node("gps_data_publisher")
    {
        sub1 = this->create_subscription<sensor_msgs::msg::NavSatFix>("/fix", 10, std::bind(&gps::latlon, this, std::placeholders::_1));

        sub2 = this->create_subscription<std_msgs::msg::Int32>("/imu_data", 10, std::bind(&gps::imu_callback, this, std::placeholders::_1));

        pub_ = create_publisher<geometry_msgs::msg::Twist>("/gps/enu", 10);
        
        pub  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);
    }

private:
    bool origin_set_=false;
    double k=1,kp=3.7;
    double origin_e_, origin_n_, origin_u_;
    geographic_msgs::msg::GeoPoint origin_geo;
    std_msgs::msg::Int32 last_imu;   // store latest IMU

    void latlon(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
    //   if (msg->status.status < 0) {
    //   RCLCPP_WARN(get_logger(), "Invalid GPS fix");
    //   return;
    // }

    /* NavSatFix → GeoPoint */
    geographic_msgs::msg::GeoPoint geo;
    geo.latitude  = msg->latitude;
    geo.longitude = msg->longitude;
    geo.altitude  = msg->altitude;

    geodesy::UTMPoint utm;
    geodesy::fromMsg(geo, utm);


    
    while(origin_geo.latitude==0){
        std::cout << "Enter latitude: ";
        std::cin >> origin_geo.latitude;
    }
    while(origin_geo.longitude==0){
        std::cout << "Enter longitude: ";
        std::cin >> origin_geo.longitude;
    }



    if (!origin_set_) {
        origin_geo.altitude  = 0.0;
        geodesy::UTMPoint origin_utm;
        geodesy::fromMsg(origin_geo, origin_utm);
        origin_e_ = origin_utm.easting;
        origin_n_ = origin_utm.northing;
        origin_u_ = origin_utm.altitude;
        origin_set_ = true;
    }

    geometry_msgs::msg::Twist twist;

    double dx = utm.easting  - origin_e_;
    double dy = utm.northing - origin_n_;
    //double dz = utm.altitude - origin_u_;

    twist.linear.x = utm.easting  - origin_e_;   // East
    twist.linear.y = utm.northing - origin_n_;   // North
    twist.linear.z = utm.altitude - origin_u_;   // Up

    double d = std::sqrt(dx*dx + dy*dy);

    twist.angular.x = last_imu.data;
    twist.angular.y = d;
    twist.angular.z = 0.0;

    geometry_msgs::msg::Twist cmd_vel;
    double phi=atan2((dx) , (dy));
    phi=abs(phi)>3.14 ? phi - 2*M_PI*phi/ abs(phi):phi;
    float compass=M_PI*(last_imu.data)/180;
    float angle=phi-compass;
    double v=(k*d);
    double ang=(kp*angle);
    if(d<0.001)
    {
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      std::cout<<"Reached Goal \n";
      origin_geo.latitude=0;
      origin_geo.longitude=0;
      origin_set_ = false;
    }
    else
    {
      cmd_vel.linear.x=v;
      cmd_vel.angular.z=ang;
    }



    pub_->publish(twist);
    pub->publish(cmd_vel);
    }

    void imu_callback(const std_msgs::msg::Int32::SharedPtr imumsg)
    {
        last_imu = *imumsg;    // store newest IMU
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub1;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gps>());
    rclcpp::shutdown();
    return 0;
}
