#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <std_msgs/msg/int32.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class TestTimeProcessor : public rclcpp::Node
{
    public:
        TestTimeProcessor()
        : Node("test_time_processor")
        {
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
            
            odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "wheel/odometry", qos, std::bind(&TestTimeProcessor::odometry_callback, this, _1)
            );

            imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "imu/data", qos, std::bind(&TestTimeProcessor::imu_callback, this, _1)
            );

            timeref_publisher_ = this->create_publisher<sensor_msgs::msg::TimeReference>("time_reference",10);
            publish_timer_ = this->create_wall_timer(
                1s, std::bind(&TestTimeProcessor::timer_callback(), this)
            );

            RCLCPP_INFO(this->get_logger(), "Start listen to /wheel/odometry and /imu/data.\n");
        }
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr timeref_publisher_;

        void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(),
            "[Odometry] Receive Time:%ld | Current Time:%ld",
            msg->header.stamp.sec,
            std::chrono::system_clock::now());                
        }
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(),
            "[IMU] Receive Time:%ld | Current Time:%ld",
            msg->header.stamp.sec,
            std::chrono::system_clock::now());
        }
        void timer_callback()
        {
            auto time_ref_message = sensor_msgs::msg::TimeReference();
        
            time_ref_message.header.stamp = rclcpp::Time::Time();
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestTimeProcessor>());
    rclcpp::shutdown();
    return 0;
}
