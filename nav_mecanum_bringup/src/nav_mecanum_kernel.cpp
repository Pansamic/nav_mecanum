/**
 * @file nav_mecanum_kernel.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_msgs/msg/joint_jog.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class NavMecanumKernel : public rclcpp::Node
{
    public:
        NavMecanumKernel()
        : Node("nav_mecanum_kernel")
        {
            /************************************/
            /*            publisher             */
            /************************************/
            laser_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            timeref_publisher_ = this->create_publisher<sensor_msgs::msg::TimeReference>("time_reference",10);
            joint_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>("cmd_mecanum",10);
            /************************************/
            /*            subsciber             */
            /************************************/
            // imu_subsciber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            //     "imu/data", 10, std::bind(&NavMecanumKernel::imu_subscription_callback, this, _1)
            // );

            // odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            //     "odom/unfiltered", 10, std::bind(&NavMecanumKernel::odom_subscription_callback, this, _1)
            // );
            twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&NavMecanumKernel::twist_subscription_callback, this, _1));
            /************************************/
            /*              timer               */
            /************************************/
            laser_joint_state_publish_timer_ = this->create_wall_timer(
                100ms, std::bind(&NavMecanumKernel::laser_joint_state_timer_callback, this));
            timeref_publish_timer_ = this->create_wall_timer(
                2s, std::bind(&NavMecanumKernel::timeref_timer_callback, this));


        }
    private:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr laser_joint_state_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr timeref_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_publisher_;

        // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subsciber_;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;

        rclcpp::TimerBase::SharedPtr laser_joint_state_publish_timer_;
        rclcpp::TimerBase::SharedPtr timeref_publish_timer_;
        rclcpp::TimerBase::SharedPtr odom_publish_timer_;

        sensor_msgs::msg::JointState::SharedPtr last_joint_states;

        double heading_;
        double x_pos_;
        double y_pos_;
        const double car_x_wheel_dis = 0.223;
        const double car_y_wheel_dis = 0.193;
        const double car_wheel_radius = 0.0375;


        void timeref_timer_callback()
        {
            auto time_ref_message = sensor_msgs::msg::TimeReference();
        
            // time_ref_message.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            time_ref_message.header.stamp = this->get_clock()->now();
            time_ref_message.time_ref = this->get_clock()->now();
            timeref_publisher_->publish(time_ref_message);
        }
        void laser_joint_state_timer_callback()
        {
            auto joint_state = sensor_msgs::msg::JointState();

            joint_state.header.stamp = this->get_clock()->now();
            joint_state.header.frame_id = "";
            joint_state.name.push_back("laser");
            joint_state.velocity.push_back(0);
            joint_state.effort.push_back(0);
            joint_state.position.push_back(0);

            laser_joint_state_publisher_->publish(joint_state);
        }
        void imu_subscription_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            static unsigned char cb_count = 0;
            cb_count++;
            if(cb_count >= 50)
            {
                RCLCPP_INFO(this->get_logger(), "[IMU]linear_accel:{x=%lf, y=%lf, z=%lf}, angular_vel:{x=%lf, y=%lf, z=%lf}",
                    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
                    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
                cb_count = 0;
            }
        }
        void odom_subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            static unsigned char cb_count = 0;
            cb_count++;
            if(cb_count >= 20)
            {
                RCLCPP_INFO(this->get_logger(), "[Odom]position:{x=%lf, y=%lf}, linear:{x=%lf, y=%lf}, angular:{z=%lf}",
                    msg->pose.pose.position.x, msg->pose.pose.position.y,
                    msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z);
                cb_count = 0;
            }
        }
        void twist_subscription_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            auto joint_jog_msg = control_msgs::msg::JointJog();
            double vel_x = msg->linear.x;   // unit: m/s
            double vel_y = msg->linear.y;   // unit: m/s
            double vel_ang = msg->angular.z; // unit: rad/s
            double left_front_wheel_vel = (vel_x - vel_y - vel_ang * (car_x_wheel_dis + car_y_wheel_dis))/car_wheel_radius;
            double left_rear_wheel_vel = (vel_x + vel_y - vel_ang * (car_x_wheel_dis + car_y_wheel_dis))/car_wheel_radius;
            double right_front_wheel_vel = (vel_x + vel_y + vel_ang * (car_x_wheel_dis + car_y_wheel_dis))/car_wheel_radius;
            double right_rear_wheel_vel = (vel_x - vel_y + vel_ang * (car_x_wheel_dis + car_y_wheel_dis))/car_wheel_radius;

            joint_jog_msg.header.stamp = this->get_clock()->now();
            joint_jog_msg.header.frame_id = "";
            joint_jog_msg.joint_names.push_back("left_front");
            joint_jog_msg.joint_names.push_back("left_rear");
            joint_jog_msg.joint_names.push_back("right_front");
            joint_jog_msg.joint_names.push_back("right_rear");
            joint_jog_msg.displacements.push_back(0);
            joint_jog_msg.displacements.push_back(0);
            joint_jog_msg.displacements.push_back(0);
            joint_jog_msg.displacements.push_back(0);
            joint_jog_msg.velocities.push_back(left_front_wheel_vel);
            joint_jog_msg.velocities.push_back(left_rear_wheel_vel);
            joint_jog_msg.velocities.push_back(right_front_wheel_vel);
            joint_jog_msg.velocities.push_back(right_rear_wheel_vel);

            joint_jog_publisher_->publish(joint_jog_msg);
        }
        void joint_states_subscription_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            auto odom_msg = nav_msgs::msg::Odometry();

            if(last_joint_states==nullptr)
            {
                last_joint_states = msg;
                return;
            }
            rclcpp::Time last_time = last_joint_states->header.stamp;
            rclcpp::Time current_time = msg->header.stamp;
            double time_interval = (last_time - current_time).seconds();

            double left_front_wheel_vel = car_wheel_radius * msg->velocity[0]; // unit:m/s
            double left_rear_wheel_vel = car_wheel_radius * msg->velocity[1];
            double right_front_wheel_vel = car_wheel_radius * msg->velocity[2];
            double right_rear_wheel_vel = car_wheel_radius * msg->velocity[3];

            double car_x_vel = (left_front_wheel_vel + right_rear_wheel_vel + left_rear_wheel_vel + right_front_wheel_vel)/4.0;
            double car_y_vel = (left_front_wheel_vel + right_rear_wheel_vel - left_rear_wheel_vel - right_front_wheel_vel)/4.0;
            double car_ang_vel = (right_front_wheel_vel + right_rear_wheel_vel - left_front_wheel_vel - left_rear_wheel_vel)/(4.0*(car_x_wheel_dis+car_y_wheel_dis));

            double delta_heading = car_ang_vel * time_interval; //radians
            double cos_h = cos(heading_);
            double sin_h = sin(heading_);
            double delta_x = (car_x_vel * cos_h - car_y_vel * sin_h) * time_interval; //m
            double delta_y = (car_x_vel * sin_h - car_y_vel * cos_h) * time_interval; //m

            //calculate current position of the robot
            x_pos_ += delta_x;
            y_pos_ += delta_y;
            heading_ += delta_heading;

            //calculate robot's heading in quaternion angle
            //ROS has a function to calculate yaw in quaternion angle
            double q[4];
            euler_to_quat(0, 0, heading_, q);

            //robot's position in x,y, and z
            odom_msg.pose.pose.position.x = x_pos_;
            odom_msg.pose.pose.position.y = y_pos_;
            odom_msg.pose.pose.position.z = 0.0;

            //robot's heading in quaternion
            odom_msg.pose.pose.orientation.x = q[1];
            odom_msg.pose.pose.orientation.y = q[2];
            odom_msg.pose.pose.orientation.z = q[3];
            odom_msg.pose.pose.orientation.w = q[0];

            odom_msg.pose.covariance[0] = 0.001;
            odom_msg.pose.covariance[7] = 0.001;
            odom_msg.pose.covariance[35] = 0.001;

            //linear speed from encoders
            odom_msg.twist.twist.linear.x = car_x_vel;
            odom_msg.twist.twist.linear.y = car_y_vel;
            odom_msg.twist.twist.linear.z = 0.0;

            //angular speed from encoders
            odom_msg.twist.twist.angular.x = 0.0;
            odom_msg.twist.twist.angular.y = 0.0;
            odom_msg.twist.twist.angular.z = car_ang_vel;

            odom_msg.twist.covariance[0] = 0.0001;
            odom_msg.twist.covariance[7] = 0.0001;
            odom_msg.twist.covariance[35] = 0.0001;

            odom_publisher_->publish(odom_msg);
        }

        void euler_to_quat(double roll, double pitch, double yaw, double* q)
        {
            double cy = cos(yaw * 0.5);
            double sy = sin(yaw * 0.5);
            double cp = cos(pitch * 0.5);
            double sp = sin(pitch * 0.5);
            double cr = cos(roll * 0.5);
            double sr = sin(roll * 0.5);

            q[0] = cy * cp * cr + sy * sp * sr;
            q[1] = cy * cp * sr - sy * sp * cr;
            q[2] = sy * cp * sr + cy * sp * cr;
            q[3] = sy * cp * cr - cy * sp * sr;
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavMecanumKernel>());
    rclcpp::shutdown();
    return 0;
}