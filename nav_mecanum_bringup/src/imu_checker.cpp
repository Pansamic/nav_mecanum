/**
 * @file imu_check.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
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
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class IMUChecker : public rclcpp::Node
{
public:
    IMUChecker():
    Node("IMUChecker")
    {
        /******************************************************************/
        /****                    Register Handles                      ****/
        /******************************************************************/
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data", 10, std::bind(&IMUChecker::imu_subscription_callback, this, _1));
    }
private:
    /******************************************************************/
    /****                        Handles                           ****/
    /******************************************************************/
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    sensor_msgs::msg::Imu::SharedPtr last_msg_;
    double angle_x_, angle_y_, angle_z_, velocity_x_, velocity_y_, velocity_z_;
    const double kRadToDegree = 57.29577951;
    const double kDegreeToRad = 0.017453293;

    void imu_subscription_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "[IMU]linear_accel:{x=%lf, y=%lf, z=%lf}, angular_vel:{x=%lf, y=%lf, z=%lf}",
        //     msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
        //     msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        if (last_msg_==nullptr)
        {
            last_msg_ = msg;
            return;
        }
        geometry_msgs::msg::TransformStamped tf_stamped;
        rclcpp::Time last_time = last_msg_->header.stamp;
        rclcpp::Time current_time = msg->header.stamp;
        double interval = (current_time - last_time).seconds();
        double angle_x = atan2(msg->linear_acceleration.y, msg->linear_acceleration.z);
        double angle_y = atan2(msg->linear_acceleration.x, msg->linear_acceleration.z);
        
        angle_x_ = KalmanFilterX(angle_x, msg->angular_velocity.x, interval);
        angle_y_ = KalmanFilterY(angle_y, msg->angular_velocity.y, interval);
        angle_z_ += msg->angular_velocity.z * interval;

        tf2::Quaternion q;
        q.setRPY(angle_x_, angle_y_, angle_z_);
        tf_stamped.header.frame_id = "world";
        tf_stamped.header.stamp = this->get_clock()->now();
        tf_stamped.child_frame_id = "base_link";
        tf_stamped.transform.rotation.x = q.x();
        tf_stamped.transform.rotation.y = q.y();
        tf_stamped.transform.rotation.z = q.z();
        tf_stamped.transform.rotation.w = q.w();
        tf_stamped.transform.translation.x = 0;
        tf_stamped.transform.translation.y = 0;
        tf_stamped.transform.translation.z = 0;
        tf_broadcaster_->sendTransform(tf_stamped);
        last_msg_ = msg;
        RCLCPP_INFO(this->get_logger(), "angle: {roll:%lf, pitch:%lf, yaw=%lf}", angle_x_, angle_y_, angle_z_);
    }

    /**
     * @brief IMU xAxis Kalman filter.
     * 
     * @param Angle unit:degree
     * @param AngVelocity unit:rad/s
     * @param interval unit:second
     * @return double 
     */
    double KalmanFilterX(double Angle, double AngVelocity, double interval)		
    {
        //static double angle_dot;
        static double angle;
        double Q_angle=0.001; 		// 过程噪声的协方差
        double Q_gyro=0.003;		// 0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
        double R_angle=0.5;			// 测量噪声的协方差 既测量偏差
        char  C_0 = 1;
        static double Q_bias, Angle_err;
        static double PCt_0, PCt_1, E;
        static double K_0, K_1, t_0, t_1;
        static double Pdot[4] ={0,0,0,0};
        static double PP[2][2] = { { 1, 0 },{ 0, 1 } };
        angle+=(AngVelocity - Q_bias) * interval; //先验估计
        Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
        Pdot[1]=-PP[1][1];
        Pdot[2]=-PP[1][1];
        Pdot[3]=Q_gyro;
        PP[0][0] += Pdot[0] * interval;   // Pk-先验估计误差协方差微分的积分
        PP[0][1] += Pdot[1] * interval;   // 先验估计误差协方差
        PP[1][0] += Pdot[2] * interval;
        PP[1][1] += Pdot[3] * interval;
        Angle_err = Angle - angle;	//zk-先验估计
        
        PCt_0 = C_0 * PP[0][0];
        PCt_1 = C_0 * PP[1][0];
        
        E = R_angle + C_0 * PCt_0;
        
        K_0 = PCt_0 / E;
        K_1 = PCt_1 / E;
        
        t_0 = PCt_0;
        t_1 = C_0 * PP[0][1];

        PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
        PP[0][1] -= K_0 * t_1;
        PP[1][0] -= K_1 * t_0;
        PP[1][1] -= K_1 * t_1;
            
        angle	+= K_0 * Angle_err;	   //后验估计
        Q_bias	+= K_1 * Angle_err;	 //后验估计
        //angle_dot   = Gyro - Q_bias;	//输出值(后验估计)的微分=角速度
        return angle;
    }
    double KalmanFilterY(double Angle, double AngVelocity, double interval)		
    {
        //static double angle_dot;
        static double angle;
        double Q_angle=0.001; 		// 过程噪声的协方差
        double Q_gyro=0.003;			// 0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
        double R_angle=0.5;			// 测量噪声的协方差 既测量偏差
        char  C_0 = 1;
        static double Q_bias, Angle_err;
        static double PCt_0, PCt_1, E;
        static double K_0, K_1, t_0, t_1;
        static double Pdot[4] ={0,0,0,0};
        static double PP[2][2] = { { 1, 0 },{ 0, 1 } };
        angle+=(AngVelocity - Q_bias) * interval; //先验估计
        Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
        Pdot[1]=-PP[1][1];
        Pdot[2]=-PP[1][1];
        Pdot[3]=Q_gyro;
        PP[0][0] += Pdot[0] * interval;   // Pk-先验估计误差协方差微分的积分
        PP[0][1] += Pdot[1] * interval;   // 先验估计误差协方差
        PP[1][0] += Pdot[2] * interval;
        PP[1][1] += Pdot[3] * interval;
        Angle_err = Angle - angle;	//zk-先验估计
        
        PCt_0 = C_0 * PP[0][0];
        PCt_1 = C_0 * PP[1][0];
        
        E = R_angle + C_0 * PCt_0;
        
        K_0 = PCt_0 / E;
        K_1 = PCt_1 / E;
        
        t_0 = PCt_0;
        t_1 = C_0 * PP[0][1];

        PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
        PP[0][1] -= K_0 * t_1;
        PP[1][0] -= K_1 * t_0;
        PP[1][1] -= K_1 * t_1;

        angle	+= K_0 * Angle_err;	   //后验估计
        Q_bias	+= K_1 * Angle_err;	 //后验估计
        //angle_dot   = Gyro - Q_bias;	//输出值(后验估计)的微分=角速度
        return angle;
    }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUChecker>());
    rclcpp::shutdown();
    return 0;
}