#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

struct RSPose
{
    double roll;
    double pitch;
    double yaw;

    double velocity_x = 0.0;
    double velocity_y = 0.0;
    double velocity_z = 0.0;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

class RSNode : public rclcpp::Node
{
public:
    RSNode();

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    RSPose computeEulerFromQuaternion(const sensor_msgs::msg::Imu::SharedPtr msg, RSPose &pose);
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    RSPose _current_pose_;
};
