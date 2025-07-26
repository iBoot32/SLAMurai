#include "RSNode.hpp"

RSNode::RSNode() : Node("rs_node")
{
    // Create best effort QoS profile using explicit policy
    auto qos = rclcpp::QoS(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", qos,
        std::bind(&RSNode::imuCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to Madgwick Filter's /imu/data");
}

void RSNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    computeEulerFromQuaternion(msg, _current_pose_);
}

RSPose RSNode::computeEulerFromQuaternion(const sensor_msgs::msg::Imu::SharedPtr msg, RSPose &pose)
{
    // Raw IMU quaternion from RealSense (optical frame)
    tf2::Quaternion q_optical(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    // Correct the orientation to match the robot's frame (correct -90, 0, -90 when facing forward)
    tf2::Quaternion rot_1, rot_2;
    rot_1.setRPY(0, 0, M_PI_2);
    rot_2.setRPY(M_PI_2, 0, 0);

    q_optical = rot_2 * rot_1 * q_optical;

    // Read in YPR -> RYP due to RealSense axis convention
    tf2::Matrix3x3 m(q_optical);
    m.getEulerYPR(pose.roll, pose.yaw, pose.pitch);

    // Read as radians, convert to degrees
    pose.yaw *= 180.0 / M_PI;
    pose.pitch *= 180.0 / M_PI;
    pose.roll *= 180.0 / M_PI;

    return pose;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RSNode>());
    rclcpp::shutdown();
    return 0;
}