// src/madgwick_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <mutex>
#include <memory>

using std::placeholders::_1;

class Madgwick {
public:
  Madgwick(double beta = 0.1) : beta_(beta) {
    q0_ = 1.0; q1_ = 0.0; q2_ = 0.0; q3_ = 0.0;
  }
  void setBeta(double b) { beta_ = b; }
  std::array<double,4> getQuaternion() const { return {q0_, q1_, q2_, q3_}; }

  void setQuaternion(double w, double x, double y, double z) {
    double norm = std::sqrt(w*w + x*x + y*y + z*z);
    if (norm > 0.0) {
        q0_ = w / norm;
        q1_ = x / norm;
        q2_ = y / norm;
        q3_ = z / norm;
    }
  }

  // update with magnetometer (mx,my,mz). If mx=my=mz=0 -> falls back to IMU-only update.
  void update(double gx, double gy, double gz,
              double ax, double ay, double az,
              double mx, double my, double mz,
              double dt)
  {
    if (dt <= 0.0) return;
    // Convert rate to radians/sec expected input is rad/s (keep as-is).
    if (mx == 0.0 && my == 0.0 && mz == 0.0) {
      updateIMU(gx,gy,gz,ax,ay,az,dt);
      return;
    }

    // Normalise accelerometer
    double recipNorm = invSqrt(ax*ax + ay*ay + az*az);
    if (!std::isfinite(recipNorm)) return;
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    // Normalise magnetometer
    recipNorm = invSqrt(mx*mx + my*my + mz*mz);
    if (!std::isfinite(recipNorm)) return;
    mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

    // shorthand
    double q0=q0_, q1=q1_, q2=q2_, q3=q3_;

    // Rate of change of quaternion from gyroscope
    double qDot1 = 0.5 * (-q1*gx - q2*gy - q3*gz);
    double qDot2 = 0.5 * ( q0*gx + q2*gz - q3*gy);
    double qDot3 = 0.5 * ( q0*gy - q1*gz + q3*gx);
    double qDot4 = 0.5 * ( q0*gz + q1*gy - q2*gx);

    // Auxiliary variables to avoid repeated arithmetic
    double _2q0mx = 2.0 * q0 * mx;
    double _2q0my = 2.0 * q0 * my;
    double _2q0mz = 2.0 * q0 * mz;
    double _2q1mx = 2.0 * q1 * mx;
    double _2q0   = 2.0 * q0;
    double _2q1   = 2.0 * q1;
    double _2q2   = 2.0 * q2;
    double _2q3   = 2.0 * q3;
    double _2q0q2 = 2.0 * q0 * q2;
    double _2q2q3 = 2.0 * q2 * q3;
    double q0q0 = q0*q0;
    double q0q1 = q0*q1;
    double q0q2 = q0*q2;
    double q0q3 = q0*q3;
    double q1q1 = q1*q1;
    double q1q2 = q1*q2;
    double q1q3 = q1*q3;
    double q2q2 = q2*q2;
    double q2q3 = q2*q3;
    double q3q3 = q3*q3;

    // Reference direction of Earth's magnetic field
    double hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    double hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    double _2bx = std::sqrt(hx * hx + hy * hy);
    double _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    double _4bx = 2.0 * _2bx;
    double _4bz = 2.0 * _2bz;

    // Gradient descent algorithm corrective step
    double s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay)
      - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
      + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
      + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);

    double s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay)
      - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az)
      + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
      + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
      + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);

    double s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay)
      - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az)
      + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
      + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
      + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);

    double s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay)
      + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
      + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
      + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);

    // normalise step magnitude
    recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta_ * s0;
    qDot2 -= beta_ * s1;
    qDot3 -= beta_ * s2;
    qDot4 -= beta_ * s3;

    // Integrate to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    q0_ = q0; q1_ = q1; q2_ = q2; q3_ = q3;
  }

  // IMU-only update (no magnetometer)
  void updateIMU(double gx, double gy, double gz,
                 double ax, double ay, double az,
                 double dt)
  {
    if (dt <= 0.0) return;

    double q0=q0_, q1=q1_, q2=q2_, q3=q3_;

    double qDot1 = 0.5 * (-q1*gx - q2*gy - q3*gz);
    double qDot2 = 0.5 * ( q0*gx + q2*gz - q3*gy);
    double qDot3 = 0.5 * ( q0*gy - q1*gz + q3*gx);
    double qDot4 = 0.5 * ( q0*gz + q1*gy - q2*gx);

    if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
      double recipNorm = invSqrt(ax*ax + ay*ay + az*az);
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

      double _2q0 = 2.0 * q0;
      double _2q1 = 2.0 * q1;
      double _2q2 = 2.0 * q2;
      double _2q3 = 2.0 * q3;
      double _4q0 = 4.0 * q0;
      double _4q1 = 4.0 * q1;
      double _4q2 = 4.0 * q2;
      double _8q1 = 8.0 * q1;
      double _8q2 = 8.0 * q2;
      double q0q0 = q0*q0;
      double q1q1 = q1*q1;
      double q2q2 = q2*q2;
      double q3q3 = q3*q3;

      double s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      double s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      double s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      double s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

      recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
      s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

      qDot1 -= beta_ * s0;
      qDot2 -= beta_ * s1;
      qDot3 -= beta_ * s2;
      qDot4 -= beta_ * s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    double recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    q0_ = q0; q1_ = q1; q2_ = q2; q3_ = q3;
  }

private:
  double q0_, q1_, q2_, q3_;
  double beta_;

  static double invSqrt(double x) {
    if (x <= 0.0) return 0.0;
    return 1.0 / std::sqrt(x);
  }
};

// ----------------- ROS2 node -----------------
class MadgwickNode : public rclcpp::Node {
public:
  MadgwickNode()
  : Node("madgwick_node"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)),
    last_stamp_set_(false)
  {
    // parameters with Dashing-compatible declare_parameter usage
    this->declare_parameter<std::string>("imu_topic", "/camera/imu");
    this->declare_parameter<std::string>("mag_topic", "/mag");
    this->declare_parameter<std::string>("output_frame", "camera_imu_optical_frame");
    this->declare_parameter<bool>("use_magnetometer", false);
    this->declare_parameter<bool>("publish_tf", false);
    this->declare_parameter<double>("beta", 0.2);
    this->declare_parameter<bool>("remove_gravity", true);
    this->declare_parameter<double>("bias_x", -0.001140);
    this->declare_parameter<double>("bias_y", 0.14651);
    this->declare_parameter<double>("bias_z", 0.0);


    imu_topic_ = this->get_parameter("imu_topic").as_string();
    mag_topic_ = this->get_parameter("mag_topic").as_string();
    frame_id_ = this->get_parameter("output_frame").as_string();
    use_mag_ = this->get_parameter("use_magnetometer").as_bool();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    double beta = this->get_parameter("beta").as_double();
    remove_gravity_ = this->get_parameter("remove_gravity").as_bool();
    bias_x_ = this->get_parameter("bias_x").as_double();
    bias_y_ = this->get_parameter("bias_y").as_double();
    bias_z_ = this->get_parameter("bias_z").as_double();

    filter_.setBeta(beta);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,  rclcpp::SensorDataQoS(),
      std::bind(&MadgwickNode::imuCallback, this, _1));

    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      mag_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MadgwickNode::magCallback, this, _1));

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_madgwick", 10);
    RCLCPP_INFO(this->get_logger(), "Madgwick node started. imu_topic: %s use_mag: %s publish_tf: %s",
                imu_topic_.c_str(), use_mag_ ? "true":"false", publish_tf_ ? "true":"false");
  }

private:
  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mag_mutex_);
    last_mag_x_ = msg->magnetic_field.x;
    last_mag_y_ = msg->magnetic_field.y;
    last_mag_z_ = msg->magnetic_field.z;
    last_mag_stamp_ = msg->header.stamp;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    rclcpp::Time stamp = msg->header.stamp;
    double dt = 0.0;
    if (last_stamp_set_) {
      dt = (stamp - last_stamp_).seconds();
      if (dt <= 0.0 || dt > 1.0) {
        // bad dt; try using a small fallback
        dt = 1.0 / 200.0;
      }
    } else {
      last_stamp_set_ = true;
      dt = 1.0 / 200.0;
    }
    last_stamp_ = stamp;

    double gx = msg->angular_velocity.x;
    double gy = msg->angular_velocity.y;
    double gz = msg->angular_velocity.z;

    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    double mx=0.0, my=0.0, mz=0.0;
    if (use_mag_) {
      std::lock_guard<std::mutex> lk(mag_mutex_);
      mx = last_mag_x_;
      my = last_mag_y_;
      mz = last_mag_z_;
    }

    // Filter needs an initial guess at orientation
    // Assuming IMU is stationary at start we can guess from the acceleration vectors
    static bool initialized = false;
    if (!initialized) {
        // Normalize accel
        double norm = std::sqrt(ax*ax + ay*ay + az*az);
        if (norm > 1e-6) {
            ax /= norm; ay /= norm; az /= norm;

            // Compute roll & pitch guess from accel
            double roll  = std::atan2(ay, az);
            double pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));

            // Use yaw=0 unless magnetometer available
            double yaw = 0.0;
            if (use_mag_) {
                // Simple yaw from mag + accel
                yaw = std::atan2(my * std::cos(roll) - mz * std::sin(roll),
                                 mx * std::cos(pitch) + my * std::sin(pitch) * std::sin(roll) + mz * std::sin(pitch) * std::cos(roll));
            }

            // Convert roll-pitch-yaw -> quaternion
            tf2::Quaternion q_init;
            q_init.setRPY(roll, pitch, yaw);
            q_init.normalize();

            // Set initial filter quaternion
            filter_.setQuaternion(q_init.w(), q_init.x(), q_init.y(), q_init.z());

            initialized = true;
        }
    }

    // remove gravity vector from imu linear accel
    if (remove_gravity_) {
      auto q = filter_.getQuaternion();
      double qw = q[0], qx = q[1], qy = q[2], qz = q[3];

      // Rotate gravity vector (0,0,9.81) into sensor frame
      double up_x = 2.0*(qx*qz - qw*qy);
      double up_y = 2.0*(qw*qx + qy*qz);
      double up_z = (qw*qw - qx*qx - qy*qy + qz*qz);

      double g_x = up_x * 9.8067;
      double g_y = up_y * 9.8067;
      double g_z = up_z * 9.8067;

      ax -= g_x;
      ay -= g_y;
      az -= g_z;
    }

    // Correct for IMU bias
    ax -= bias_x_;
    ay -= bias_y_;
    az -= bias_z_;

    // update filter
    filter_.update(gx,gy,gz, ax,ay,az, mx,my,mz, dt);

    auto q = filter_.getQuaternion();

    // publish sensor_msgs/Imu with orientation and copied linear accel and angular vel
    sensor_msgs::msg::Imu out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = frame_id_;
    out.orientation.w = q[0];
    out.orientation.x = q[1];
    out.orientation.y = q[2];
    out.orientation.z = q[3];

    out.angular_velocity = msg->angular_velocity;
    if (remove_gravity_) {
      out.linear_acceleration.x = ax;
      out.linear_acceleration.y = ay;
      out.linear_acceleration.z = az;
    } else {
      out.linear_acceleration = msg->linear_acceleration;
    }

    // Copy angular/linear covariances
    out.angular_velocity_covariance = msg->angular_velocity_covariance;
    out.linear_acceleration_covariance = msg->linear_acceleration_covariance;
   
    // Orientation covariances experimentally determined
    out.orientation_covariance[0] = 2.62198157e-04;
    out.orientation_covariance[1] = -7.88885194e-06;
    out.orientation_covariance[2] = 0.0;
    out.orientation_covariance[3] = -7.88885194e-06;
    out.orientation_covariance[4] = 2.51064521e-07;
    out.orientation_covariance[5] = 0.0;
    out.orientation_covariance[6] = 0.0;
    out.orientation_covariance[7] = 0.0;
    out.orientation_covariance[8] = 0.0;

    pub_->publish(out);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tmsg;
      tmsg.header.stamp = msg->header.stamp;
      tmsg.header.frame_id = "base_link";
      tmsg.child_frame_id = frame_id_;
      tmsg.transform.translation.x = 0.0;
      tmsg.transform.translation.y = 0.0;
      tmsg.transform.translation.z = 0.0;
      tmsg.transform.rotation = out.orientation;
      tf_broadcaster_->sendTransform(tmsg);
    }
  }

  // members
  Madgwick filter_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string imu_topic_;
  std::string mag_topic_;
  std::string frame_id_;
  bool use_mag_;
  bool publish_tf_;
  bool remove_gravity_;

  // IMU bias on each axis (experimentally determined by user)
  double bias_x_, bias_y_, bias_z_;

  std::mutex mag_mutex_;
  double last_mag_x_{0.0}, last_mag_y_{0.0}, last_mag_z_{0.0};
  rclcpp::Time last_mag_stamp_;
  rclcpp::Time last_stamp_;
  bool last_stamp_set_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MadgwickNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

