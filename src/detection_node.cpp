#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class AccelDecelTestNode : public rclcpp::Node {
public:
  AccelDecelTestNode() : Node("detection_node") {
    g_ = this->declare_parameter<double>("g", 9.81);
    mu_ = this->declare_parameter<double>("mu", 0.6);
    h_ = this->declare_parameter<double>("h", 0.15);
    Lf_ = this->declare_parameter<double>("Lf", 0.25);
    Lr_ = this->declare_parameter<double>("Lr", 0.45);
    L_ = Lf_ + Lr_;

    alpha_ = this->declare_parameter<double>("alpha", 0.9);
    beta_ = this->declare_parameter<double>("beta", 0.8);

    v_max1_ = this->declare_parameter<double>("v_max1", 3.6 / 3.6);
    v_max2_ = this->declare_parameter<double>("v_max2", 15.0 / 3.6);

    cruise1_time_ = this->declare_parameter<double>("cruise1_time", 0.8);
    cruise2_time_ = this->declare_parameter<double>("cruise2_time", 0.8);

    double raw_a_acc = calc_accel_limit();
    double raw_a_dec = calc_decel_limit();

    a_acc_ = raw_a_acc * alpha_;
    a_dec_ = raw_a_dec * beta_;

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/aiformula_control/twist_mux/cmd_vel", 10);

    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.best_effort();

    drive_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/aiformula_sensing/gyro_odometry_publisher/odom", qos_profile,
        std::bind(&AccelDecelTestNode::drive_odom_callback, this, std::placeholders::_1));

    dt_ = 0.1;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AccelDecelTestNode::control_loop, this));
  }

  ~AccelDecelTestNode() {
    emergency_stop();
  }

private:
  double calc_accel_limit() {
    double term1 = (mu_ * g_ * Lr_) / (L_ + mu_ * h_);
    double term2 = g_ * (Lr_ / h_);
    double term3 = mu_ * g_;
    return std::min({term1, term2, term3});
  }

  double calc_decel_limit() {
    double term1 = (mu_ * g_ * Lr_) / (L_ - mu_ * h_);
    double term2 = g_ * (Lf_ / h_);
    double term3 = mu_ * g_;
    return std::min({term1, term2, term3});
  }

  void drive_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    current_v_drive_ = std::hypot(vx, vy);
  }

  void emergency_stop() {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_pub_->publish(twist);
  }

  void control_loop() {
    rclcpp::Time now = this->now();
    if (!has_start_time_) {
      start_time_ = now;
      has_start_time_ = true;
    }

    double elapsed_total = (now - start_time_).seconds();

    if (state_ == "ACCEL1") {
      target_v_ += a_acc_ * dt_;
      if (target_v_ >= v_max1_) {
        target_v_ = v_max1_;
        state_ = "CRUISE1";
        phase_start_time_ = now;
      }
    } else if (state_ == "CRUISE1") {
      if ((now - phase_start_time_).seconds() >= cruise1_time_) state_ = "ACCEL2";
    } else if (state_ == "ACCEL2") {
      target_v_ += a_acc_ * dt_;
      if (target_v_ >= v_max2_) {
        target_v_ = v_max2_;
        state_ = "CRUISE2";
        phase_start_time_ = now;
      }
    } else if (state_ == "CRUISE2") {
      if ((now - phase_start_time_).seconds() >= cruise2_time_) state_ = "DECEL";
    } else if (state_ == "DECEL") {
      target_v_ -= a_dec_ * dt_;
      if (target_v_ <= 0.0) target_v_ = 0.0;
      if (target_v_ == 0.0 && current_v_drive_ <= 0.05) state_ = "DONE";
    } else if (state_ == "DONE") {
      target_v_ = 0.0;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "加減速テスト完了 (停止中)");
    }

    if (state_ != "DONE") {
      RCLCPP_INFO(this->get_logger(),
                  "[%s] 経過時間: %.1fs | 指令速度: %.2f m/s",
                  state_.c_str(), elapsed_total, target_v_);
    }

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = target_v_;
    twist.angular.z = 0.0;
    cmd_pub_->publish(twist);
  }

  double g_, mu_, h_, Lf_, Lr_, L_;
  double alpha_, beta_;
  double v_max1_, v_max2_;
  double cruise1_time_, cruise2_time_;
  double a_acc_, a_dec_;
  double dt_;

  double current_v_drive_{0.0};
  double target_v_{0.0};
  std::string state_{"ACCEL1"};
  rclcpp::Time start_time_;
  rclcpp::Time phase_start_time_;
  bool has_start_time_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drive_odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AccelDecelTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
