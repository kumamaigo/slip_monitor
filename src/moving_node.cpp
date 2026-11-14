#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class MovingNode : public rclcpp::Node {
public:
  MovingNode() : Node("moving_node") {
    // 1. 車両・路面パラメータ (detection_node.cpp の初期値を継承)
    g_ = this->declare_parameter<double>("g", 9.81);
    mu_ = this->declare_parameter<double>("mu", 0.6);
    h_ = this->declare_parameter<double>("h", 0.15);
    Lf_ = this->declare_parameter<double>("Lf", 0.25);
    Lr_ = this->declare_parameter<double>("Lr", 0.45);
    L_ = Lf_ + Lr_;

    // 2. 安全係数 (加速2段階・減速2段階で個別設定可能)
    alpha1_ = this->declare_parameter<double>("alpha1", 0.5);  // 0 -> v_max1 の加速用
    alpha2_ = this->declare_parameter<double>("alpha2", 0.9);  // v_max1 -> v_max2 の加速用
    beta1_ = this->declare_parameter<double>("beta1", 0.8);    // v_max2 -> v_dec_target の減速用
    beta2_ = this->declare_parameter<double>("beta2", 0.5);    // v_dec_target -> 0 の減速用

    // 3. 目標速度 [m/s] (デフォルト: 3km/h -> 15km/h -> 停止)
    v_max1_ = std::max(0.0, this->declare_parameter<double>("v_max1", 3.0 / 3.6));
    v_max2_ = std::max(0.0, this->declare_parameter<double>("v_max2", 15.0 / 3.6));
    v_dec_target_ = std::max(0.0, this->declare_parameter<double>("v_dec_target", 0.0));

    // 4. 各段階後の等速維持時間 [s]
    cruise1_time_ = std::max(0.0, this->declare_parameter<double>("cruise1_time", 1.0));
    cruise2_time_ = std::max(0.0, this->declare_parameter<double>("cruise2_time", 1.5));
    cruise3_time_ = std::max(0.0, this->declare_parameter<double>("cruise3_time", 1.0));

    // 5. 安全装置: 全体タイムアウト時間 [s]
    max_total_time_ = this->declare_parameter<double>("max_total_time", 30.0);

    // 警告チェック (安全係数 0 以下だと加減速しないため)
    if (alpha1_ <= 0.0 || alpha2_ <= 0.0 || beta1_ <= 0.0 || beta2_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
                  "安全係数 (alpha1, alpha2, beta1, beta2) に 0 以下の値が設定されています。"
                  "1段階加減速にしたい場合は安全係数ではなく v_max1 や v_dec_target を 0.0 にしてください。");
    }

    // 限界加減速度の算出と安全係数の適用
    double raw_a_acc = calc_accel_limit();
    double raw_a_dec = calc_decel_limit();

    a_acc1_ = raw_a_acc * alpha1_;
    a_acc2_ = raw_a_acc * alpha2_;
    a_dec1_ = raw_a_dec * beta1_;
    a_dec2_ = raw_a_dec * beta2_;

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/aiformula_control/twist_mux/cmd_vel", 10);

    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.best_effort();

    drive_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/aiformula_sensing/gyro_odometry_publisher/odom", qos_profile,
        std::bind(&MovingNode::drive_odom_callback, this, std::placeholders::_1));

    // 50Hz (20ms) 制御ループ
    dt_ = 0.02;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&MovingNode::control_loop, this));

    // 起動時に設定パラメータ一覧をログ出力
    RCLCPP_INFO(this->get_logger(), "=== MovingNode Initialized (50Hz) ===");
    RCLCPP_INFO(this->get_logger(),
                "Limits: raw_acc=%.2f m/s^2, raw_dec=%.2f m/s^2", raw_a_acc, raw_a_dec);
    RCLCPP_INFO(this->get_logger(),
                "Accel1: 0 -> %.2f km/h (a=%.2f m/s^2, alpha1=%.2f) -> Cruise1: %.2f s",
                v_max1_ * 3.6, a_acc1_, alpha1_, cruise1_time_);
    RCLCPP_INFO(this->get_logger(),
                "Accel2: %.2f -> %.2f km/h (a=%.2f m/s^2, alpha2=%.2f) -> Cruise2: %.2f s",
                v_max1_ * 3.6, v_max2_ * 3.6, a_acc2_, alpha2_, cruise2_time_);
    RCLCPP_INFO(this->get_logger(),
                "Decel1: %.2f -> %.2f km/h (d=%.2f m/s^2, beta1=%.2f) -> Cruise3: %.2f s",
                v_max2_ * 3.6, v_dec_target_ * 3.6, a_dec1_, beta1_, cruise3_time_);
    RCLCPP_INFO(this->get_logger(),
                "Decel2: %.2f -> 0.00 km/h (d=%.2f m/s^2, beta2=%.2f)",
                v_dec_target_ * 3.6, a_dec2_, beta2_);
  }

  ~MovingNode() {
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

    // フェイルセーフ: 全体制限時間を超えたら強制停止
    if (state_ != "DONE" && elapsed_total >= max_total_time_) {
      RCLCPP_WARN(this->get_logger(),
                  "最大実行時間 (%.1f s) に到達したため強制停止します。", max_total_time_);
      target_v_ = 0.0;
      state_ = "DONE";
    }

    if (state_ == "ACCEL1") {
      if (v_max1_ <= 0.0) {
        // 1段階加速モードの場合は即座にCRUISE1へ
        target_v_ = 0.0;
        state_ = "CRUISE1";
        phase_start_time_ = now;
      } else {
        target_v_ += a_acc1_ * dt_;
        if (target_v_ >= v_max1_) {
          target_v_ = v_max1_;
          state_ = "CRUISE1";
          phase_start_time_ = now;
        }
      }
    } else if (state_ == "CRUISE1") {
      if ((now - phase_start_time_).seconds() >= cruise1_time_) {
        state_ = "ACCEL2";
      }
    } else if (state_ == "ACCEL2") {
      if (v_max2_ <= target_v_) {
        target_v_ = v_max2_;
        state_ = "CRUISE2";
        phase_start_time_ = now;
      } else {
        target_v_ += a_acc2_ * dt_;
        if (target_v_ >= v_max2_) {
          target_v_ = v_max2_;
          state_ = "CRUISE2";
          phase_start_time_ = now;
        }
      }
    } else if (state_ == "CRUISE2") {
      if ((now - phase_start_time_).seconds() >= cruise2_time_) {
        state_ = "DECEL1";
      }
    } else if (state_ == "DECEL1") {
      if (target_v_ <= v_dec_target_) {
        target_v_ = v_dec_target_;
        if (v_dec_target_ <= 0.0) {
          state_ = "STOPPING";
          phase_start_time_ = now;
        } else {
          state_ = "CRUISE3";
          phase_start_time_ = now;
        }
      } else {
        target_v_ -= a_dec1_ * dt_;
        if (target_v_ <= v_dec_target_) {
          target_v_ = v_dec_target_;
          if (v_dec_target_ <= 0.0) {
            state_ = "STOPPING";
            phase_start_time_ = now;
          } else {
            state_ = "CRUISE3";
            phase_start_time_ = now;
          }
        }
      }
    } else if (state_ == "CRUISE3") {
      if ((now - phase_start_time_).seconds() >= cruise3_time_) {
        state_ = "DECEL2";
      }
    } else if (state_ == "DECEL2") {
      target_v_ -= a_dec2_ * dt_;
      if (target_v_ <= 0.0) {
        target_v_ = 0.0;
        state_ = "STOPPING";
        phase_start_time_ = now;
      }
    } else if (state_ == "STOPPING") {
      target_v_ = 0.0;
      // 実測速度が十分落ちるか、指令ゼロから2.0秒経過したら完了とする
      if (current_v_drive_ <= 0.05 || (now - phase_start_time_).seconds() >= 2.0) {
        state_ = "DONE";
      }
    } else if (state_ == "DONE") {
      target_v_ = 0.0;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "加減速テスト完了 (停止中)");
    }

    if (state_ != "DONE") {
      // 50Hzループでログが流れすぎないよう 200ms (5Hz) ごとに表示
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 200,
          "[%-8s] 経過: %4.1fs | 指令: %4.2f m/s (%5.2f km/h) | 実測: %4.2f m/s",
          state_.c_str(), elapsed_total, target_v_, target_v_ * 3.6, current_v_drive_);
    }

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = target_v_;
    twist.angular.z = 0.0;
    cmd_pub_->publish(twist);
  }

  // パラメータ
  double g_, mu_, h_, Lf_, Lr_, L_;
  double alpha1_, alpha2_, beta1_, beta2_;
  double v_max1_, v_max2_, v_dec_target_;
  double cruise1_time_, cruise2_time_, cruise3_time_;
  double max_total_time_;

  // 計算済み加減速度
  double a_acc1_, a_acc2_, a_dec1_, a_dec2_;
  double dt_;

  // 状態変数
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
  auto node = std::make_shared<MovingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
