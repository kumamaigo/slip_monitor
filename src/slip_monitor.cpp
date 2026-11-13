#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class SlipMonitorNode : public rclcpp::Node {
public:
  SlipMonitorNode() : Node("slip_monitor_node") {
    this->declare_parameter<double>("wheel.diameter", 0.254);
    this->declare_parameter<double>("wheel.tread", 0.60);
    this->declare_parameter<double>("wheel.caster_length", 0.815);
    this->declare_parameter<std::string>("output_dir", ".");
    this->declare_parameter<std::string>("sub_odom_topic", "/sub_odom");
    this->declare_parameter<std::string>("imu_topic", "/aiformula_sensing/vectornav/imu");
    this->declare_parameter<std::string>("gyro_odom_topic",
                                         "/aiformula_sensing/gyro_odometry_publisher/odom");

    diameter_ = this->get_parameter("wheel.diameter").as_double();
    tread_ = this->get_parameter("wheel.tread").as_double();
    caster_length_ = this->get_parameter("wheel.caster_length").as_double();
    output_dir_ = this->get_parameter("output_dir").as_string();
    std::string sub_odom_topic = this->get_parameter("sub_odom_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string gyro_odom_topic = this->get_parameter("gyro_odom_topic").as_string();

    init_csv_file();

    slip_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/slip_monitor/slip", 10);

    rclcpp::QoS can_qos(rclcpp::KeepLast(2));
    can_qos.best_effort();

    // 1. CANから駆動輪RPMを取得
    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/aiformula_sensing/vehicle_info", can_qos,
        std::bind(&SlipMonitorNode::can_callback, this, std::placeholders::_1));

    // 2. 従属輪オドメトリ (/sub_odom) から縦速度と横由来角速度を取得
    sub_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        sub_odom_topic, 10,
        std::bind(&SlipMonitorNode::sub_odom_callback, this, std::placeholders::_1));

    // 3. VectorNav IMU から実測旋回角速度を取得
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&SlipMonitorNode::imu_callback, this, std::placeholders::_1));

    // 4. 比較対象オドメトリ (駆動輪RPM + ZED IMU) から理論座標 (x1, y1) を取得
    gyro_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        gyro_odom_topic, 10,
        std::bind(&SlipMonitorNode::gyro_odom_callback, this, std::placeholders::_1));

    // 5. 速度指令値 (cmd_vel) を取得
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/aiformula_control/twist_mux/cmd_vel", 10,
        std::bind(&SlipMonitorNode::cmd_vel_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Slip Monitor Node Initialized (sub_odom: %s, imu: %s, gyro_odom: %s)",
                sub_odom_topic.c_str(), imu_topic.c_str(), gyro_odom_topic.c_str());
  }

  ~SlipMonitorNode() {
    if (csv_file_.is_open()) csv_file_.close();
  }

private:
  void init_csv_file() {
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << output_dir_ << "/slip_log_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
    std::string filename = oss.str();

    csv_file_.open(filename, std::ios::out);
    if (csv_file_.is_open()) {
      csv_file_ << "time,rpm_l,rpm_r,v_l_wheel,v_r_wheel,v_l_real,v_r_real,"
                   "slip_l,slip_r,omega_wheel,omega_imu,spin_diff,"
                   "v_sub_x,sub_wz,vy_real,x1,y1,yaw1,x2,y2,yaw2,pos_error,cmd_vx,cmd_wz\n";
      csv_file_.flush();
    }
  }

  void sub_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_sub_odom_ = msg;
    has_sub_odom_ = true;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    latest_imu_ = msg;
    has_imu_ = true;
  }

  void gyro_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double raw_x = msg->pose.pose.position.x;
    double raw_y = msg->pose.pose.position.y;
    const auto &q = msg->pose.pose.orientation;
    double raw_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // 起動時の座標・向きを (0, 0, 0) に揃えて (x2, y2, yaw2) と比較できるようにする
    if (!has_gyro_odom_init_) {
      init_x1_ = raw_x;
      init_y1_ = raw_y;
      init_yaw1_ = raw_yaw;
      has_gyro_odom_init_ = true;
    }

    double dx = raw_x - init_x1_;
    double dy = raw_y - init_y1_;
    x1_ = dx * std::cos(-init_yaw1_) - dy * std::sin(-init_yaw1_);
    y1_ = dx * std::sin(-init_yaw1_) + dy * std::cos(-init_yaw1_);
    yaw1_ = std::atan2(std::sin(raw_yaw - init_yaw1_), std::cos(raw_yaw - init_yaw1_));
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vx_ = msg->linear.x;
    cmd_wz_ = msg->angular.z;
  }

  void can_callback(const can_msgs::msg::Frame::SharedPtr msg) {
    if (msg->id != 0x711 || msg->data.size() < 8) return;

    rclcpp::Time now_time = this->now();
    double now_sec = now_time.seconds();

    int32_t rpm_r = static_cast<int32_t>(
        (static_cast<uint32_t>(msg->data[0])) |
        (static_cast<uint32_t>(msg->data[1]) << 8) |
        (static_cast<uint32_t>(msg->data[2]) << 16) |
        (static_cast<uint32_t>(msg->data[3]) << 24));
    int32_t rpm_l = static_cast<int32_t>(
        (static_cast<uint32_t>(msg->data[4])) |
        (static_cast<uint32_t>(msg->data[5]) << 8) |
        (static_cast<uint32_t>(msg->data[6]) << 16) |
        (static_cast<uint32_t>(msg->data[7]) << 24));

    double v_sub_x = 0.0;
    double sub_wz = 0.0;
    double omega_imu = 0.0;

    if (has_sub_odom_ && latest_sub_odom_) {
      v_sub_x = latest_sub_odom_->twist.twist.linear.x;
      sub_wz = latest_sub_odom_->twist.twist.angular.z;  // caster_vel_y / l_

      rclcpp::Time sub_time = latest_sub_odom_->header.stamp;
      if (sub_time.nanoseconds() > 0) {
        double time_diff = std::abs((now_time - sub_time).seconds());
        if (time_diff > 0.05) {
          RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "Sub-wheel odom timestamp diff > 50ms (%.3f s)", time_diff);
        }
      }
    }

    if (has_imu_ && latest_imu_) {
      omega_imu = latest_imu_->angular_velocity.z;

      rclcpp::Time imu_time = latest_imu_->header.stamp;
      if (imu_time.nanoseconds() > 0) {
        double time_diff = std::abs((now_time - imu_time).seconds());
        if (time_diff > 0.05) {
          RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "IMU timestamp diff > 50ms (%.3f s)", time_diff);
        }
      }
    }

    // --- 1. 駆動輪の空転・ロック検知 ---
    double v_R_wheel = static_cast<double>(rpm_r) * (1.0 / 60.0) * M_PI * diameter_;
    double v_L_wheel = static_cast<double>(rpm_l) * (1.0 / 60.0) * M_PI * diameter_;

    double v_R_real = v_sub_x + omega_imu * (tread_ / 2.0);
    double v_L_real = v_sub_x - omega_imu * (tread_ / 2.0);

    double denom_r = std::max(std::abs(v_R_wheel), std::abs(v_R_real));
    double denom_l = std::max(std::abs(v_L_wheel), std::abs(v_L_real));
    double slip_r = (denom_r >= 0.1) ? (v_R_wheel - v_R_real) / denom_r : 0.0;
    double slip_l = (denom_l >= 0.1) ? (v_L_wheel - v_L_real) / denom_l : 0.0;

    // --- 2. 機体のスピン評価用：従属輪＋IMUによる実測座標 (x2, y2, yaw2) の時間積分 ---
    double vy_real = caster_length_ * (omega_imu - sub_wz);  // 旋回成分を除いた車体の横滑り速度

    if (has_prev_time_) {
      double dt = now_sec - prev_time_sec_;
      if (dt > 0.0 && dt < 0.5) {
        double mid_yaw = yaw2_ + 0.5 * omega_imu * dt;
        x2_ += (v_sub_x * std::cos(mid_yaw) - vy_real * std::sin(mid_yaw)) * dt;
        y2_ += (v_sub_x * std::sin(mid_yaw) + vy_real * std::cos(mid_yaw)) * dt;
        yaw2_ += omega_imu * dt;
        yaw2_ = std::atan2(std::sin(yaw2_), std::cos(yaw2_));
      }
    }
    prev_time_sec_ = now_sec;
    has_prev_time_ = true;

    double omega_wheel = (v_R_wheel - v_L_wheel) / tread_;
    double spin_diff = omega_wheel - omega_imu;
    double pos_error = std::hypot(x2_ - x1_, y2_ - y1_);

    auto array_msg = std_msgs::msg::Float64MultiArray();
    array_msg.data = {slip_l, slip_r, spin_diff, pos_error};
    slip_pub_->publish(array_msg);

    if (csv_file_.is_open()) {
      csv_file_ << std::fixed << std::setprecision(6)
                << now_sec << ","
                << rpm_l << "," << rpm_r << ","
                << v_L_wheel << "," << v_R_wheel << ","
                << v_L_real << "," << v_R_real << ","
                << slip_l << "," << slip_r << ","
                << omega_wheel << "," << omega_imu << ","
                << spin_diff << ","
                << v_sub_x << "," << sub_wz << "," << vy_real << ","
                << x1_ << "," << y1_ << "," << yaw1_ << ","
                << x2_ << "," << y2_ << "," << yaw2_ << ","
                << pos_error << ","
                << cmd_vx_ << "," << cmd_wz_ << "\n";
      csv_file_.flush();
    }
  }

  double diameter_, tread_, caster_length_;
  std::string output_dir_;
  std::ofstream csv_file_;

  nav_msgs::msg::Odometry::SharedPtr latest_sub_odom_;
  sensor_msgs::msg::Imu::SharedPtr latest_imu_;
  bool has_sub_odom_{false};
  bool has_imu_{false};

  // 理論座標 (x1, y1, yaw1) 用
  bool has_gyro_odom_init_{false};
  double init_x1_{0.0}, init_y1_{0.0}, init_yaw1_{0.0};
  double x1_{0.0}, y1_{0.0}, yaw1_{0.0};

  // 実測座標 (x2, y2, yaw2) 用
  bool has_prev_time_{false};
  double prev_time_sec_{0.0};
  double x2_{0.0}, y2_{0.0}, yaw2_{0.0};

  double cmd_vx_{0.0}, cmd_wz_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr slip_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gyro_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlipMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
