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
#include "std_msgs/msg/float64_multi_array.hpp"

class SlipMonitorNode : public rclcpp::Node {
public:
  SlipMonitorNode() : Node("slip_monitor_node") {
    this->declare_parameter<double>("wheel.diameter", 0.254);
    this->declare_parameter<double>("wheel.tread", 0.60);
    this->declare_parameter<std::string>("output_dir", ".");

    diameter_ = this->get_parameter("wheel.diameter").as_double();
    tread_ = this->get_parameter("wheel.tread").as_double();
    output_dir_ = this->get_parameter("output_dir").as_string();

    init_csv_file();

    slip_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/slip_monitor/slip", 10);

    rclcpp::QoS can_qos(rclcpp::KeepLast(2));
    can_qos.best_effort();

    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/aiformula_sensing/vehicle_info", can_qos,
        std::bind(&SlipMonitorNode::can_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_fusion", 10,
        std::bind(&SlipMonitorNode::odom_callback, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/aiformula_control/twist_mux/cmd_vel", 10,
        std::bind(&SlipMonitorNode::cmd_vel_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Slip Monitor Node Initialized.");
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
                   "slip_l,slip_r,omega_wheel,omega_imu,spin_diff,cmd_vx,cmd_wz\n";
      csv_file_.flush();
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = msg;
    has_odom_ = true;
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vx_ = msg->linear.x;
    cmd_wz_ = msg->angular.z;
  }

  void can_callback(const can_msgs::msg::Frame::SharedPtr msg) {
    if (msg->id != 0x711 || msg->data.size() < 8) return;

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
    double omega_imu = 0.0;

    if (has_odom_ && latest_odom_) {
      v_sub_x = latest_odom_->twist.twist.linear.x;
      omega_imu = latest_odom_->twist.twist.angular.z;

      rclcpp::Time now_time = this->now();
      rclcpp::Time odom_time = latest_odom_->header.stamp;
      if (odom_time.nanoseconds() > 0) {
        double time_diff = std::abs((now_time - odom_time).seconds());
        if (time_diff > 0.05) {
          RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "Odom timestamp diff > 50ms (%.3f s)", time_diff);
        }
      }
    }

    double v_R_wheel = static_cast<double>(rpm_r) * (1.0 / 60.0) * M_PI * diameter_;
    double v_L_wheel = static_cast<double>(rpm_l) * (1.0 / 60.0) * M_PI * diameter_;

    double v_R_real = v_sub_x + omega_imu * (tread_ / 2.0);
    double v_L_real = v_sub_x - omega_imu * (tread_ / 2.0);

    double slip_r = (std::abs(v_R_wheel) >= 0.1) ? (v_R_wheel - v_R_real) / std::abs(v_R_wheel) : 0.0;
    double slip_l = (std::abs(v_L_wheel) >= 0.1) ? (v_L_wheel - v_L_real) / std::abs(v_L_wheel) : 0.0;

    double omega_wheel = (v_R_wheel - v_L_wheel) / tread_;
    double spin_diff = omega_wheel - omega_imu;

    auto array_msg = std_msgs::msg::Float64MultiArray();
    array_msg.data = {slip_l, slip_r, spin_diff};
    slip_pub_->publish(array_msg);

    if (csv_file_.is_open()) {
      csv_file_ << std::fixed << std::setprecision(6)
                << this->now().seconds() << ","
                << rpm_l << "," << rpm_r << ","
                << v_L_wheel << "," << v_R_wheel << ","
                << v_L_real << "," << v_R_real << ","
                << slip_l << "," << slip_r << ","
                << omega_wheel << "," << omega_imu << ","
                << spin_diff << ","
                << cmd_vx_ << "," << cmd_wz_ << "\n";
      csv_file_.flush();
    }
  }

  double diameter_, tread_;
  std::string output_dir_;
  std::ofstream csv_file_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  bool has_odom_{false};
  double cmd_vx_{0.0}, cmd_wz_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr slip_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlipMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
