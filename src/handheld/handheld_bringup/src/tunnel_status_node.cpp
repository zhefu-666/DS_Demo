// 隧道/洞口状态判定节点。
// 订阅 ROGMap 投影层类型栅格 /rog_map/layer_type（UNKNOWN=0, FREE=33, PASSABLE=66, OCCUPIED=100），
// 统计 ROI 内各类格子占比，输出 PASSABLE / BLOCKED / UNKNOWN，并可写 CSV 用于测量
// “放入/移走箱体”到状态切换的延迟。
//
// ROI 两种模式：
//   fixed         : ROI 在栅格坐标系（通常 camera_init）中固定，适合洞口静态测试；
//   body_relative : ROI 跟随里程计（机体前方），适合手持推进测试。

#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
constexpr int8_t kUnknown = 0;
constexpr int8_t kFree = 33;
constexpr int8_t kPassable = 66;
constexpr int8_t kOccupied = 100;
}  // namespace

class TunnelStatusNode : public rclcpp::Node
{
public:
  TunnelStatusNode()
  : Node("tunnel_status_node")
  {
    grid_topic_ = declare_parameter<std::string>("grid_topic", "/rog_map/layer_type");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    roi_mode_ = declare_parameter<std::string>("roi_mode", "fixed");
    // ROI 矩形：fixed 模式为栅格坐标系下 [x_min, x_max, y_min, y_max]；
    // body_relative 模式为机体坐标系下（x 前、y 左）。
    roi_x_min_ = declare_parameter<double>("roi.x_min", 0.5);
    roi_x_max_ = declare_parameter<double>("roi.x_max", 1.5);
    roi_y_min_ = declare_parameter<double>("roi.y_min", -0.4);
    roi_y_max_ = declare_parameter<double>("roi.y_max", 0.4);
    blocked_occ_ratio_ = declare_parameter<double>("blocked_occupied_ratio", 0.15);
    unknown_ratio_ = declare_parameter<double>("unknown_ratio", 0.50);
    debounce_count_ = declare_parameter<int>("debounce_count", 3);
    csv_path_ = declare_parameter<std::string>("csv_path", "");

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      grid_topic_, sensor_qos,
      std::bind(&TunnelStatusNode::gridCallback, this, std::placeholders::_1));
    if (roi_mode_ == "body_relative") {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(10),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          odom_ = msg;
        });
    } else if (roi_mode_ != "fixed") {
      RCLCPP_WARN(get_logger(), "未知 roi_mode '%s'，改用 fixed。", roi_mode_.c_str());
      roi_mode_ = "fixed";
    }
    // 人工事件（箱体放入/移走），由 tunnel_event_marker.py 发布，用于计算状态切换延迟。
    event_sub_ = create_subscription<std_msgs::msg::String>(
      "/tunnel_event", rclcpp::QoS(10),
      std::bind(&TunnelStatusNode::eventCallback, this, std::placeholders::_1));

    status_pub_ = create_publisher<std_msgs::msg::String>("/tunnel_status", rclcpp::QoS(10));
    detail_pub_ = create_publisher<std_msgs::msg::String>(
      "/tunnel_status/detail", rclcpp::QoS(10));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/tunnel_status/roi", rclcpp::QoS(1));

    if (!csv_path_.empty()) {
      csv_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (!csv_) {
        RCLCPP_ERROR(get_logger(), "无法写入 CSV: %s", csv_path_.c_str());
      } else {
        csv_ << "stamp,total,unknown,free,passable,occupied,occ_ratio,unk_ratio,"
                "raw_status,stable_status,transition,event,latency_s\n";
      }
    }
    RCLCPP_INFO(get_logger(), "tunnel_status: grid=%s mode=%s ROI x[%.2f,%.2f] y[%.2f,%.2f]",
      grid_topic_.c_str(), roi_mode_.c_str(), roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_);
  }

private:
  struct Counts
  {
    int total{0};
    int unknown{0};
    int free{0};
    int passable{0};
    int occupied{0};
  };

  // ROI 中心点与朝向（栅格坐标系）。fixed 模式为原点与 0 朝向。
  bool roiPose(double & ox, double & oy, double & yaw) const
  {
    if (roi_mode_ == "fixed") {
      ox = 0.0;
      oy = 0.0;
      yaw = 0.0;
      return true;
    }
    if (!odom_) {
      return false;
    }
    const auto & p = odom_->pose.pose.position;
    const auto & q = odom_->pose.pose.orientation;
    ox = p.x;
    oy = p.y;
    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return true;
  }

  void gridCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid)
  {
    double ox = 0.0;
    double oy = 0.0;
    double yaw = 0.0;
    if (!roiPose(ox, oy, yaw)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "等待里程计 %s", odom_topic_.c_str());
      return;
    }
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double res = grid->info.resolution;
    const double gx0 = grid->info.origin.position.x;
    const double gy0 = grid->info.origin.position.y;
    const int width = static_cast<int>(grid->info.width);
    const int height = static_cast<int>(grid->info.height);

    // 在 ROI 局部坐标系内按分辨率采样，映射回栅格索引（行主序 y*width+x）。
    Counts counts;
    for (double lx = roi_x_min_ + 0.5 * res; lx < roi_x_max_; lx += res) {
      for (double ly = roi_y_min_ + 0.5 * res; ly < roi_y_max_; ly += res) {
        const double wx = ox + c * lx - s * ly;
        const double wy = oy + s * lx + c * ly;
        const int ix = static_cast<int>(std::floor((wx - gx0) / res));
        const int iy = static_cast<int>(std::floor((wy - gy0) / res));
        ++counts.total;
        if (ix < 0 || iy < 0 || ix >= width || iy >= height) {
          ++counts.unknown;  // 超出局部地图视为未知
          continue;
        }
        switch (grid->data[static_cast<size_t>(iy) * width + ix]) {
          case kOccupied: ++counts.occupied; break;
          case kPassable: ++counts.passable; break;
          case kFree: ++counts.free; break;
          case kUnknown:
          default: ++counts.unknown; break;
        }
      }
    }
    if (counts.total == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ROI 尺寸为 0，检查 roi.* 参数");
      return;
    }

    const double occ_ratio = static_cast<double>(counts.occupied) / counts.total;
    const double unk_ratio = static_cast<double>(counts.unknown) / counts.total;
    std::string raw = "PASSABLE";
    if (occ_ratio >= blocked_occ_ratio_) {
      raw = "BLOCKED";
    } else if (unk_ratio >= unknown_ratio_) {
      raw = "UNKNOWN";
    }

    // 去抖：连续 debounce_count 帧一致才切换稳定状态。
    if (raw == candidate_) {
      ++candidate_count_;
    } else {
      candidate_ = raw;
      candidate_count_ = 1;
    }
    bool transition = false;
    if (candidate_count_ >= debounce_count_ && candidate_ != stable_) {
      RCLCPP_INFO(get_logger(), "状态切换 %s -> %s (occ=%.2f unk=%.2f)",
        stable_.c_str(), candidate_.c_str(), occ_ratio, unk_ratio);
      stable_ = candidate_;
      transition = true;
    }

    const rclcpp::Time stamp(grid->header.stamp, get_clock()->get_clock_type());
    double latency = -1.0;
    std::string event_for_row;
    if (transition && pending_event_) {
      latency = (stamp - pending_event_stamp_).seconds();
      event_for_row = pending_event_name_;
      RCLCPP_INFO(get_logger(), "事件 '%s' 到状态切换延迟 %.3f s", pending_event_name_.c_str(), latency);
      pending_event_ = false;
    }

    std_msgs::msg::String status;
    status.data = stable_;
    status_pub_->publish(status);

    std::ostringstream detail;
    detail << std::fixed << std::setprecision(3) << "{\"raw\":\"" << raw << "\",\"stable\":\"" << stable_
           << "\",\"total\":" << counts.total << ",\"unknown\":" << counts.unknown
           << ",\"free\":" << counts.free << ",\"passable\":" << counts.passable
           << ",\"occupied\":" << counts.occupied << ",\"occ_ratio\":" << occ_ratio
           << ",\"unk_ratio\":" << unk_ratio << "}";
    std_msgs::msg::String detail_msg;
    detail_msg.data = detail.str();
    detail_pub_->publish(detail_msg);

    if (csv_) {
      csv_ << std::fixed << std::setprecision(6) << stamp.seconds() << ',' << counts.total << ','
           << counts.unknown << ',' << counts.free << ',' << counts.passable << ','
           << counts.occupied << ',' << occ_ratio << ',' << unk_ratio << ',' << raw << ','
           << stable_ << ',' << (transition ? 1 : 0) << ',' << event_for_row << ',' << latency
           << '\n';
      csv_.flush();
    }

    publishMarker(grid->header.frame_id, ox, oy, yaw);
  }

  void eventCallback(std_msgs::msg::String::ConstSharedPtr msg)
  {
    pending_event_ = true;
    pending_event_name_ = msg->data;
    pending_event_stamp_ = now();
    RCLCPP_INFO(get_logger(), "收到事件 '%s'，开始计时", msg->data.c_str());
    if (csv_) {
      csv_ << std::fixed << std::setprecision(6) << pending_event_stamp_.seconds()
           << ",,,,,,,,," << stable_ << ",0," << msg->data << ",\n";
    }
  }

  void publishMarker(const std::string & frame, double ox, double oy, double yaw)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = now();
    m.ns = "tunnel_roi";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.03;
    m.pose.orientation.w = 1.0;
    m.color.a = 1.0;
    if (stable_ == "BLOCKED") {
      m.color.r = 1.0;
    } else if (stable_ == "PASSABLE") {
      m.color.g = 1.0;
    } else {
      m.color.r = 1.0;
      m.color.g = 1.0;
    }
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double xs[5] = {roi_x_min_, roi_x_max_, roi_x_max_, roi_x_min_, roi_x_min_};
    const double ys[5] = {roi_y_min_, roi_y_min_, roi_y_max_, roi_y_max_, roi_y_min_};
    for (int i = 0; i < 5; ++i) {
      geometry_msgs::msg::Point p;
      p.x = ox + c * xs[i] - s * ys[i];
      p.y = oy + s * xs[i] + c * ys[i];
      p.z = 0.05;
      m.points.push_back(p);
    }

    visualization_msgs::msg::Marker text = m;
    text.id = 1;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.points.clear();
    text.scale.z = 0.2;
    text.pose.position = m.points[2];
    text.pose.position.z = 0.4;
    text.text = stable_;

    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(m);
    arr.markers.push_back(text);
    marker_pub_->publish(arr);
  }

  std::string grid_topic_;
  std::string odom_topic_;
  std::string roi_mode_;
  double roi_x_min_;
  double roi_x_max_;
  double roi_y_min_;
  double roi_y_max_;
  double blocked_occ_ratio_;
  double unknown_ratio_;
  int debounce_count_;
  std::string csv_path_;
  std::ofstream csv_;

  std::string stable_{"UNKNOWN"};
  std::string candidate_{"UNKNOWN"};
  int candidate_count_{0};

  bool pending_event_{false};
  std::string pending_event_name_;
  rclcpp::Time pending_event_stamp_;

  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detail_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TunnelStatusNode>());
  rclcpp::shutdown();
  return 0;
}
