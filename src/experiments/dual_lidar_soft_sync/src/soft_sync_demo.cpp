#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>

using Msg = livox_ros_driver2::msg::CustomMsg;
using Policy = message_filters::sync_policies::ApproximateTime<Msg, Msg>;
class Demo : public rclcpp::Node {
public:
  Demo() : Node("soft_sync_demo") {
    const auto front = declare_parameter<std::string>("front_topic", "input/front");
    const auto back = declare_parameter<std::string>("back_topic", "input/back");
    mock_ = declare_parameter<bool>("mock", true);
    duration_ = declare_parameter<double>("duration_s", 8.0);
    offset_ = declare_parameter<double>("mock_offset_ms", 3.0);
    jitter_ = declare_parameter<double>("mock_jitter_ms", 1.0);
    drop_every_ = declare_parameter<int>("mock_drop_every", 0);
    const auto tolerance = declare_parameter<double>("max_delta_ms", 5.0);
    const auto queue = declare_parameter<int>("queue_size", 10);
    const auto path = declare_parameter<std::string>("csv_path", "/tmp/soft_sync_pairs.csv");
    if (queue < 2 || tolerance <= 0 || duration_ <= 0 || drop_every_ < 0)
      throw std::runtime_error("queue>=2, max_delta_ms>0, duration_s>0, mock_drop_every>=0 required");
    csv_.open(path, std::ios::out | std::ios::trunc);
    if (!csv_) throw std::runtime_error("Cannot open CSV: " + path);
    csv_ << "front_stamp_ns,back_stamp_ns,delta_ms,callback_age_front_ms,front_received,back_received,pairs\n";
    // SensorDataQoS supports live best-effort drivers and reliable bag publishers.
    front_.subscribe(this, front, rmw_qos_profile_sensor_data);
    back_.subscribe(this, back, rmw_qos_profile_sensor_data);
    front_.registerCallback([this](Msg::ConstSharedPtr) { ++front_count_; });
    back_.registerCallback([this](Msg::ConstSharedPtr) { ++back_count_; });
    Policy policy(queue);
    policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(tolerance / 1000.0));
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(policy);
    sync_->connectInput(front_, back_);
    paired_front_ = create_publisher<Msg>("paired/front", 10);
    paired_back_ = create_publisher<Msg>("paired/back", 10);
    std::function<void(const Msg::ConstSharedPtr&, const Msg::ConstSharedPtr&)> callback = [this](const Msg::ConstSharedPtr& a, const Msg::ConstSharedPtr& b) {
      const auto ta = rclcpp::Time(a->header.stamp).nanoseconds();
      const auto tb = rclcpp::Time(b->header.stamp).nanoseconds();
      ++pairs_;
      csv_ << ta << ',' << tb << ',' << (tb-ta)/1e6 << ',' << (now().nanoseconds()-ta)/1e6
           << ',' << front_count_ << ',' << back_count_ << ',' << pairs_ << '\n';
      // Preserve all original per-point timestamps; do not impersonate a fused LIO cloud.
      paired_front_->publish(*a);
      paired_back_->publish(*b);
    };
    sync_->registerCallback(std::bind(callback, std::placeholders::_1, std::placeholders::_2));
    if (mock_) {
      mock_front_ = create_publisher<Msg>(front, rclcpp::SensorDataQoS());
      mock_back_ = create_publisher<Msg>(back, rclcpp::SensorDataQoS());
      generator_ = create_wall_timer(std::chrono::milliseconds(10), [this] { generate(); });
    }
    start_ = std::chrono::steady_clock::now();
    stop_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      if (std::chrono::duration<double>(std::chrono::steady_clock::now()-start_).count() >= duration_) {
        csv_.flush();
        std::ofstream summary(csv_path_ + ".counts");
        summary << front_count_ << ',' << back_count_ << ',' << pairs_ << '\n';
        RCLCPP_INFO(get_logger(), "received front=%zu back=%zu pairs=%zu; unmatched includes pending queue", front_count_, back_count_, pairs_);
        rclcpp::shutdown();
      }
    });
    csv_path_ = path;
    RCLCPP_INFO(get_logger(), "SOFTWARE experiment mock=%d max_delta=%.3fms CSV=%s", mock_, tolerance, path.c_str());
  }
private:
  void generate() {
    ++sequence_;
    // Leave the final 0.5s to drain DDS and synchronizer queues.
    if (std::chrono::duration<double>(std::chrono::steady_clock::now()-start_).count() > duration_-0.5) return;
    Msg a;
    auto base = now().nanoseconds();
    a.header.stamp = rclcpp::Time(base);
    a.header.frame_id = "mock_front";
    a.timebase = base;
    a.point_num = 2;
    a.points.resize(2);
    a.points[0].x = 1.0f;
    a.points[1].x = 1.1f;
    a.points[1].offset_time = 1000000;
    mock_front_->publish(a);
    if (drop_every_ && sequence_ % drop_every_ == 0) return;
    Msg b = a;
    const auto shift = static_cast<int64_t>((offset_ + jitter_*std::sin(sequence_*0.7))*1e6);
    b.header.stamp = rclcpp::Time(base+shift);
    b.header.frame_id = "mock_back";
    b.timebase = base+shift;
    mock_back_->publish(b);
  }
  bool mock_;
  double duration_, offset_, jitter_;
  int drop_every_;
  size_t sequence_{0}, front_count_{0}, back_count_{0}, pairs_{0};
  std::string csv_path_;
  std::ofstream csv_;
  std::chrono::steady_clock::time_point start_;
  message_filters::Subscriber<Msg> front_, back_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Publisher<Msg>::SharedPtr paired_front_, paired_back_, mock_front_, mock_back_;
  rclcpp::TimerBase::SharedPtr generator_, stop_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<Demo>()); }
  catch (const std::exception& e) { fprintf(stderr, "%s\n", e.what()); return 1; }
  return 0;
}
