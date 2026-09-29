// 独立运行 ROGMap 的最小宿主节点。
// ROGMapROS 内部创建订阅（cloud_topic / odom_topic）、更新定时器和可视化发布，
// 这里只负责加载 "rog_map.*" 参数、提供 TF buffer 并用多线程执行器驱动回调组。

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rog_map_ros/rog_map_ros2.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rog_map_node");

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  rog_map::Config cfg;
  std::shared_ptr<rog_map::ROGMapROS> rog_map_ros;
  try {
    cfg.loadFromRosNode(node, "rog_map");
    rog_map_ros = std::make_shared<rog_map::ROGMapROS>(node, cfg, tf_buffer);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "ROGMap 初始化失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "ROGMap 已启动，等待点云与里程计输入。");

  // ROGMap 的点云、里程计、更新和可视化分别位于独立的 MutuallyExclusive 回调组。
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
