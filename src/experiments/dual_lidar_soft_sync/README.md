# 双雷达软时间同步实验

这是从 Dust_sentry_navigation 的 `ApproximateTime<CustomMsg, CustomMsg>` 方案抽出的独立测试包。
仅测试消息级配对，保留原始 header、timebase、offset_time，不融合点云、不接管驱动、不启动 LIO。
增加显式最大配对时间差，避免将 queue_size=10 误读为 10ms。

## 构建与模拟测试

在 DS_Demo 根目录执行（已有 Livox 驱动构建产物）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select dual_lidar_soft_sync
source install/setup.bash
ros2 launch dual_lidar_soft_sync soft_sync.launch.py
python3 src/experiments/dual_lidar_soft_sync/scripts/summarize.py /tmp/soft_sync_pairs.csv
# 三组真实 DDS + ApproximateTime 自动断言，隔离在 ROS_DOMAIN_ID=178
ros2 run dual_lidar_soft_sync run_mock_tests.py /tmp/dual_lidar_soft_sync
```

干净工作区需先构建消息依赖：`colcon build --packages-up-to dual_lidar_soft_sync`。
模拟源为 100Hz，默认时间偏差 3ms、正弦抖动 ±1ms；数据是两个合成点，非真实扫描。
mock_offset_ms 改的是消息时间戳，未模拟网络延迟。超过一帧周期的偏差可能导致跨帧配对，
所以模拟测试不能证明同一次采集被匹配，也不能证明物理同步精度。

## 接入分路雷达或 rosbag

先用现有 `dual_split` 驱动或回放分别发布两路 CustomMsg，再启动观察节点：

```bash
ros2 launch dual_lidar_soft_sync soft_sync.launch.py \
  mock:=false duration_s:=60.0 \
  front_topic:=/livox/lidar_192_168_1_135 \
  back_topic:=/livox/lidar_192_168_1_122 \
  max_delta_ms:=5.0 csv_path:=/tmp/soft_live.csv
```

输入话题可覆盖，应以 `ros2 topic list` 的实际分路话题为准。
对于 bag，先启动本节点，再从另一终端回放；若使用 `ros2 bag play --clock`，加 `use_sim_time:=true`。
运行时长始终按主机单调时钟计算，启动前确保 bag 有足够数据。
QoS 为 SensorDataQoS。节点不会开启/关闭 PTP 或修改雷达配置。
输出仅为 `/sync_compare/soft/paired/front`、`/sync_compare/soft/paired/back`，保持各自坐标系。
它们不是一个可直接替代 `/merged_cloud` 的融合输出。

## 指标与后续硬同步对比边界

CSV 字段固定为：
`front_stamp_ns,back_stamp_ns,delta_ms,callback_age_front_ms,front_received,back_received,pairs`。
`delta_ms = back - front`；`callback_age_front_ms = 节点ROS时间 - front时间戳`，
只有时钟同源且时间戳语义一致时才可解释为消息年龄，不等于纯算法耗时或网络延迟。
正常定时结束后另写 `.counts`，汇总脚本输出绝对偏差均值/P95/最大值、配对率和未配对数。
未配对数包含末尾等待队列、容差拒绝等，不能直接称为雷达或网络丢包率。

| 项目 | 当前软同步 | 后续硬同步实验约定 |
|---|---|---|
| 实现 | 本包，ApproximateTime | 独立包/launch，不能改本包冒充硬同步 |
| 输出命名空间 | /sync_compare/soft | /sync_compare/hard |
| 结果目录 | soft/ | hard/ |
| 时钟依据 | 输入消息时间戳；无硬同步时通常受主机接收延迟影响 | 真实 PTP/PPS/GPS 状态和同源时间证据 |
| 配对与指标 | 当前 CSV 契约 | 复用字段和统计口径，注明配对策略 |

硬同步尚未实现。对比应固定发布频率、运动轨迹/场景、采集时长、QoS、容差和队列，记录
驱动版本、时间同步状态、参数及原始 bag。先比较未筛选输入的时间差，再比较配对率和配对后偏差；
不能只比较通过容差筛选后的 P95（那会掩盖拒绝的数据）。物理同步验收需额外硬件或设备状态证据。

不复刻原融合节点清零逐点时间的行为。后续若实现运动补偿融合，应独立处理坐标外参、统一时间基准、
逐点相对时间与排序，并验证下游 LIO 接口。
