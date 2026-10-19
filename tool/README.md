# 双雷达时间戳诊断工具

`lidar_timestamp_check.py` 只订阅、不修改消息、不配置设备、不启动驱动。
同时适用于软同步和后续硬同步：用不同输出目录保存结果，保持相同采集时长和阈值。

## 真实雷达测试

从项目根目录运行，先启动发布两个独立话题的驱动（dual_split），确认实际话题名：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list
python3 tool/lidar_timestamp_check.py \
  --front /livox/lidar_192_168_1_135 \
  --back /livox/lidar_192_168_1_122 \
  --duration 30 --tolerance-ms 5 \
  --output data/results/soft_timestamp_run1
```

Dust 项目对应的话题可替换为 `/livox/lidar_192_168_25_100` 和
`/livox/lidar_192_168_25_136`。PointCloud2 使用
`--message-type sensor_msgs/msg/PointCloud2`。两路须使用相同消息类型。
QoS 使用 SensorDataQoS（best effort），统计的是本工具实际收到的消息。
不要订阅融合输出或软同步包的 `paired/*` 做输入，否则无法观察筛选前的数据。

硬同步开启后，保持输入话题和命令参数一致，仅改输出目录如
`--output data/results/hard_timestamp_run1`，并单独记录设备 PTP/PPS/GPS 同步状态。
现有结果目录不允许覆盖。Ctrl+C 可提前结束并保存。

## 看哪些指标

- `streams.front/back.stamp_interval_ms`：各雷达连续消息头时间戳差，含均值/P50/P95/P99/最小/最大。
  例如 100Hz 预期约 10ms，10Hz 预期约 100ms；大间隔可能来自漏帧、停流或时间跳变。
- `average_hz`：正向时间戳间隔换算的平均频率；有回退时不可独立用它判断正常。
- `receive_interval_ms`：主机单调时钟记录的回调间隔，与采样时间戳间隔区分，帮助观察接收抖动。
- `non_increasing` / `zero_stamp`：时间戳重复或回退次数、零时间戳数量，正常应为 0。
- `nearest_delta_ms`：每帧 front 在整个 back 数据中寻找最近时间戳，差值为 back-front，正值表示 back 较晚。
- `nearest_abs_delta_ms`：上述差值绝对值，不先用容差过滤，关注 P95/P99/最大值。
  最近邻允许 back 帧复用，因此必须结合覆盖率阅读。
- `window_pairs` / `coverage.front/back`：指定容差内的按时间顺序一对一配对，双方各自被配对的比例。
  这是时间窗口匹配，不是严格同一采集序号匹配。

默认判定条件：双方各至少 100 帧，无零时间戳/重复/回退，双方在 ±5ms 内配对覆盖率均至少 95%。
可用 `--min-samples`、`--coverage` 和 `--tolerance-ms` 修改。
该判定只针对消息头接近程度，帧间隔是否符合设备设定频率需同时查看统计。
短时断流是否不可接受，也需要按应用要求判断。

**周期数据可能错配到前后帧；相差整帧周期仍可能看起来很接近。**
两台驱动都用主机接收时刻打时间戳时，结果接近也不能证明真实采样同步。
本工具不检测逐点 offset_time，不验证物理硬同步，不以回调时刻代替采样时刻。

## 输出与离线分析

`raw.csv` 保存 source、原始 stamp_ns、receive_monotonic_ns，`report.json` 保存参数和汇总。
采集阶段每两秒输出双方接收计数，结束后显示完整统计。
离线重新调整容差不需要 ROS：

```bash
python3 tool/lidar_timestamp_check.py \
  --from-csv data/results/soft_timestamp_run1/raw.csv \
  --tolerance-ms 1 --output data/results/soft_timestamp_1ms
python3 -m unittest discover -s tool -v
```

支持外部启动的 rosbag 回放。工具不依赖 `/clock`，帧间隔直接读取原始 header，
主机接收间隔会受回放速度影响，因此软/硬对比应采用相同回放倍率。

当前实机默认 IP 已更新为 `.25.100` / `.25.136`。
按 Dust 的 100Hz 配置与软/硬隔离运行方式见 [Dust配置说明.md](Dust配置说明.md)。
