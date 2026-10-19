# Dust 双雷达 100Hz 时间测试配置

独立配置 `dust_dual_split.yaml` / `dust_dual_mid360.json`，旧阶段 2 配置保留。

| 项目 | 本次配置 |
|---|---|
| 电脑有线地址 | 192.168.25.47/24，无默认网关 |
| cloud1 / cloud2 | 192.168.25.100 / 192.168.25.136 |
| 发布频率 | 100Hz，预期帧间隔 10ms |
| 消息 | CustomMsg，各雷达分路发布，保留逐点时间 |
| Dust 导航使用的 IMU | /livox/imu_192_168_25_136 |
| 软件观察器 | /sync_compare/soft，队列10，默认限差1ms（新增测试阈值，非 Dust 原有要求） |

不启动导航/LIO，不更改硬同步状态。测试命名空间不改变设备同步方式。
当前硬同步已开启时，运行软件观察器测到的是“硬同步输入上的软件配对”，不能当作关闭硬同步后的软同步基线。

构建并 source 后运行（确保网口具有上述地址）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select handheld_bringup
source install/setup.bash
# 硬同步组：仅启动 Dust 100Hz 分路驱动
ros2 launch handheld_bringup dust_sync_test.launch.py
# 另一终端：预留至少6秒稳定时间再采集
python3 tool/lidar_timestamp_check.py --duration 30 --tolerance-ms 1 \
  --output data/results/dust_hard_100hz_run1
```

软件配对测试入口（与上面驱动二选一启动，勿启动两套驱动）：

```bash
ros2 launch handheld_bringup dust_sync_test.launch.py soft_observer:=true \
  duration_s:=60 csv_path:=/tmp/dust_soft_pairs.csv
```

上述两个入口都会启动驱动，Ctrl+C 退出；软件观察器计时结束不会自动关闭驱动。

当前机器旧构建的 SDK 库引用已失效。若尚未安装正式 SDK，可在启动前临时指定已验证库：

```bash
export LD_LIBRARY_PATH="/home/liu/Dust_sentry_navigation/src/pb2025_sentry_nav/livox_ros_driver2/Livox-SDK2/lib/amd64:${LD_LIBRARY_PATH:-}"
```

该路径仅适用于本机；其他机器按 `scripts/install_livox_sdk2.sh` 安装对应 SDK。

本配置只对齐 Dust 的采集参数。未复制 Dust 原融合节点清零 offset_time 的行为，也未把 Dust 的安装外参
套到当前设备上。正式接入 Point-LIO 时，仍需实现保留时间的统一坐标融合，并验证与 .136 IMU 的时间和外参。

## 本机验证记录（2026-10-18）

驱动稳定6秒后采集18秒，硬同步保持用户现有配置：

- 两台各1795帧；平均间隔9.99985ms / 9.99999ms，约100Hz。
- 时间戳无重复、回退、零值；间隔范围9.6～10.09252ms / 9.6～10.08ms。
- 两路时间差绝对值P95=0.3539ms、最大0.40655ms；1ms内一对一覆盖率均100%。
- 驱动和可选软件观察器均能启动，观察器正常退出；驱动SIGINT退出时出现exit -7，
  SDK/驱动退出兼容问题尚未修复，不能将本次采集正常等同于驱动生命周期完全验证。
- 结果保存在 `data/results/dust_hard_100hz_20261018/`（数据目录不进Git）。
- 构建、Python语法检查、5项工具测试及git diff检查通过。

## 双雷达 Point-LIO Demo（2026-10-19）

入口 `dust_dual_lio.launch.py`：沿用 navi_minco_bit 的驱动内合并结构，
Livox 驱动与 Point-LIO 同容器，开启 intra-process；不启动导航或底盘控制。

- 主雷达 `.136`，副雷达 `.100`；输出 `/livox/lidar`，仅使用 `/livox/imu_192_168_25_136`。
- 两路驱动改为 20Hz，配对限差 5ms；超限丢较旧帧，任一路掉线将无法继续合并。
- 副雷达经过 `p_136 = Rz(yaw) Ry(pitch) Rx(roll) p_100 + t` 转换，主雷达点不变。
- 合并 timebase 取两帧较早时间，逐点时间重算为相对该 timebase 的偏移，不清零。
- LIO 使用 0.15m 体素，扫描周期 0.05s，不再使用临时 Python 10帧聚合器。
- 新安装不沿用旧机体盲区中心偏移，关闭盲区中心偏移；近距过滤仍由 blind 控制。
- **没有默认安装外参**：`merge_extrinsic` 必须提供 6 个有限数值，缺失时启动失败。
  这些数值必须来自新安装的测量/标定；入口校验格式，不验证标定质量。

先停止 `dust_sync_test`、临时输入适配器和其他 LIO，避免重复驱动或同名输出。
沿用上文 SDK 的 `LD_LIBRARY_PATH`，然后运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# 先将下列变量设为实际标定结果，格式 x,y,z,roll,pitch,yaw，单位 m/rad。
# 不要套用旧的 .135/.122 或 Dust 原车外参。
ros2 launch handheld_bringup dust_dual_lio.launch.py \
  merge_extrinsic:="${DUST_EXTRINSIC:?请先设置新安装的 .100 到 .136 外参}" rviz:=true
```

静止至少 30s 检查 `/aft_mapped_to_init` 漂移，并检查墙面是否重影；
随后缓慢移动检查点云、轨迹和两路断流行为。启动与消息输出不等于精度验收。
当前只完成软件接入；新安装外参尚未提供，不能报告实机双雷达效果已通过。

已知时间边界：合并器对早于 timebase 的点偏移置零，避免负差下溢，
但这不等于底层原始时间戳已修复；后续仍需验证逐点时间跨度、回退和帧间连续性。
退出时若出现驱动 exit -7，保留日志并确认无进程残留；该生命周期问题未由此接入修复。
