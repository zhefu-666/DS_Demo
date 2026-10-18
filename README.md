# DS_Demo：RoboMaster 哨兵手持感知平台工作空间

对应 [RoboMaster哨兵手持感知平台测试方案.md](RoboMaster哨兵手持感知平台测试方案.md)，覆盖 6 个阶段：
单雷达 Point-LIO → 双雷达 Point-LIO → ROGMap 隧道感知 → HIPNUC 替换主 IMU → D435 + OpenVINS 对照 → small_gicp 出口重定位。

环境：Ubuntu 22.04 + ROS 2 Humble，目标机 Intel NUC。

## 目录结构

```
DS_Demo/
├── scripts/                     环境安装、编译、录包、回放、评估脚本
├── data/{bags,maps,results}/    原始包 / 先验地图 / 评估结果（不进 git）
└── src/
    ├── drivers/livox_ros_driver2        Mid-360 驱动（含驱动内双雷达合并）
    ├── drivers/ds_nav_imu               HIPNUC 串口 IMU（/imu/data，m/s²、rad/s）
    ├── lio/point_lio                    首选定位基线（可 component 加载）
    ├── lio/fast_lio                     FAST-LIO2 对照
    ├── lio/small_point_lio              轻量候选
    ├── mapping/rog_map                  ROGMap
    ├── relocalization/icp_relocalization  small_gicp 重定位（gicp_node、pcd_sanitize_tool）
    ├── calibration/dual_lidar_calibration 双雷达外参标定
    ├── third_party/open_vins            OpenVINS
    └── handheld/handheld_bringup        本项目的启动、配置与工具（下面主要改这里）
        ├── launch/   lidar_pointlio（核心）、driver、s1~s6、replay_lio、s2_calib_record
        ├── config/   driver / point_lio / fast_lio / small_point_lio / rog_map / gicp /
        │             hipnuc / realsense / openvins / calibration / record（录包话题清单）
        ├── rviz/     handheld / lio_compare / tunnel / vio / reloc（各阶段显示配置）
        ├── src/      rog_map_node、tunnel_status_node
        └── scripts/  bag_to_tum.py、resource_monitor.py、tunnel_event_marker.py
```

## 一、环境配置（新机器 / NUC 一次性）

以下带 sudo 的步骤会提示输入密码。

```bash
git clone <仓库地址> ~/DS_Demo && cd ~/DS_Demo
scripts/install_deps.sh          # apt 依赖 + rosdep + evo（pip --user）+ dialout 组
scripts/install_livox_sdk2.sh    # Livox-SDK2 v1.3.1 -> /usr/local
scripts/build.sh                 # 重包串行编译，其余并行
echo "source ~/DS_Demo/install/setup.bash" >> ~/.bashrc
```

- `install_deps.sh` 会装 `python3-serial`、`linuxptp`、`ethtool`、`ros-humble-rosbag2-storage-mcap`、`ros-humble-realsense2-camera` 等；装完 **重新登录** 一次让 dialout 组生效。
- 不想用 sudo 装 SDK：`PREFIX=$HOME/.local scripts/install_livox_sdk2.sh`，`build.sh` 会自动找到，但运行前需 `export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH`。
- NUC 内存紧张时 `JOBS=2 scripts/build.sh`。只改了本包：`scripts/build.sh handheld_bringup`（`--symlink-install`，改 yaml/launch 不用重编）。
- evo 装在 `~/.local/bin`，不在 PATH 时：`echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc`。

### 1.1 雷达网络

默认 IP（与实物不一致时要改）：

| 设备 | IP | 配置位置 |
| --- | --- | --- |
| NUC 雷达网口 | 192.168.1.47 | `config/driver/*.json` 的 `host_net_info`，`scripts/setup_network.sh`（`HOST_IP=`） |
| 前雷达（主，IMU 来源） | 192.168.1.135 | json `lidar_configs`、`dual_merged.yaml` 的 `merge_front_ip`、`point_lio/mid360_dual.yaml` 的 imu 话题、`config/record/*.txt`、`calibration/dual_mid360.yaml` |
| 后雷达 | 192.168.1.122 | 同上（`merge_back_ip`） |
| 单雷达阶段 | 192.168.1.135 | `config/driver/single_mid360.json` |

IP 同时出现在话题名里（`livox/imu_192_168_1_135`），改 IP 后用 `grep -rn 192_168_1_ src/handheld` 全部替换。

```bash
ip -br link                          # 找到接交换机的网卡，例如 enp89s0
scripts/setup_network.sh enp89s0     # NetworkManager 静态 IP，开机自动生效
ping -c 3 192.168.1.135 && ping -c 3 192.168.1.122
```

### 1.2 时间同步（PTP）

两台雷达必须同源授时，否则驱动合并会因 `|Δbase_time| > merge_max_interval_ms`（5 ms）持续丢帧。
方案：NUC 作 PTP 主钟（交换机为透明时钟/边界时钟，按交换机手册配置）。

```bash
scripts/setup_ptp.sh enp89s0                 # 安装 systemd 服务 ptp4l-livox（硬件时间戳时附带 phc2sys-livox）
journalctl -u ptp4l-livox -f                 # 看到 assuming the grand master role 即正常
scripts/setup_ptp.sh enp89s0 --foreground    # 调试：前台运行
```

验证：启动 `ros2 launch handheld_bringup driver.launch.py driver:=dual_split`，
`ros2 topic echo --field header.stamp /livox/imu_192_168_1_135` 与 `_122` 的时间应与 `date +%s` 接近且两路一致。
**D435 与 HIPNUC 不走 PTP**，只能用主机接收时间，阶段 5 要单独标定时间偏移（见 5.5）。

### 1.3 HIPNUC 串口

```bash
ls -l /dev/serial/by-id/            # 找到 HIPNUC，记下 idVendor/idProduct：udevadm info -a -n /dev/ttyUSB0 | grep -m2 'idVendor\|idProduct'
sudo tee /etc/udev/rules.d/99-hipnuc.rules <<'R'
SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="yyyy", SYMLINK+="hipnuc", MODE="0660", GROUP="dialout"
R
sudo udevadm control --reload && sudo udevadm trigger
```

之后启动时传 `serial_port:=/dev/hipnuc`（或直接改 `config/hipnuc/hipnuc.yaml`）。波特率默认 115200。

### 1.4 D435

`ros-humble-realsense2-camera` 由 `install_deps.sh` 安装。`rs-enumerate-devices` 能看到相机即可；
USB 必须是 3.x（`rs-enumerate-devices | grep Usb` 显示 3.2），2.0 下 848×480@30 双红外会掉帧。

## 二、通用约定

- **核心 launch**：`lidar_pointlio.launch.py` 把驱动与 Point-LIO 放进同一个 `component_container_mt`（进程内零拷贝），
  各阶段都 include 它。`driver:=none` 表示回放 bag 不启驱动。
- **坐标系**：Point-LIO 世界系 `camera_init`，雷达系 `body`；另发布 `camera_init -> base_link`，base_link = 前雷达前方 **0.20 m**（只随 yaw 旋转，
  变形哨机体中心）。改这个偏移时三处必须一起改：
  `point_lio/*.yaml` 的 `preprocess.blind_center`、`rog_map/rog_map.yaml` 的 `map_sliding.center_offset`、
  `src/lio/point_lio/src/laserMapping.cpp` 的 `offset_vec`。
- **Point-LIO 输出**：`/aft_mapped_to_init`（Odometry，评估用）、`/path`、`/cloud_registered`、`/cloud_registered_full`（ROGMap/GICP 输入）。
- **原始数据先行**：每次实验先录原始包，算法对比全部离线回放同一份包。`scripts/record_stage.sh <清单> <名称>` 录包，
  话题清单在 `config/record/`，同时写 `<名称>.meta.txt`（git 版本、时间、`NOTE=` 备注）。原始包不会覆盖同名。
- **回放**：`scripts/play_raw.sh <bag>` 只放原始传感器话题（排除录包里的 `/tf`、`/aft_mapped_to_init` 等旧输出），自带 `--clock`；
  对应 launch 要加 `use_sim_time:=true`（`replay_lio.launch.py` 默认已开）。
- **一键回放评估**：`scripts/replay_eval.sh <bag> <algo> <sensor>` = 启动算法 + 回放 + 录输出 + 转 TUM + 记录负载，
  结果在 `data/results/<bag名>/`。

### 可视化（RViz）

所有阶段 launch 加 `rviz:=true` 即打开对应配置（在 `rviz/` 下，未运行的话题显示 No messages 属正常，
左侧面板勾选即可打开默认关闭的图层）：

| launch | RViz 配置 | 固定帧 | 默认显示 |
|---|---|---|---|
| s1 / s2 / s4 | `handheld.rviz` | `camera_init` | 当前帧点云 `/cloud_registered`（保留 10 s 形成局部地图）、位姿、轨迹、TF；可勾选 `/Laser_map`（需 `save_pcd:=true`） |
| `replay_lio` | `lio_compare.rviz` | 按算法自动：`camera_init` / `fastlio_odom` / `odom` | Point-LIO、FAST-LIO、small_point_lio 三组点云+位姿+轨迹 |
| s3 | `tunnel.rviz` | `camera_init` | ROGMap 投影层 `/rog_map/layer_type`（灰度：白 UNKNOWN、浅灰 FREE、深灰 PASSABLE、黑 OCCUPIED）、占据体素、局部地图边界、ROI 框（绿 PASSABLE / 红 BLOCKED / 黄 UNKNOWN，框角文字为稳定状态） |
| s5 | `vio.rviz` | `global` | OpenVINS 位姿、轨迹、MSCKF/SLAM 特征点、**特征跟踪图 `trackhist`**；可勾选左右红外原图 |
| s6 | `reloc.rviz` | `map` | 先验地图（灰）、对齐后的当前帧（红），收敛后二者应重合；可勾选参与配准的局部地图 |

- **回放时**：launch 带 `use_sim_time:=true`（replay_lio 默认开），RViz 会跟随 `/clock`；若 bag 暂停导致点云"消失"，把 Decay Time 调大即可。
- **图像**：也可单独看 `ros2 run rqt_image_view rqt_image_view /ov_msckf/trackhist`。
- **原始雷达点云**：驱动输出的是 Livox CustomMsg（`xfer_format: 1`，Point-LIO 需要逐点时间），RViz 不能直接显示，看 `/cloud_registered` 即可。
- **阶段 5 为什么看不到雷达点云**：OpenVINS 的 `global` 与雷达的 `camera_init` 之间没有 TF（方案要求两者独立、不做融合），
  需要对照时另开一个 RViz：`rviz2 -d install/handheld_bringup/share/handheld_bringup/rviz/handheld.rviz`。
- **阶段 6 收敛前**：`map -> camera_init` 的 TF 还不存在，只显示先验地图；收敛后当前帧与 LIO 点云才会出现在 `map` 下。
- **NUC 无显示器**：NUC 上不开 RViz（`rviz:=false`，也省 CPU），开发机与 NUC 同网段、设相同 `ROS_DOMAIN_ID` 后在开发机运行
  `rviz2 -d <配置>`。跨机订阅点云带宽较大，卡顿时降低 Decay Time 或只勾选必要图层；测负载时关闭远程 RViz。

## 三、评估方法（所有阶段通用）

方案要求：同一路线重复、用实测停靠点作参考、记录 NUC 负载，**不能只凭地图好看或回到起点判断精度**。

1. **路线与停靠点**：每个场景固定路线，沿途地面贴 3~5 个停靠点，用卷尺/激光测距在"起点坐标系"（起点处 x 朝前、y 朝左）下实测坐标。
   每次录包在每个点静止 ≥3 s，每条路线至少重复 3 次（`s2_loop_01/02/03`）。
2. **停靠点误差**：从轨迹中找静止区间，写成 `stops.csv`（`t_start,t_end,x,y`，时间取包内时间）：
   ```bash
   ros2 run handheld_bringup bag_to_tum.py data/bags/s2_loop_01 --topic /aft_mapped_to_init --output data/results/s2_loop_01/point_lio_live.tum
   scripts/eval_evo.sh stops data/results/s2_loop_01/point_lio_live.tum data/results/s2_loop_01/stops.csv
   ```
   输出每个停靠点的 XY 误差、平均与最大值。Point-LIO 起点即 `camera_init` 原点，起点朝向即 x 轴，与实测坐标系一致。
3. **算法互比**：`scripts/eval_evo.sh pair <参考.tum> <待测.tum>`，输出 SE(3) 对齐后的 APE 与 1 m 段 RPE（`ape.txt`、`rpe_1m.txt`）。
   互比只说明"两者差多少"，精度结论以停靠点为准。
4. **负载**：`resource_monitor.py` 记录进程 CPU%（单核 100% 计）、RSS、整机 CPU/内存/温度。实时跑时另开终端：
   ```bash
   ros2 run handheld_bringup resource_monitor.py --output data/results/s2_loop_01/load_live.csv \
       --match component_container rog_map_node gicp_node run_subscribe_msckf realsense2_camera hipnuc_imu_node
   ```
   `replay_eval.sh` 会自动记录。注意回放负载 ≠ 实时负载（回放无驱动开销），上车判断以实时为准。

## 四、分阶段使用

以下命令都在工作空间根目录执行，且已 `source install/setup.bash`。每个 launch 的完整参数：`ros2 launch handheld_bringup <文件> --show-args`。

### 阶段 1：单 Mid-360 + Point-LIO，FAST-LIO2 / small_point_lio 对照

只接前雷达（192.168.1.135），驱动配置 `config/driver/single_mid360.{yaml,json}`。

```bash
# 终端 1：实时
ros2 launch handheld_bringup s1_single_pointlio.launch.py rviz:=true
# 终端 2：录原始包
NOTE="房间 路线A 第1次" scripts/record_stage.sh s1_single s1_room_01
# 离线：同一份包三种算法
scripts/replay_eval.sh data/bags/s1_room_01 point_lio single
scripts/replay_eval.sh data/bags/s1_room_01 fast_lio single
scripts/replay_eval.sh data/bags/s1_room_01 small_point_lio single
scripts/eval_evo.sh pair data/results/s1_room_01/point_lio_single.tum data/results/s1_room_01/fast_lio_single.tum
```

- FAST-LIO 输出重映射到 `/fast_lio/*`，TF 用 `fastlio_odom/fastlio_body`，不与 Point-LIO 冲突。
- small_point_lio 输出重映射到 `/small_point_lio/*`，它需要 `base_link -> livox_frame` 静态 TF（launch 已发布单位变换，
  所以它的 odometry 就是雷达位姿，与 Point-LIO 的 `/aft_mapped_to_init` 同为雷达系，可直接比）。
- 方案建议先跑通 Point-LIO 基线，small_point_lio 不必一开始就调。
- 场景：房间建图、直线往返、原地转向、走廊穿越，各录一包。

### 阶段 2：双 Mid-360

**2.1 外参标定（先做）**

```bash
ros2 launch handheld_bringup s2_calib_record.launch.py          # 驱动分路发布，不合并
NOTE="静置30s" scripts/record_stage.sh s2_split calib_static -d 30   # 静置包（估计 IMU 零偏）
scripts/record_stage.sh s2_split calib_motion                   # 慢速绕 8 字、多轴旋转，1~2 分钟，环境要有丰富结构
ros2 run dual_lidar_calibration calibrate --bag data/bags/calib_motion --static-bag data/bags/calib_static \
    --config install/handheld_bringup/share/handheld_bringup/config/calibration/dual_mid360.yaml \
    --output data/results/calib_$(date +%Y%m%d)
```

退出码 0 = 通过，2 = 质量不达标（重录），1 = 错误。通过后把结果里的 `merge_extrinsic_back_to_front`
抄到 `config/driver/dual_merged.yaml` 与 `dual_split.yaml`（两处保持一致），并同步 `calibration/dual_mid360.yaml` 的 `initial_extrinsic`。
外参格式 `[x, y, z, roll, pitch, yaw]`（m/rad），含义 `p_front = R·p_back + t`，`R = Rz·Ry·Rx`。

**2.2 双雷达实时 + 对照**

```bash
ros2 launch handheld_bringup s2_dual_pointlio.launch.py rviz:=true      # mode:=merged（默认）
scripts/record_stage.sh s2_dual s2_corridor_01
```

仅前 / 仅后 / 双雷达对照要用**同一份数据**：录分路包，离线分别跑：

```bash
ros2 launch handheld_bringup s2_calib_record.launch.py        # 分路驱动
scripts/record_stage.sh s2_split s2_corridor_split_01
scripts/replay_eval.sh data/bags/s2_corridor_split_01 point_lio front
scripts/replay_eval.sh data/bags/s2_corridor_split_01 point_lio back
```

分路包里没有合并点云，"双雷达"那一路需要把驱动的合并逻辑离线跑一遍，工作空间暂不提供离线合并工具；
因此建议同一路线分别录 `s2_dual`（合并）与 `s2_split`（分路）各 3 次，用停靠点误差比较，而不是逐帧比较。
实时也可 `mode:=front_only / back_only` 直接看效果。`back_only` 时 blind 球心会自动按外参换算到后雷达坐标系；
后雷达自身坐标系下的轨迹是后雷达位姿，评估前需按外参换算或只比较停靠点间相对距离。

**2.3 遮挡与断流**

- 遮挡：手/纸板挡住一台雷达部分视场，记录 `NOTE=` 并对比停靠点误差。
- 断流：运行中拔掉一台雷达网线。**驱动合并模式下任一雷达断流，合并输出会停止**（日志 `waiting for front/back lidar data`），
  Point-LIO 没有点云输入，只靠 IMU 递推会快速发散——这是当前合并实现的已知行为，也是本测试要记录的结论
  （断流到输出停止的时间、恢复插回后能否继续）。整车上若需要单雷达降级，要改驱动合并逻辑。
- 驱动日志里频繁出现丢帧，通常是 PTP 没同步或 `merge_max_interval_ms` 太小。

### 阶段 3：ROGMap 隧道感知

```bash
ros2 launch handheld_bringup s3_tunnel.launch.py csv_path:=$PWD/data/results/s3_tunnel_01.csv rviz:=true
scripts/record_stage.sh s3_tunnel s3_tunnel_01
ros2 run handheld_bringup tunnel_event_marker.py        # 放入箱体按 p，移走按 r
```

- `tunnel_status_node` 统计 ROI 内 `/rog_map/layer_type` 栅格：占用比 ≥ `blocked_occupied_ratio`(0.15) → BLOCKED，
  未知比 ≥ `unknown_ratio`(0.5) → UNKNOWN，否则 PASSABLE；连续 `debounce_count` 帧一致才切换。输出 `/tunnel_status`、`/tunnel_status/detail`、RViz `/tunnel_status/roi`。
- ROI 在 `config/rog_map/tunnel_status.yaml`：`fixed` 模式是 `camera_init` 下固定矩形——把设备放在起点，量出洞口相对起点的位置填进去；
  `body_relative`（`roi_mode:=body_relative`）是机体前方矩形，适合推进测试。RViz 勾选 `rog_map_layer_type` 和 `tunnel_roi` 可以直接对照。
- CSV 里事件行记录按键时刻，状态切换行的 `latency_s` 就是"放入/移走箱体 → 判定切换"的延迟。
- 回放复现：`ros2 launch handheld_bringup s3_tunnel.launch.py driver:=none use_sim_time:=true`，再 `scripts/play_raw.sh data/bags/s3_tunnel_01`（`/tunnel_event` 也会回放，延迟可复算）。
- 要调的 ROGMap 参数：`resolution`、`raycasting`（未知/空闲更新快慢）、`decay`、`map_size`；近墙盲区与 Point-LIO 的 `blind` 相关。

### 阶段 4：HIPNUC 替换主 IMU

上车前必须确认三件事（缺一项结果都不可信）：

1. **单位**：`ds_nav_imu` 输出 m/s²、rad/s，所以 `point_lio/mid360_hipnuc.yaml` 用 `acc_norm: 9.81`，`satu_acc`/`satu_gyro` 按 m/s²/rad/s 填（Mid-360 内置 IMU 是 g，`acc_norm: 1.0`）。
   检查：静止时 `ros2 topic echo /imu/data --field linear_acceleration` 模长 ≈ 9.8。
2. **外参**：`mapping.extrinsic_T/R`（前雷达在 HIPNUC 坐标系下的位姿）当前是 TODO 单位阵，按支架实测填写；
   也可先设 `extrinsic_est_en: True` 跑几包看收敛值，再固定。
3. **时间**：HIPNUC 用主机接收时间，带串口延迟。用静置 + 快速晃动的包，对比 `/imu/data` 与 `/livox/imu_192_168_1_135` 的角速度曲线互相关求偏移，
   填到 `config/hipnuc/hipnuc.yaml` 的 `timestamp_offset_sec`。

```bash
ros2 launch handheld_bringup s4_hipnuc_pointlio.launch.py serial_port:=/dev/hipnuc
scripts/record_stage.sh s4_hipnuc s4_corridor_01
# 同一份包：内置 IMU vs HIPNUC
scripts/replay_eval.sh data/bags/s4_corridor_01 point_lio dual
scripts/replay_eval.sh data/bags/s4_corridor_01 point_lio hipnuc
```

判断标准："可重复的改善"——多次重复路线的停靠点误差都更小，而不是某一次更好。

### 阶段 5：D435 红外双目 + HIPNUC + OpenVINS

**5.1 采集**（推荐：同时录视觉和雷达，OpenVINS 离线跑，雷达轨迹作参考）

```bash
ros2 launch handheld_bringup s5_d435_openvins.launch.py openvins:=false lidar:=true serial_port:=/dev/hipnuc
scripts/record_stage.sh s5_vio s5_corridor_01
```

D435 只开左右红外（848×480@30，全局快门），关闭彩色、深度和**结构光发射器**（`config/realsense/d435_ir.yaml`，投射点会破坏特征跟踪）。
话题：`/camera/camera/infra1/image_rect_raw`、`/camera/camera/infra2/image_rect_raw`。

**5.2 回放 OpenVINS**

```bash
ros2 launch handheld_bringup s5_d435_openvins.launch.py camera:=false imu:=false use_sim_time:=true
scripts/play_raw.sh data/bags/s5_corridor_01
```

实时跑：`ros2 launch handheld_bringup s5_d435_openvins.launch.py`（看负载用）。

**5.3 对照**：`/ov_msckf/poseimu`（`global` 系）与 `/aft_mapped_to_init` 各自导出 TUM，
`scripts/eval_evo.sh pair <雷达.tum> <openvins.tum>`（SE(3) 对齐）。**两者独立评估，不做位姿平均。**
重点看：有纹理走廊能否补充纵向运动；关灯/遮挡时是否失效、失效后是否发散。

**5.4 标定（OpenVINS 结果可信的前提）**，全部在 `config/openvins/d435_hipnuc/`：

- `kalibr_imu_chain.yaml`：HIPNUC 噪声参数目前是占位值，用静置 2 h 以上的包做 Allan 方差（如 allan_variance_ros）后填写。
- `kalibr_imucam_chain.yaml`：相机内参（可先用 `ros2 topic echo /camera/camera/infra1/camera_info` 的出厂值）、
  `T_cam_imu`（HIPNUC→红外相机，当前是按安装方向写的占位）、`timeshift_cam_imu`。用 Kalibr 的 camera-imu 标定得到。
- 标定好之前可在 `estimator_config.yaml` 打开 `calib_cam_extrinsics` / `calib_cam_timeoffset` 在线估计，但不能代替离线标定。

**5.5 时间关系**：D435 与 HIPNUC 都是主机时间，互相之间的偏移由 Kalibr `timeshift_cam_imu` 给出；
与雷达（PTP → 系统时间）的对照只在轨迹层面做，evo `--t_max_diff 0.02` 容忍 20 ms。

### 阶段 6：出口重定位（small_gicp）

**6.1 建先验地图**

```bash
ros2 launch handheld_bringup s2_dual_pointlio.launch.py save_pcd:=true   # 慢速走完整个场地，回到起点后 Ctrl-C
scripts/save_map.sh field_20260930                                     # -> data/maps/field_20260930.pcd（pcd_sanitize_tool 清理）
```

也可回放建图：`ros2 launch handheld_bringup replay_lio.launch.py sensor:=dual save_pcd:=true` + `scripts/play_raw.sh <bag>`。
Point-LIO 在退出时写 `src/lio/point_lio/PCD/scans.pcd`，必须正常 Ctrl-C 退出。地图坐标系 = 建图时的 `camera_init`（建图起点）。

**6.2 重定位测试**

```bash
ros2 launch handheld_bringup s6_relocalization.launch.py map_file:=data/maps/field_20260930.pcd \
    init_x:=0.0 init_y:=0.0 init_yaw:=0.0 results_csv:=$PWD/data/results/s6_exit_01.csv
scripts/record_stage.sh s6_reloc s6_exit_01
ros2 service call /gicp_recall std_srvs/srv/Trigger      # 出洞后触发重新匹配
```

- 输出 TF `map -> camera_init`；`initial_guess` 模式在给定初值附近搜索，`mode:=multi_guess` 在 `relocalization.yaml` 的 `multi_guess.search_rectangles` 内全局搜索。
- 测法：在地图内已知停靠点（地图坐标实测）开始/结束，隧道内人为制造漂移，出洞后触发 `/gicp_recall`，记录
  收敛时间、`results_csv` 中的得分/内点率、校正后位姿与实测停靠点的偏差；再在相似结构处故意给错误初值，统计**错误匹配**次数。
- `timeout_seconds`(60) 内未收敛会回退 `default_pose_on_timeout`，测试中一律记为"重定位失败"。
- 回放：加 `driver:=none use_sim_time:=true`，再 `scripts/play_raw.sh <bag>`。

## 五、移植到 NUC / 上传 git

仓库只含源码与配置；`build/ install/ log/ data/*` 和 `*.pcd` 已在 `.gitignore` 中排除（原始包请另存到移动硬盘/NAS）。

```bash
# 开发机
cd ~/DS_Demo && git init && git add -A && git commit -m "handheld perception workspace"
git remote add origin <仓库地址> && git push -u origin main
# NUC
git clone <仓库地址> ~/DS_Demo && cd ~/DS_Demo
scripts/install_deps.sh && scripts/install_livox_sdk2.sh && scripts/build.sh
scripts/setup_network.sh <网卡> && scripts/setup_ptp.sh <网卡>
```

NUC 上要按实物核对的内容：

| 项 | 位置 |
| --- | --- |
| 雷达网卡名、主机/雷达 IP | `setup_network.sh` 参数、`config/driver/*.json`、话题名（见 1.1） |
| 双雷达外参 | `config/driver/dual_merged.yaml`、`dual_split.yaml` |
| 雷达→内置 IMU 外参 | `point_lio/mid360_*.yaml`、`fast_lio/mid360.yaml`、`small_point_lio/mid360.yaml`（Mid-360 出厂值 `[-0.011, -0.02329, 0.04412]`） |
| HIPNUC 串口、外参、时间偏移 | `hipnuc/hipnuc.yaml`、`point_lio/mid360_hipnuc.yaml` |
| 相机-IMU 标定与噪声 | `openvins/d435_hipnuc/*.yaml` |
| 洞口 ROI | `rog_map/tunnel_status.yaml` |
| base_link 偏移 0.20 m | 见"二、通用约定"三处同步 |

`src/` 下各第三方包是修改过的版本（Point-LIO component 化、驱动合并、blind 球心等），不要用上游覆盖。

## 六、常见问题

- **编译找不到 Livox SDK**：`scripts/install_livox_sdk2.sh`，或 `LIVOX_SDK_PREFIX=<前缀> scripts/build.sh`。
- **运行时 `liblivox_lidar_sdk_shared.so: cannot open`**：SDK 不在 /usr/local 时把 `<前缀>/lib` 加进 `LD_LIBRARY_PATH`；装到 /usr/local 后执行过 `sudo ldconfig`。
- **Point-LIO 没输出**：`ros2 topic hz /livox/lidar` 与 IMU 话题；合并模式看驱动日志是否在等某台雷达；回放时是否 `use_sim_time:=true` 且用 `play_raw.sh`（`--clock`）。
- **ROGMap 无 `/rog_map/layer_type`**：该话题只在有订阅者时发布（RViz 勾选或 tunnel_status_node 运行时）；回放时 use_sim_time 必须为 true。
- **HIPNUC 打不开串口**：`groups | grep dialout`（加组后需重新登录），`ls -l /dev/hipnuc`。
- **OpenVINS 不初始化**：先静止 1~2 s 再动；检查 `/imu/data` 频率（`update_rate`）与两路红外时间戳是否同步（`enable_sync`）。
- **gicp 超时**：地图坐标系是否正确（建图起点）、`initial_pose` 是否接近真值、地图是否经过 `pcd_sanitize_tool`，`target_pcd_file` 必须是绝对路径（launch 已自动转换）。
- **录包 mcap 不可用**：未装 `ros-humble-rosbag2-storage-mcap` 时自动退回 sqlite3，功能相同。

### 独立双雷达软时间同步实验

新增 [dual_lidar_soft_sync](src/experiments/dual_lidar_soft_sync/README.md)：
使用 ApproximateTime 对分路 CustomMsg 做限差配对，支持模拟源、真实分路话题/rosbag、CSV 统计和三场景测试。
输出隔离在 `/sync_compare/soft`，不改现有阶段 2 驱动/LIO 链路；硬同步实验预留独立命名空间和对比口径。

双雷达原始时间戳与帧间隔检查见 [tool/README.md](tool/README.md)：
分别统计帧间隔、时间差分位数及双向配对覆盖率，可独立用于软/硬同步实验。
