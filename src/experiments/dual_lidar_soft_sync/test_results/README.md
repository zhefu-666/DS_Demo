# 本地实跑记录

2026-10-18，ROS 2 Humble，DS_Demo 本地工作区。

- `colcon build --packages-select dual_lidar_soft_sync`：通过。
- `ros2 run dual_lidar_soft_sync run_mock_tests.py /tmp/dual_lidar_soft_sync`：三场景断言全部通过。
- `ros2 launch dual_lidar_soft_sync soft_sync.launch.py duration_s:=2.0 csv_path:=/tmp/soft_launch_check.csv`：正常结束，149/149 接收、148 对配对。
- `git diff --check`：通过。

`mock_results.json` 保存当次参数和实测指标；其中 `/tmp` 路径是当次运行位置，非仓库依赖。
ApproximateTime 最后一个候选可能留在队列，因此有一帧未配对不等于物理丢包。
本次未连接真实雷达，未启动硬同步，也未验证 LIO 精度。
