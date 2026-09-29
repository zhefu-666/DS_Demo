#!/usr/bin/env bash
# 编译整个工作空间。重包（驱动 / ROGMap / 重定位 / Point-LIO）先串行，避免 NUC 内存不足，其余并行。
#   scripts/build.sh                  # 全部
#   scripts/build.sh handheld_bringup # 只编指定包（及其依赖已编好时）
# 环境变量：
#   LIVOX_SDK_PREFIX  Livox-SDK2 安装前缀（默认依次查找 /usr/local、$HOME/.local）
#   JOBS              并行任务数（默认 nproc/2，至少 1）
set -euo pipefail
source "$(dirname "$0")/common.sh"
cd "$WS_DIR"
set +u; source /opt/ros/humble/setup.bash; set -u

JOBS="${JOBS:-$(( $(nproc) / 2 > 0 ? $(nproc) / 2 : 1 ))}"
CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DPython3_EXECUTABLE=/usr/bin/python3)

# 定位 Livox-SDK2
SDK_PREFIX="${LIVOX_SDK_PREFIX:-}"
if [[ -z "$SDK_PREFIX" ]]; then
  for p in /usr/local "$HOME/.local"; do
    if [[ -f "$p/lib/liblivox_lidar_sdk_shared.so" ]]; then SDK_PREFIX="$p"; break; fi
  done
fi
if [[ -z "$SDK_PREFIX" || ! -f "$SDK_PREFIX/lib/liblivox_lidar_sdk_shared.so" ]]; then
  echo "[error] 未找到 Livox-SDK2（liblivox_lidar_sdk_shared.so），先运行 scripts/install_livox_sdk2.sh" >&2
  exit 1
fi
if [[ "$SDK_PREFIX" != "/usr/local" && "$SDK_PREFIX" != "/usr" ]]; then
  CMAKE_ARGS+=(-DLIVOX_LIDAR_SDK_LIBRARY="$SDK_PREFIX/lib/liblivox_lidar_sdk_shared.so"
               -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$SDK_PREFIX/include")
  export LD_LIBRARY_PATH="$SDK_PREFIX/lib:${LD_LIBRARY_PATH:-}"
  echo "[info] 使用 Livox-SDK2: $SDK_PREFIX（运行时需要 LD_LIBRARY_PATH 包含 $SDK_PREFIX/lib）"
fi

COMMON=(--symlink-install --cmake-args "${CMAKE_ARGS[@]}")

if [[ $# -gt 0 ]]; then
  colcon build "${COMMON[@]}" --parallel-workers "$JOBS" --packages-select "$@"
  exit 0
fi

HEAVY=(livox_ros_driver2 rog_map icp_relocalization point_lio)
for pkg in "${HEAVY[@]}"; do
  MAKEFLAGS="-j${JOBS}" colcon build "${COMMON[@]}" --parallel-workers 1 --packages-up-to "$pkg"
done
MAKEFLAGS="-j2" colcon build "${COMMON[@]}" --parallel-workers "$JOBS"
echo
echo "编译完成：source $WS_DIR/install/setup.bash"
