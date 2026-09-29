#!/usr/bin/env bash
# 安装系统依赖（需要 sudo，会提示输入密码）。Ubuntu 22.04 + ROS 2 Humble 已安装的前提下运行。
#   scripts/install_deps.sh
set -euo pipefail
source "$(dirname "$0")/common.sh"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未检测到 ROS 2 Humble，请先按 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html 安装 ros-humble-desktop" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git python3-colcon-common-extensions python3-rosdep python3-pip \
  libeigen3-dev libpcl-dev libyaml-cpp-dev libgoogle-glog-dev libboost-all-dev libopencv-dev \
  python3-serial python3-numpy python3-yaml \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-tf2-eigen ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-rosbag2-storage-mcap \
  ros-humble-realsense2-camera ros-humble-realsense2-description \
  ethtool linuxptp chrony

# rosdep 补齐 package.xml 中声明但上面未覆盖的依赖
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
set +u; source /opt/ros/humble/setup.bash; set -u
rosdep install --from-paths "$WS_DIR/src" --ignore-src -r -y --rosdistro humble \
  --skip-keys "livox_sdk2 small_gicp"

# evo 轨迹评估（装到用户目录，不需要 sudo）
python3 -m pip install --user "evo==1.31.1"
if ! echo ":$PATH:" | grep -q ":$HOME/.local/bin:"; then
  echo "[提示] 把 ~/.local/bin 加入 PATH：echo 'export PATH=\$HOME/.local/bin:\$PATH' >> ~/.bashrc"
fi

# 串口权限（HIPNUC），重新登录后生效
if ! id -nG "$USER" | grep -qw dialout; then
  sudo usermod -aG dialout "$USER"
  echo "[提示] 已把 $USER 加入 dialout 组，重新登录后生效"
fi
echo "依赖安装完成。下一步：scripts/install_livox_sdk2.sh"
