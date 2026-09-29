#!/usr/bin/env bash
# 编译安装 Livox-SDK2（livox_ros_driver2 依赖），默认装到 /usr/local（需要 sudo）。
#   scripts/install_livox_sdk2.sh            # 安装 v1.3.1 到 /usr/local
#   PREFIX=$HOME/.local scripts/install_livox_sdk2.sh   # 不用 sudo，装到用户目录（build.sh 会自动识别）
set -euo pipefail
VERSION="${VERSION:-v1.3.1}"
PREFIX="${PREFIX:-/usr/local}"
SRC_DIR="${SRC_DIR:-$HOME/.cache/Livox-SDK2}"

if [[ ! -d "$SRC_DIR/.git" ]]; then
  git clone https://github.com/Livox-SDK/Livox-SDK2.git "$SRC_DIR"
fi
git -C "$SRC_DIR" fetch --tags
git -C "$SRC_DIR" checkout "$VERSION"

cmake -S "$SRC_DIR" -B "$SRC_DIR/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX"
cmake --build "$SRC_DIR/build" -j"$(nproc)"
if [[ -w "$PREFIX" ]]; then
  cmake --install "$SRC_DIR/build"
else
  sudo cmake --install "$SRC_DIR/build"
  sudo ldconfig
fi
echo "Livox-SDK2 $VERSION 已安装到 $PREFIX"
