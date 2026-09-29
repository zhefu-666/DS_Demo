# 各脚本共用：定位工作空间并 source 环境（被 source，不单独执行）
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WS_DIR
source_ws() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash"
  else
    echo "[warn] 未找到 $WS_DIR/install/setup.bash，请先运行 scripts/build.sh" >&2
  fi
  set -u
}
