#!/usr/bin/env bash
# 雷达网口静态 IP（NetworkManager 持久化，需要 sudo）。
#   scripts/setup_network.sh <网卡名>        例：scripts/setup_network.sh enp89s0
# 网卡名用 `ip -br link` 查看。主机 IP 必须与 config/driver/*.json 中 host_net_info 一致（默认 192.168.1.47）。
set -euo pipefail
IFACE="${1:?用法: $0 <网卡名>  （ip -br link 查看）}"
HOST_IP="${HOST_IP:-192.168.1.47}"
CON="livox-${IFACE}"

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "网卡 $IFACE 不存在" >&2; exit 1
fi
if nmcli -t -f NAME con show | grep -qx "$CON"; then
  sudo nmcli con delete "$CON"
fi
sudo nmcli con add type ethernet ifname "$IFACE" con-name "$CON" \
  ipv4.method manual ipv4.addresses "${HOST_IP}/24" ipv4.never-default yes ipv6.method ignore \
  connection.autoconnect yes
sudo nmcli con up "$CON"
ip -br addr show "$IFACE"
echo "测试连通：ping -c 3 192.168.1.135 && ping -c 3 192.168.1.122（按实际雷达 IP）"
