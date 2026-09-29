#!/usr/bin/env bash
# 以 NUC 为 PTP 主时钟给两台 Mid-360 授时（linuxptp，需要 sudo）。
#   scripts/setup_ptp.sh <网卡名>            # 安装并启用 systemd 服务 ptp4l-livox
#   scripts/setup_ptp.sh <网卡名> --foreground   # 前台运行，Ctrl-C 结束（调试用）
# 网卡需支持硬件时间戳（ethtool -T <网卡> 看 hardware-transmit）；不支持时自动退回软件时间戳。
set -euo pipefail
IFACE="${1:?用法: $0 <网卡名> [--foreground]}"
MODE="${2:-}"

if ethtool -T "$IFACE" 2>/dev/null | grep -q "hardware-transmit"; then
  TS="hardware"
else
  TS="software"
fi
echo "网卡 $IFACE 使用 ${TS} 时间戳"

CONF=/etc/linuxptp/ptp4l-livox.conf
TMP=$(mktemp)
cat > "$TMP" <<CFG
[global]
# 作为主时钟：priority 取小值，禁止成为从时钟
priority1               127
priority2               127
clientOnly              0
serverOnly              1
time_stamping           ${TS}
network_transport       UDPv4
delay_mechanism         E2E
logSyncInterval         0
logAnnounceInterval     1
CFG

if [[ "$MODE" == "--foreground" ]]; then
  sudo ptp4l -i "$IFACE" -f "$TMP" -m
  exit 0
fi

sudo install -D -m 644 "$TMP" "$CONF"
rm -f "$TMP"
sudo tee /etc/systemd/system/ptp4l-livox.service >/dev/null <<UNIT
[Unit]
Description=PTP master for Livox Mid-360 on ${IFACE}
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/sbin/ptp4l -i ${IFACE} -f ${CONF}
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
UNIT
if [[ "$TS" == "hardware" ]]; then
  # 硬件时间戳时，phc2sys 让网卡 PHC 跟随系统时钟，保证雷达时间 = 系统时间
  sudo tee /etc/systemd/system/phc2sys-livox.service >/dev/null <<UNIT
[Unit]
Description=Sync ${IFACE} PHC from system clock
After=ptp4l-livox.service
Requires=ptp4l-livox.service

[Service]
ExecStart=/usr/sbin/phc2sys -s CLOCK_REALTIME -c ${IFACE} -O 0 -m
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
UNIT
fi
sudo systemctl daemon-reload
sudo systemctl enable --now ptp4l-livox.service
[[ "$TS" == "hardware" ]] && sudo systemctl enable --now phc2sys-livox.service
systemctl --no-pager status ptp4l-livox.service | head -n 12
echo "查看日志：journalctl -u ptp4l-livox -f"
