#!/usr/bin/env bash
# Administrative deployment only. The running UI has no SSH/WSL dependency.
set -euo pipefail
cd "$(dirname "$0")/.."
pi_host=${BUSTEPPER_PI:-pi@192.168.1.104}
auth=()
if [[ -n ${SSHPASS:-} ]]; then auth=(sshpass -e); fi
ssh_options=(-o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new)
"${auth[@]}" ssh "${ssh_options[@]}" "$pi_host" 'mkdir -p /home/pi/bustepper/ui'
"${auth[@]}" scp "${ssh_options[@]}" server/can_bridge.py server/bustepper-can.service server/requirements.txt "$pi_host:/home/pi/bustepper/"
"${auth[@]}" scp "${ssh_options[@]}" ui/index.html ui/style.css ui/app.js "$pi_host:/home/pi/bustepper/ui/"
"${auth[@]}" ssh "${ssh_options[@]}" "$pi_host" \
    'sudo -n install -m 644 /home/pi/bustepper/bustepper-can.service /etc/systemd/system/bustepper-can.service && sudo -n systemctl daemon-reload && sudo -n systemctl enable bustepper-can.service && sudo -n systemctl restart bustepper-can.service && systemctl is-active bustepper-can.service'
echo 'UI runs directly on Raspberry Pi: http://192.168.1.104:8088'
