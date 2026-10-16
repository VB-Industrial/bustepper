#!/usr/bin/env bash
# Run as root on Raspberry Pi after installing /opt/yakut/venv and DSDL sources.
set -euo pipefail
[[ $EUID == 0 ]] || { echo 'Run with sudo on Raspberry Pi' >&2; exit 1; }
base=/opt/yakut
"$base/venv/bin/yakut" compile \
    "$base/public_regulated_data_types/uavcan" \
    "$base/public_regulated_data_types/reg" --output "$base/dsdl"
"$base/venv/bin/python" - <<'PY'
import pathlib, site
pathlib.Path(site.getsitepackages()[0], 'bustepper_dsdl.pth').write_text('/opt/yakut/dsdl\n')
PY
cat > /usr/local/bin/yakut <<'WRAPPER'
#!/bin/sh
export YAKUT_PATH="${YAKUT_PATH:-/opt/yakut/dsdl}"
export UAVCAN__NODE__ID="${UAVCAN__NODE__ID:-100}"
export UAVCAN__CAN__IFACE="${UAVCAN__CAN__IFACE:-socketcan:can0}"
export UAVCAN__CAN__MTU="${UAVCAN__CAN__MTU:-64}"
exec /opt/yakut/venv/bin/yakut "$@"
WRAPPER
chmod 755 /usr/local/bin/yakut
ln -sfn yakut /usr/local/bin/y
cat > /etc/profile.d/yakut.sh <<'PROFILE'
export YAKUT_PATH="${YAKUT_PATH:-/opt/yakut/dsdl}"
export UAVCAN__NODE__ID="${UAVCAN__NODE__ID:-100}"
export UAVCAN__CAN__IFACE="${UAVCAN__CAN__IFACE:-socketcan:can0}"
export UAVCAN__CAN__MTU="${UAVCAN__CAN__MTU:-64}"
PROFILE
chmod 644 /etc/profile.d/yakut.sh
# The previous user installation otherwise masks the shared launcher in login shells.
for executable in /home/pi/.local/bin/y /home/pi/.local/bin/yakut; do
    if [[ -f "$executable" && ! -L "$executable" ]]; then
        mv "$executable" "$executable.before-system-install"
        ln -s /usr/local/bin/yakut "$executable"
    fi
done
chown -R root:root "$base"
chmod -R a+rX "$base"
git -C "$base/public_regulated_data_types" rev-parse HEAD > "$base/DSDL_COMMIT"
echo 'System Yakut and DSDL installed'
