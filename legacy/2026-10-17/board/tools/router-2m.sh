#!/bin/bash
set -euo pipefail
# Run with sudo on Uno Q. Preserve the board-generated GPIO boot/reset hooks.
target=/etc/systemd/system/arduino-router.service.d/90-xgoduck-baud.conf
if [ -n "${SUDO_USER:-}" ]; then
  user_home=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
  user_home="$HOME"
fi
backup_dir="${user_home:-/tmp}/xgoduck-backup"
mkdir -p "$backup_dir" /etc/systemd/system/arduino-router.service.d
if [ -f "$target" ]; then
  cp -p "$target" "$backup_dir/router-override-$(date +%Y%m%d-%H%M%S).conf"
fi
systemctl cat arduino-router > "$backup_dir/router-before-$(date +%Y%m%d-%H%M%S).txt"
ready_hook=""
if /usr/bin/arduino-router --help 2>&1 | grep -q -- '--after-ready'; then
  ready_hook=" --after-ready '/usr/bin/gpioset -c /dev/gpiochip1 -t0 70=1'"
elif ! systemctl cat arduino-router | grep -q '^ExecStartPost=.*70=1'; then
  echo 'Missing stock GPIO ready hook; refusing router override' >&2
  exit 1
fi
cat > "$target" <<EOF
[Service]
ExecStart=
ExecStart=/usr/bin/arduino-router --unix-port /var/run/arduino-router.sock --serial-port /dev/ttyHS1 --serial-baudrate 2000000${ready_hook}
EOF
systemctl daemon-reload
systemctl restart arduino-router
for attempt in 1 2 3 4 5; do
  if systemctl is-active --quiet arduino-router && [ -S /var/run/arduino-router.sock ]; then break; fi
  sleep 1
done
systemctl is-active --quiet arduino-router
test -S /var/run/arduino-router.sock
systemctl show arduino-router -p ExecStart
