#!/bin/sh
set -eu
config_path=${1:-/home/rmul/Dust_infantry_vision/configs/standard3.yaml}
timeout_seconds=${CAMERA_WAIT_TIMEOUT_SECONDS:-60}
vid_pid=$(awk '/^[[:space:]]*vid_pid:/ {print $2; exit}' "$config_path" | tr -d "")
if [ -z "$vid_pid" ]; then
  echo "wait-camera: vid_pid is missing from $config_path" >&2
  exit 1
fi
case "$vid_pid" in
  ????\:????) ;;
  *) echo "wait-camera: invalid vid_pid: $vid_pid" >&2; exit 1 ;;
esac
i=0
while [ "$i" -lt "$timeout_seconds" ]; do
  if command -v lsusb >/dev/null 2>&1 && lsusb -d "$vid_pid" >/dev/null 2>&1; then
    exit 0
  fi
  i=$((i + 1))
  sleep 1
done
echo "wait-camera: USB camera $vid_pid was not found after ${timeout_seconds}s" >&2
exit 1
