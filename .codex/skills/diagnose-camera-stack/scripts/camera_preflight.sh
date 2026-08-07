#!/usr/bin/env bash
set -u

config=${1:-configs/standard3.yaml}
root=$(pwd)
failures=0

report() { printf '[camera-preflight] %s\n' "$*"; }
check() {
  if "$@"; then return 0; fi
  failures=$((failures + 1))
  return 1
}

report "root=$root config=$config"
check test -f "$config" || report "missing config: $config"
if test -f "$config"; then
  for key in camera_name vid_pid exposure_ms; do
    check rg -q "^${key}:" "$config" || report "missing config key: $key"
  done
  camera_name=$(awk '/^camera_name:/{sub(/^[^:]+:[[:space:]]*/, ""); gsub(/[ "'"'"'\r]/, ""); print}' "$config")
  vid_pid=$(awk '/^vid_pid:/{sub(/^[^:]+:[[:space:]]*/, ""); gsub(/[ "'"'"'\r]/, ""); print}' "$config")
  report "camera_name=${camera_name:-<unset>} vid_pid=${vid_pid:-<unset>}"
  case "$camera_name" in
    hikrobot) check rg -q '^gain:' "$config" || report "HikRobot config lacks gain" ;;
    mindvision) check rg -q '^gamma:' "$config" || report "MindVision config lacks gamma" ;;
    *) report "unknown camera_name: ${camera_name:-<unset>}"; failures=$((failures + 1));;
  esac
fi

if command -v lsusb >/dev/null 2>&1; then
  if test -n "${vid_pid:-}"; then
    if ! lsusb | rg -qi "$vid_pid"; then
      report "configured USB VID:PID not found: $vid_pid"
      failures=$((failures + 1))
    fi
  else
    report "skipping USB lookup because VID:PID is unset"
  fi
else
  report "lsusb unavailable; USB presence not checked"
fi

if test -x build-ninja/camera_test; then
  report "camera_test is available"
else
  report "build-ninja/camera_test is missing; build it before hardware testing"
  failures=$((failures + 1))
fi

if test "${camera_name:-}" = hikrobot; then
  test -f /opt/MVS/lib/64/libMvCameraControl.so && report "HikRobot MVS runtime found" || report "HikRobot MVS runtime not found at /opt/MVS/lib/64"
fi

if test "$failures" -eq 0; then report "PASS: environment checks found no blocking issue"; else report "FAIL: $failures blocking check(s); no device was opened"; fi
exit "$([ "$failures" -eq 0 ] && printf 0 || printf 1)"
