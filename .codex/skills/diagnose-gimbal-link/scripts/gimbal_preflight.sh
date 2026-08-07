#!/usr/bin/env bash
set -u

config=${1:-configs/standard3.yaml}
failures=0
report() { printf '[gimbal-preflight] %s\n' "$*"; }

if test ! -f "$config"; then
  report "missing config: $config"
  exit 1
fi

if ! rg -q '^com_port:' "$config"; then
  report "missing config key: com_port"
  failures=$((failures + 1))
fi

port=$(awk '/^com_port:/{sub(/^[^:]+:[[:space:]]*/, ""); gsub(/[ "'"'"'\r]/, ""); print}' "$config")
report "configured port=${port:-<unset>}"
if test -z "$port"; then
  failures=$((failures + 1))
elif test ! -e "$port"; then
  report "serial device does not exist: $port"
  failures=$((failures + 1))
else
  test -r "$port" && report "read permission: yes" || { report "read permission: no"; failures=$((failures + 1)); }
  test -w "$port" && report "write permission: yes" || { report "write permission: no"; failures=$((failures + 1)); }
fi

if rg -q '^skip_cboard_crc:[[:space:]]*true' "$config"; then
  report "WARNING: skip_cboard_crc=true; CRC is disabled"
fi

for target in gimbal_test gimbal_response_test; do
  if test -x "build-ninja/$target"; then
    report "$target is available"
  else
    report "build-ninja/$target is missing"
    failures=$((failures + 1))
  fi
done

if test "$failures" -eq 0; then
  report "PASS: safe checks complete; serial port was not opened"
else
  report "FAIL: $failures blocking check(s); serial port was not opened"
fi
exit "$([ "$failures" -eq 0 ] && printf 0 || printf 1)"
