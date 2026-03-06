#!/usr/bin/env bash
set -euo pipefail

BASE_URL="http://127.0.0.1:8080"
ENABLE_MOTION_TEST=0

for arg in "$@"; do
  case "$arg" in
    --motion)
      ENABLE_MOTION_TEST=1
      ;;
    http://*|https://*)
      BASE_URL="$arg"
      ;;
    *)
      echo "usage: $0 [BASE_URL] [--motion]" >&2
      exit 2
      ;;
  esac
done

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || { echo "missing command: $1" >&2; exit 2; }
}

require_cmd curl
require_cmd jq

wait_http_ready() {
  local url="$1"
  local timeout_sec="${2:-20}"
  local start_ts now_ts
  start_ts="$(date +%s)"
  while true; do
    if curl -fsS --max-time 1 "$url" >/dev/null 2>&1; then
      return 0
    fi
    now_ts="$(date +%s)"
    if [ $((now_ts - start_ts)) -ge "$timeout_sec" ]; then
      return 1
    fi
    sleep 0.4
  done
}

if ! wait_http_ready "${BASE_URL}/telemetry" 20; then
  echo "smoke_test: web not ready within timeout: ${BASE_URL}" >&2
  exit 1
fi

echo "[1/3] telemetry reachable"
t0="$(curl -fsS "${BASE_URL}/telemetry")"
echo "$t0" | jq -e '.fps != null and .autonav != null and .digit != null' >/dev/null

echo "[2/3] web root reachable"
root_html="$(curl -fsS "${BASE_URL}/")"
[[ "$root_html" == *"Pinky MJPEG Stream"* ]]

echo "[3/3] summary"
echo "$t0" | jq '{fps, us, autonav: .autonav.enabled, digit: .digit.enabled}'

if [ "${ENABLE_MOTION_TEST}" = "1" ]; then
  echo "[motion] autonav start/stop"
  curl -fsS "${BASE_URL}/autonav/start?mode=person_follow" >/dev/null
  sleep 0.6
  t1="$(curl -fsS "${BASE_URL}/telemetry")"
  echo "$t1" | jq -e '.autonav.enabled == true and .autonav.mode == "person_follow"' >/dev/null
  curl -fsS "${BASE_URL}/autonav/stop" >/dev/null
  sleep 0.4
  t2="$(curl -fsS "${BASE_URL}/telemetry")"
  echo "$t2" | jq -e '.autonav.enabled == false' >/dev/null

  echo "[motion] digit start/stop"
  curl -fsS "${BASE_URL}/digit/start?id=1" >/dev/null
  sleep 0.8
  d1="$(curl -fsS "${BASE_URL}/digit/status")"
  echo "$d1" | jq -e '.enabled == true and .target_id == 1 and (.state|type=="string")' >/dev/null
  curl -fsS "${BASE_URL}/digit/stop" >/dev/null
  sleep 0.3
  d2="$(curl -fsS "${BASE_URL}/digit/status")"
  echo "$d2" | jq -e '.enabled == false and .state == "idle"' >/dev/null

  echo "[motion] number start/stop"
  curl -fsS "${BASE_URL}/number/start?id=2" >/dev/null
  sleep 0.8
  n1="$(curl -fsS "${BASE_URL}/number/status")"
  echo "$n1" | jq -e '.enabled == true and .target_id == 2 and (.state|type=="string")' >/dev/null
  curl -fsS "${BASE_URL}/number/stop" >/dev/null
  sleep 0.3
  n2="$(curl -fsS "${BASE_URL}/number/status")"
  echo "$n2" | jq -e '.enabled == false and .state == "idle"' >/dev/null
fi

echo "smoke_test: PASS"
