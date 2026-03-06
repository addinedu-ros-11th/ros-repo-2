#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="/home/pinky"
WEB_DIR="${ROOT_DIR}/pinky_web"
VISION_DIR="${ROOT_DIR}/pinky_visionNav/models"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi
if [ -f "${ROOT_DIR}/pinky_pro/install/local_setup.bash" ]; then
  # shellcheck disable=SC1090
  source "${ROOT_DIR}/pinky_pro/install/local_setup.bash"
fi
if [ -f "${WEB_DIR}/.env" ]; then
  # shellcheck disable=SC1090
  source "${WEB_DIR}/.env"
fi

set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-11}"
# Prefer the local camera stack over ROS-packaged libcamera bindings.
export PYTHONPATH="/usr/local/lib/aarch64-linux-gnu/python3.12/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/usr/local/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
export LIBCAMERA_IPA_MODULE_PATH="/usr/local/lib/aarch64-linux-gnu/libcamera"

POSE_MODEL="${VISION_DIR}/yolo11n-pose.pt"
OBJ_MODEL="${VISION_DIR}/yolo11n.pt"
if [ ! -f "${OBJ_MODEL}" ]; then
  OBJ_MODEL="${POSE_MODEL}"
fi

# Camera orientation is controlled in one place: WEB_DIR/.env (PINKY_CAM_FLIP).
CAM_FLIP_MODE="$(printf '%s' "${PINKY_CAM_FLIP:-none}" | tr '[:upper:]' '[:lower:]')"
FLIP_ARGS=()
case "${CAM_FLIP_MODE}" in
  none|"")
    ;;
  h)
    FLIP_ARGS+=(--hflip)
    ;;
  v)
    FLIP_ARGS+=(--vflip)
    ;;
  hv|vh|both)
    FLIP_ARGS+=(--hflip --vflip)
    ;;
  *)
    echo "[run_pinky_web] Invalid PINKY_CAM_FLIP='${CAM_FLIP_MODE}', fallback to 'hv'" >&2
    CAM_FLIP_MODE="hv"
    FLIP_ARGS+=(--hflip --vflip)
    ;;
esac
echo "[run_pinky_web] camera flip mode: ${CAM_FLIP_MODE}" >&2

exec python3 "${WEB_DIR}/pinky_web.py" \
  --host 0.0.0.0 \
  --port 8080 \
  --width 320 \
  --height 240 \
  --lcd-default-text "11" \
  "${FLIP_ARGS[@]}" \
  --camera-timeout 12 \
  --safety-us-stop 0.06 \
  --safety-us-slow 0.12 \
  --safety-us-min-valid 0.03 \
  --safety-scan-stop 0.07 \
  --safety-scan-slow 0.12 \
  --safety-scan-back-stop 0.06 \
  --safety-scan-back-slow 0.11 \
  --safety-min-speed 0.10 \
  --safety-scan-front-half-deg 40 \
  --safety-scan-center-deg 0 \
  --safety-scan-swap-lr 1 \
  --safety-scan-swap-fb 0 \
  --safety-avoid-enabled 1 \
  --safety-avoid-back-sec 0.45 \
  --safety-avoid-turn-sec 2.20 \
  --safety-avoid-back-speed 0.09 \
  --safety-avoid-turn-speed 0.80 \
  --safety-avoid-step-deg 30 \
  --safety-avoid-cooldown 1.5 \
  --safety-avoid-resume-sec 0.70 \
  --safety-avoid-resume-speed 0.08 \
  --safety-avoid-clear-scan 0.24 \
  --safety-avoid-clear-us 0.20 \
  --reverse-speed-scale 0.30 \
  --reverse-turn-scale 0.55 \
  --digit-forward-speed 0.36 \
  --digit-forward-min-scale 0.70 \
  --digit-stop-box-ratio 0.08 \
  --digit-arrive-box-ratio 0.20 \
  --digit-marker-size-m 0.16 \
  --digit-arrive-dist-m 0.45 \
  --digit-arrive-ratio-fallback 0.30 \
  --digit-seek-forward-speed 0.15 \
  --digit-switch-backoff-sec 1.2 \
  --digit-backoff-speed 0.12 \
  --digit-search-ang 0.40 \
  --number-fps 6 \
  --number-turn-gain 1.10 \
  --number-forward-speed 0.36 \
  --number-arrive-ratio 0.34 \
  --number-arrive-center-err 0.11 \
  --number-arrive-streak-need 5 \
  --number-search-ang 0.20 \
  --number-map-seek-forward-speed 0.14 \
  --number-target-lock-sec 1.10 \
  --number-center-deadband 0.09 \
  --number-hold-forward-speed 0.10 \
  --number-ocr-enabled 1 \
  --number-ocr-threshold 0.28 \
  --number-ocr-stride 2 \
  --pose-model "${POSE_MODEL}" \
  --obj-model "${OBJ_MODEL}"
