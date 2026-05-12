#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
set -u

COMMAND="${1:-demo}"
shift || true

BAG_PATH="${BAG_PATH:-/external/euroc/MH_01_easy_ros2_ready}"
SAVED_DIR="${SAVED_DIR:-/data/saved}"
BAG_RATE="${BAG_RATE:-0.5}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/cam0/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/cam0/camera_info}"
WEB_PORT="${WEB_PORT:-8080}"
BAG_AUTO_DOWNLOAD="${BAG_AUTO_DOWNLOAD:-1}"
BAG_DRIVE_URL="${BAG_DRIVE_URL:-}"

download_ros2_bag_if_enabled() {
  if [[ "${BAG_AUTO_DOWNLOAD}" == "0" ]]; then
    return 1
  fi
  if [[ -z "${BAG_DRIVE_URL}" ]]; then
    return 1
  fi
  /opt/euroc_web_lab/scripts/download_ros2_bag.sh
  return 0
}

require_ros2_bag() {
  if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
    echo "ROS2 bag not found at ${BAG_PATH}"
    echo "Trying automatic Google Drive download if configured..."
    if download_ros2_bag_if_enabled && [[ -f "${BAG_PATH}/metadata.yaml" ]]; then
      return 0
    fi
    echo "ROS2 bag not found at ${BAG_PATH}" >&2
    echo "Mount a ROS2 bag directory and set BAG_PATH, for example:" >&2
    echo "  BAG_PATH=/external/euroc/MH_01_easy_ros2_ready" >&2
    echo "Or set BAG_DRIVE_URL to a public Google Drive folder and keep BAG_AUTO_DOWNLOAD=1." >&2
    exit 2
  fi
}

case "${COMMAND}" in
  demo)
    mkdir -p "${SAVED_DIR}"
    require_ros2_bag
    echo "Starting web bridge on http://localhost:${WEB_PORT}"
    echo "The web bridge manages ros2 bag play from ${BAG_PATH} at rate ${BAG_RATE}."
    ros2 run euroc_web_lab web_bridge_node --ros-args \
      -p image_topic:="${IMAGE_TOPIC}" \
      -p camera_info_topic:="${CAMERA_INFO_TOPIC}" \
      -p save_dir:="${SAVED_DIR}" \
      -p port:="${WEB_PORT}" \
      -p bag_path:="${BAG_PATH}" \
      -p bag_rate:="${BAG_RATE}" \
      -p manage_bag_play:=true
    ;;
  bag-info)
    require_ros2_bag
    ros2 bag info "${BAG_PATH}"
    ;;
  download-bag)
    download_ros2_bag_if_enabled
    ;;
  shell)
    exec bash
    ;;
  *)
    exec "${COMMAND}" "$@"
    ;;
esac
