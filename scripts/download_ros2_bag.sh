#!/usr/bin/env bash
set -euo pipefail

target="${BAG_PATH:-/external/euroc/MH_01_easy_ros2_ready}"
drive_url="${BAG_DRIVE_URL:-}"

if [[ -z "${drive_url}" ]]; then
  echo "BAG_DRIVE_URL is empty; cannot download ROS2 bag." >&2
  exit 2
fi

if [[ -z "${target}" || "${target}" == "/" ]]; then
  echo "Refusing unsafe BAG_PATH: ${target}" >&2
  exit 2
fi

if [[ -f "${target}/metadata.yaml" ]]; then
  echo "ROS2 bag already exists at ${target}"
  exit 0
fi

parent="$(dirname "${target}")"
tmp="${target}.download.$$"

mkdir -p "${parent}"
rm -rf "${tmp}"
mkdir -p "${tmp}"

echo "Downloading ROS2 bag from Google Drive folder:"
echo "  ${drive_url}"
echo "Temporary directory:"
echo "  ${tmp}"

gdown --folder "${drive_url}" -O "${tmp}"

metadata_path="$(find "${tmp}" -name metadata.yaml -type f -print -quit)"
if [[ -z "${metadata_path}" ]]; then
  echo "Downloaded files do not contain metadata.yaml." >&2
  rm -rf "${tmp}"
  exit 3
fi

bag_dir="$(dirname "${metadata_path}")"
if ! find "${bag_dir}" -maxdepth 1 -name "*.db3" -type f | grep -q .; then
  echo "Downloaded bag directory does not contain a .db3 file: ${bag_dir}" >&2
  rm -rf "${tmp}"
  exit 3
fi

rm -rf "${target}"
if [[ "${bag_dir}" == "${tmp}" ]]; then
  mv "${tmp}" "${target}"
else
  mv "${bag_dir}" "${target}"
  rm -rf "${tmp}"
fi

echo "Downloaded ROS2 bag to ${target}"
ros2 bag info "${target}"
