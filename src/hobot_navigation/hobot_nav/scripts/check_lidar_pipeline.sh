#!/usr/bin/env bash
set -euo pipefail

scan_raw_topic="${SCAN_RAW_TOPIC:-/scan_raw}"
scan_topic="${SCAN_TOPIC:-/scan}"
cone_points_topic="${CONE_POINTS_TOPIC:-/cones/points}"
cone_poses_topic="${CONE_POSES_TOPIC:-/cones/poses}"
cone_markers_topic="${CONE_MARKERS_TOPIC:-/cones/markers}"
base_frame="${BASE_FRAME:-base_link}"
laser_frame="${LASER_FRAME:-laser_link}"
odom_topic="${ODOM_TOPIC:-/odom}"
cmd_topic="${CMD_TOPIC:-/cmd_vel}"
cmd_safe_topic="${CMD_SAFE_TOPIC:-/cmd_vel_safe}"

has_topic() {
  ros2 topic list | grep -Fxq "$1"
}

check_hz() {
  local topic="$1"
  local seconds="${2:-8}"
  echo
  echo "== ${topic} hz =="
  if has_topic "$topic"; then
    timeout "${seconds}s" ros2 topic hz "$topic" || true
  else
    echo "missing topic: ${topic}"
  fi
}

check_once() {
  local topic="$1"
  local fields="${2:-}"
  echo
  echo "== ${topic} sample =="
  if has_topic "$topic"; then
    if [[ -n "$fields" ]]; then
      ros2 topic echo "$topic" --once --field "$fields" || true
    else
      ros2 topic echo "$topic" --once || true
    fi
  else
    echo "missing topic: ${topic}"
  fi
}

echo "== lidar topics =="
ros2 topic list | grep -E "(^${scan_raw_topic}$|^${scan_topic}$|^/cones/|^/local_costmap/|^/global_costmap/)" || true

echo
echo "== nodes =="
ros2 node list | grep -E '(scan_filter|scan_deskew|cone_detector|costmap|collision_monitor)' || true

check_hz "$scan_raw_topic"
check_hz "$scan_topic"
check_hz "$cone_points_topic"

echo
echo "== ${scan_topic} type/info =="
if has_topic "$scan_topic"; then
  ros2 topic info "$scan_topic" || true
fi

check_once "$scan_topic" "header"
check_once "$scan_topic" "angle_min"
check_once "$scan_topic" "angle_max"
check_once "$scan_topic" "angle_increment"
check_once "$scan_topic" "scan_time"
check_once "$scan_topic" "time_increment"
check_once "$cone_points_topic" "header"

echo
echo "== ${base_frame} -> ${laser_frame} tf =="
timeout 8s ros2 run tf2_ros tf2_echo "$base_frame" "$laser_frame" || true

echo
echo "== local costmap topics =="
ros2 topic list | grep -E '^/local_costmap/(costmap|costmap_raw|published_footprint)$' || true

echo
echo "== collision monitor =="
if ros2 node list | grep -Fxq "/collision_monitor"; then
  ros2 topic list | grep -E "(^${cmd_topic}$|^${cmd_safe_topic}$|collision_monitor)" || true
else
  echo "collision_monitor node not running"
fi

echo
echo "Suggested rosbag command:"
echo "ros2 bag record -o /root/lidar_debug_\$(date +%Y%m%d_%H%M%S) \\"
echo "  ${scan_raw_topic} ${scan_topic} ${cone_points_topic} ${cone_poses_topic} ${cone_markers_topic} \\"
echo "  /tf /tf_static ${odom_topic} ${cmd_topic} ${cmd_safe_topic} \\"
echo "  /local_costmap/costmap /local_costmap/costmap_raw /local_costmap/published_footprint"
