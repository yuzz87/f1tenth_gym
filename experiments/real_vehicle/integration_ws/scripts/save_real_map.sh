#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"

MAP_NAME="${1:-real_map_$(date +%Y%m%d_%H%M%S)}"
if [[ ! "${MAP_NAME}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
    echo "map名は英数字で始め、英数字・._-だけを使用してください。" >&2
    exit 2
fi

OUTPUT_DIR="${F1TENTH_GYM_ROOT}/experiments/real_vehicle/maps/${MAP_NAME}"
MAP_PREFIX="${OUTPUT_DIR}/${MAP_NAME}"
mkdir -p "${OUTPUT_DIR}"

echo "Waiting for /map ..."
if ! ros2 topic info /map --verbose 2>/dev/null \
    | grep -Eq 'Publisher count: [1-9][0-9]*'; then
    echo "/mapを受信できません。real_mapping.launch.pyを起動してください。" >&2
    exit 1
fi

echo "Saving occupancy grid: ${MAP_PREFIX}.yaml"
ros2 run nav2_map_server map_saver_cli \
    -t /map \
    -f "${MAP_PREFIX}" \
    --fmt pgm \
    --mode trinary \
    --ros-args -p save_map_timeout:=10000

echo "Saving slam_toolbox pose graph ..."
if ! timeout 20 ros2 service call \
    /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '${MAP_PREFIX}'}" >/dev/null; then
    echo "pose graphの保存に失敗しました。占有格子地図は保存済みです。" >&2
    exit 1
fi

cat >"${OUTPUT_DIR}/metadata.yaml" <<EOF
map_name: ${MAP_NAME}
created_at: $(date --iso-8601=seconds)
map_frame: map
odom_frame: odom
base_frame: base_link
laser_frame: laser
resolution_m_per_cell: 0.05
lidar_range_m: [0.15, 12.0]
lidar_rear_mask_deg: 30.0
lidar_transform_xyz_m: [0.25, 0.0, 0.11]
lidar_yaw_rad: 0.0
encoder_distance_per_count_m: 0.001415428167021
mapping_method: slam_toolbox_online_async_hand_push
EOF

echo "Saved map files:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -printf '  %f\n' | sort
