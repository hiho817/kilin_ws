#!/usr/bin/env bash
# Run one safe, offline terrain-window/planner-shadow replay.
# ROS Humble setup scripts reference optional unset variables, so enable
# nounset only after sourcing the ROS and workspace overlays below.
set -eo pipefail

# The Orin has a user-site NumPy 2.x installation which is binary-incompatible
# with Ubuntu 22.04's ROS Python SciPy.  The real controller launch already
# uses this isolation; make offline replays equally reproducible.
export PYTHONNOUSERSITE=1

usage() {
  cat <<'EOF'
Usage: run_terrain_shadow_replay.sh --bag BAG_DIRECTORY --label LABEL [options]

Options:
  --fastlio-config PATH       FAST-LIO YAML (default: balanced profile)
  --terrain-resolution-m M    Terrain cell size (default: 0.10)
  --rate R                    rosbag replay rate (default: 1.0)
  --record-derived            Record derived debug topics; never copies raw LiDAR/IMU
  --visualize-non-nominal-stance
                              Replay-only: render a planner horizon even when the
                              recorded unarmed hips are not at nominal 45 degrees
  --output-root DIRECTORY     Default: ~/kilin_ws/logs/2026-08-27

The launch is unarmed and writes only to /kilin/terrain_shadow/motor_command.
The bag player publishes only /livox/lidar, /livox/imu, and /motor/state.
EOF
}

bag=""
label=""
rate="1.0"
record_derived=false
visualize_non_nominal_stance=false
terrain_resolution_m="0.10"
ros_ws="${KILIN_ROS_WS:-$HOME/kilin_ws/kilin_ros_ws}"
output_root="${KILIN_TERRAIN_REPLAY_ROOT:-$HOME/kilin_ws/logs/2026-08-27}"
fastlio_config="$ros_ws/src/kilin_fastlio_bringup/config/fastlio_mid360s_terrain_balanced.yaml"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) bag="$2"; shift 2 ;;
    --label) label="$2"; shift 2 ;;
    --fastlio-config) fastlio_config="$2"; shift 2 ;;
    --terrain-resolution-m) terrain_resolution_m="$2"; shift 2 ;;
    --rate) rate="$2"; shift 2 ;;
    --record-derived) record_derived=true; shift ;;
    --visualize-non-nominal-stance) visualize_non_nominal_stance=true; shift ;;
    --output-root) output_root="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -d "$bag" ]] || { echo "Bag directory not found: $bag" >&2; exit 2; }
[[ -n "$label" ]] || { echo "--label is required" >&2; exit 2; }
[[ -f "$fastlio_config" ]] || { echo "FAST-LIO config not found: $fastlio_config" >&2; exit 2; }

source /opt/ros/humble/setup.bash
# Server replays may need the message-only overlay. On Orin the real driver
# already supplies these message definitions, so source the overlay only if it
# exists locally.
if [[ -f "$HOME/kilin_ws/offline_livox_interfaces/install/setup.bash" ]]; then
  source "$HOME/kilin_ws/offline_livox_interfaces/install/setup.bash"
fi
source "$ros_ws/install/setup.bash"
set -u

run_dir="$output_root/$label"
mkdir -p "$run_dir"
printf 'bag=%q\nlabel=%q\nrate=%q\nfastlio_config=%q\nterrain_resolution_m=%q\nrecord_derived=%q\nvisualize_non_nominal_stance=%q\npython_user_site=%q\n' \
  "$bag" "$label" "$rate" "$fastlio_config" "$terrain_resolution_m" "$record_derived" "$visualize_non_nominal_stance" "$PYTHONNOUSERSITE" > "$run_dir/run.env"
sha256sum "$fastlio_config" >> "$run_dir/run.env"

launch_pid=""
record_pid=""
cleanup() {
  [[ -n "$record_pid" ]] && kill "$record_pid" 2>/dev/null || true
  [[ -n "$launch_pid" ]] && kill "$launch_pid" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

launch_args=(
  fastlio_config:="$fastlio_config"
  terrain_resolution_m:="$terrain_resolution_m"
)
if [[ "$visualize_non_nominal_stance" == true ]]; then
  launch_args+=(shadow_initial_hip_error_limit_deg:=180.0)
fi

ros2 launch kilin_fastlio_bringup terrain_shadow_replay.launch.py "${launch_args[@]}" \
  > "$run_dir/launch.log" 2>&1 &
launch_pid=$!

sleep 3

if [[ "$record_derived" == true ]]; then
  ros2 bag record -o "$run_dir/derived_debug" \
    /Odometry /kilin/fastlio/odometry /cloud_registered \
    /kilin/terrain/local_window /kilin/terrain/local_window/cells \
    /kilin/terrain/local_window/bounds /kilin/terrain/local_window_overlay \
    /kilin/planner/debug/horizon /kilin/planner/debug/footprints \
    /motor/state /kilin/terrain_shadow/motor_command /tf /tf_static \
    > "$run_dir/record.log" 2>&1 &
  record_pid=$!
  sleep 1
fi

ros2 bag play "$bag" --clock --rate "$rate" \
  --topics /livox/lidar /livox/imu /motor/state \
  > "$run_dir/player.log" 2>&1

echo "Completed unarmed shadow replay: $run_dir"
