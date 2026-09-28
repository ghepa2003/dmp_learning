#!/usr/bin/env bash
# Generates all 4 standard comparison plots for one learning cycle in one
# command:
#   1. taught trajectory (device)      vs learned DMP (algorithmic replay)
#   2. taught trajectory (device)      vs robot's actual execution while live-mirroring
#   3. robot's actual live execution   vs robot's actual DMP-replay execution
#   4. taught trajectory (device)      vs robot's actual DMP-replay execution
#
# Assumes the standard live-demo file layout and that actual_pose_live.csv /
# actual_pose_replay.csv were recorded with record_pose_topic.py's
# --buttons-trigger / --pose-trigger modes (already trimmed to the active
# motion window - no --duration/--trim-idle needed here). If you recorded
# manually (Ctrl+C), regenerate plot 2/3/4 yourself with plot_pose_csvs.py
# and the appropriate --duration/--window flags instead of using this
# script.
#
# Plots 2 and 4 compare a device-frame trajectory (taught) against a
# robot-frame one (actual_pose*.csv) - these differ by the same one-shot
# rigid SE(3) offset franka_cartesian_control's FrameAligner applies at
# controller activation, so plot_pose_csvs.py's --align is used to anchor
# the taught trajectory onto the robot's frame before plotting (see
# plot_pose_csvs.py's own docstring for details). Plots 1 and 3 are each
# already within a single, shared frame and need no such alignment.
#
# Usage (all arguments optional, defaults match the rest of this project's
# conventions):
#   plot_all_comparisons.sh [weights.yaml] [demo_raw.csv] [actual_pose_live.csv] [actual_pose_replay.csv]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

WEIGHTS="${1:-$HOME/thesis_ws/live_demo_dmp_weights.yaml}"
DEMO_RAW="${2:-$HOME/thesis_ws/live_demo_raw.csv}"
ACTUAL_LIVE="${3:-$HOME/thesis_ws/actual_pose_live.csv}"
ACTUAL_REPLAY="${4:-$HOME/thesis_ws/actual_pose_replay.csv}"

for f in "$WEIGHTS" "$DEMO_RAW" "$ACTUAL_LIVE" "$ACTUAL_REPLAY"; do
    if [ ! -f "$f" ]; then
        echo "[ERROR] File not found: $f" >&2
        exit 1
    fi
done

OUT_DIR="plots/full_comparison"
mkdir -p "$OUT_DIR"

echo "== [1/4] Taught trajectory vs learned DMP (algorithmic replay) =="
./02_real_data/scripts/replay_build_and_run.sh "$WEIGHTS" >/dev/null
MPLBACKEND=Agg python3 02_real_data/scripts/plot_real_demo.py "$DEMO_RAW"
echo "  -> plots/02_real_data/real_demo_plot.png"
echo "  -> plots/02_real_data/angular_error_over_time.png"
echo ""

echo "== [2/4] Taught trajectory vs robot's actual execution while live-mirroring =="
MPLBACKEND=Agg python3 common/scripts/plot_pose_csvs.py "$OUT_DIR/taught_vs_robot_live.png" \
    "$DEMO_RAW" "taught (device)" \
    "$ACTUAL_LIVE" "robot (live mirroring)" \
    --align 0:1
echo ""

echo "== [3/4] Robot's actual live execution vs robot's actual DMP-replay execution =="
MPLBACKEND=Agg python3 common/scripts/plot_pose_csvs.py "$OUT_DIR/robot_live_vs_robot_replay.png" \
    "$ACTUAL_LIVE" "robot (live mirroring)" \
    "$ACTUAL_REPLAY" "robot (DMP replay)"
echo ""

echo "== [4/4] Taught trajectory vs robot's actual DMP-replay execution =="
MPLBACKEND=Agg python3 common/scripts/plot_pose_csvs.py "$OUT_DIR/taught_vs_robot_replay.png" \
    "$DEMO_RAW" "taught (device)" \
    "$ACTUAL_REPLAY" "robot (DMP replay)" \
    --align 0:1
echo ""

echo "All 4 plots saved under:"
echo "  $ROOT_DIR/plots/02_real_data/real_demo_plot.png"
echo "  $ROOT_DIR/$OUT_DIR/taught_vs_robot_live.png"
echo "  $ROOT_DIR/$OUT_DIR/robot_live_vs_robot_replay.png"
echo "  $ROOT_DIR/$OUT_DIR/taught_vs_robot_replay.png"
