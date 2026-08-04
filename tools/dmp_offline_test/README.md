# dmp_offline_test

Offline test harness (no ROS2, no Docker, no physical device) for `core::DMP` and
`core::QuaternionDMP` — points directly to the official package source files (`../../src/haptic_dmp_learning`), no duplicate code copies.

## Structure

```
tools/dmp_offline_test/
├── build/                          # Compiled executables
├── data/                           # Generated CSV files (synthetic demos, replay, metrics)
├── weights/                        # Generated YAML files with DMP weights
├── plots/                          # Generated PNG plots
├── metrics.hpp / metrics.cpp       # Evaluation metrics, LOCAL to tools
├── test_core_offline.cpp           # Multi-trajectory synthetic test
├── build_and_run.sh
├── replay_saved_dmp.cpp            # Loads real dmp_weights.yaml and regenerates replay
├── replay_build_and_run.sh
├── plot_dmp_test.py                # Plot synthetic demo vs replay, per trajectory
└── plot_real_demo.py               # Plot REAL demo (Geomagic) vs replay + metrics
```

## Synthetic Multi-Trajectory Test

```bash
chmod +x build_and_run.sh
./build_and_run.sh
```

Tests **4 different trajectories** in sequence, each with a specific rationale:

| Trajectory | Purpose |
|---|---|
| `reach_semplice` | Simple case, baseline — expected high fidelity |
| `reach_lift_pitch` | Mixed case (translation + bump + rotation) — used in previous tests |
| `rotazione_pura` | Minimal positional displacement, wide rotation — isolates Quaternion DMP; triggers guardrail A under new goal |
| `reach_complesso_gradino` | Step-like profile — reproduces multi-segment trajectory limits observed on real hardware |

For each trajectory, it generates:
- `data/demo_original_<name>.csv`, `data/replay_same_goal_<name>.csv`, `data/replay_new_goal_<name>.csv`
- `weights/dmp_weights_<name>.yaml` (combined position + orientation format)
- A row in `data/metrics_summary.csv` for `_same_goal` (RMSE, max error) and `_new_goal` (final endpoint error)
- Console warnings for guardrail A (`isScaleReliable`)

## Plotting a Specific Trajectory

```bash
python3 plot_dmp_test.py reach_lift_pitch
```

Without arguments, defaults to `reach_lift_pitch` or lists available trajectories.

## Test with REAL Demo from Geomagic Touch

Once a real demo is recorded (button 0 → move device → button 1), the wrapper outputs `dmp_weights.yaml` and `dmp_demo_recorded.csv` in `~/thesis_ws/`.

```bash
chmod +x replay_build_and_run.sh
./replay_build_and_run.sh ~/thesis_ws/dmp_weights.yaml
python3 plot_real_demo.py ~/thesis_ws/dmp_demo_recorded.csv
```

`plot_real_demo.py` prints metrics (position RMSE, angular error) directly to console alongside the plot using the exact same formulas as `metrics.cpp`.

## Note on `metrics.hpp/.cpp`

Located **only here**, not in the ROS2 package (`src/haptic_dmp_learning`) — validation/testing tools only. Uses namespace `dmp_tools::metrics`, separated from `haptic_dmp_learning::core`.
