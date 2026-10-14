# orientation_law_compare

Offline comparison of three orientation laws from the hold orientation to the planner goal in time tau. Pure
orientation analysis (no joint simulation, no position rollout); it changes nothing in planner or executor.

| law | definition |
|---|---|
| slerp | `q0 * (q0^-1 q1)^s`, s = t/tau (`grasp_orientation::slerpAt`, as the executor in planner_goal) |
| minjerk | same geodesic, s(u) = 10u^3 - 15u^4 + 6u^5, u = t/tau |
| qdmp | `QuaternionDMP` with the learned weights, run in the **demo frame**: Q = q_hold * q_demo(0)^-1 (the FrameAligner offset, left multiplication), rollout start = q_demo(0), goal = Q^-1 * q_goal, eta0 = 0, output = Q * q_rollout(t) |

q_demo(0) is the `q0` stored in the `quaternion_dmp` section of the weights file.

Contact-phase variants `nominal`, `phase_p20`, `phase_m20`: the goal is rotated about `--axis` by
delta = +/-0.2 * |omega| * tau, i.e. `q_goal(theta + delta) = R_axis(delta) * q_goal(theta)`.

## Build (default OFF)

    colcon build --packages-select satellite_grasp_planner --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_ANALYSIS_TOOLS=ON

## Run

    ros2 run satellite_grasp_planner orientation_law_compare \
        --weights <weights.yaml> [--qdmp-weights <weights.yaml>] --demo-params <weights_demo_params.yaml> \
        --goal x,y,z,w --omega-deg-s <w> \
        (--q-meas j1,...,j7 --urdf <fer_flat_effort.urdf> | --q-hold x,y,z,w) \
        --out <dir> [--axis x,y,z] [--demo-csv <demo.csv>]

- `--weights`: ProDMP/QDMP yaml; `--qdmp-weights` (default = `--weights`): yaml with the `quaternion_dmp` section.
- `--demo-params`: tau is `demo.tau_s`.
- Hold orientation: FK of the TCP (`fer_hand_tcp`) at `--q-meas` (needs `--urdf`), or `--q-hold` directly.
- `--goal`, `--q-hold`: quaternions in xyzw order. `--axis`: satellite spin axis in world (default 0,0,1).
- `--demo-csv` (optional): recorded demo. Header deduced: time = t|time|time_s|timestamp|t_s; quaternion components
  = q|quat_|q_|orientation_|ori_ + w,x,y,z. Without it the demo section is skipped (stated on stdout).

## Output

stdout: header (tau, phase offset, goal, frame of the QDMP weights), optional recorded-demo section (swept angle,
angle q(0)->q(end), duration, QDMP reproduction error max/RMS and residual at tau in the demo frame), then one table per
variant: swept angle, max |omega|, max |alpha|, residual at tau, and SLERP-vs-QDMP angular difference (max, RMS).
omega and alpha are finite differences of the 5 ms quaternion profile, same method for all laws.

`<out>/summary.csv`, `<out>/profile_<variant>.csv` (t and x,y,z,w per law, 5 ms).
