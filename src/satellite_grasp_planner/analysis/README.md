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
variant: swept angle, max |omega|, max |alpha|, residual at tau, max angular excursion from q_hold (`max_t angle(q(t), q_hold)`),
and SLERP-vs-QDMP angular difference (max, RMS).
omega and alpha are finite differences of the 5 ms quaternion profile, same method for all laws.

`<out>/summary.csv` (incl. `max_angle_from_hold_deg`), `<out>/profile_<variant>.csv` (t and x,y,z,w per law, 5 ms).

`--demo-csv` reproduction control: the QDMP is run with the **eta0 stored in the weights file** (declared on stdout); the
grasp rollouts (all tables above) start from rest, eta0 = 0.

Shared definitions (laws, phase variants, QDMP frame offset) live in `orientation_laws.hpp`.

# orientation_path_check

Kinematic feasibility ALONG the path for the same laws/variants (definitions shared via `orientation_laws.hpp`).
Same chain as the planner's post-hoc path check (`checkKinematicFeasibility`): ProDMP position rollout from FK(`--q-meas`)
to `p0 + delta_g_demo_m` (or `--p-goal`; same for every variant: the phase variants change only the orientation),
orientation law sampled at t_i = (i+1)*dt, `JointPathSimulator` + `NullspaceBiasedResolution` (no per-sample IK, no clamp).
"IK fail" = first instant a joint is outside its limits by > 1e-4 rad (or non-finite q).

    ros2 run satellite_grasp_planner orientation_path_check \
        --weights <w.yaml> [--qdmp-weights <w.yaml>] --demo-params <p.yaml> --urdf <fer_flat_effort.urdf> \
        --q-meas j1,...,j7 --goal x,y,z,w --omega-deg-s <w> --out <dir> \
        [--axis x,y,z] [--p-goal x,y,z] [--psi-ref x,y,z,w] [--dt 0.02] [--sim-dt 0.005]

- `--dt` (default 20 ms) = step of the profile CSVs; the simulation integrates at `--sim-dt` (default 5 ms, the planner's step) and
  all extrema are taken on the 5 ms grid. `--dt` must be a multiple of `--sim-dt`.
- Thresholds are printed at the top, local constants of the tool (the former `S3Params` thresholds margin 0.3 rad / w_hat 0.5, no longer planner parameters; `VelocityIkSolver::Params::max_joint_speed`
  of the simulator); a `*` marks a violation. w_hat = w_trans / w_trans_demo (`demo.end_joints_rad`).
- `--psi-ref`: nominal (psi = 0) TCP orientation of the goal grasp (`orientation_nominal_world`); psi(tau) is the rotation of q(tau) about the
  local z relative to it. Without it psi is reported as not computed.
- Hand-collar distance: not computed by this tool (the planner's v2 clearance constraint, `approachClearance`, is not called here).

Output: table per variant on stdout, `<out>/summary.csv`, `<out>/profile_<law>_<variant>.csv` (t, w_hat, min margin, angle from hold, q1..q7).

# s3_landscape

Exports the landscape of the S3 selector (`selectGraspS3`) per candidate, for OFFLINE evaluation of other cost formulations
(`tools/s3_landscape_report.py`). The selector is called as is (`planGraspAndLaunchS1` with `eval_log`, `S3Context::eval_log`);
nothing is changed in the selection. NOTE: the selector is now single-mode (J_static / J, no legacy cost) and requires `rollout_q_ref`, which this
tool does not pass yet; the `--selftest` checks on the winner and the RUN15 reference below describe the removed legacy selection and are not valid until the tool is reworked. `--selftest` compares the tool's psi = 0 rows with the real log and the winner (exit != 0 on mismatch).

    colcon build --packages-select satellite_grasp_planner --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_ANALYSIS_TOOLS=ON
    ros2 run satellite_grasp_planner s3_landscape --selftest
    ros2 run satellite_grasp_planner s3_landscape --seed-spread 0 --out landscape.csv

Arguments (defaults = the 15/10/2026 run): `--weights-dir runs/20261015_095418_fit_demo_20261014T150851`, `--params <planner yaml>` (default `config/grasp_planner_example.yaml`, found via the installed package, then the source tree; missing file, empty `--params` or a yaml without `cube_geometry.grasp_radius_m` = error, the code's default radius 0.0725 is never used;
flat `s1.*`, `cube_geometry.*` keys; precedence defaults < yaml < CLI), `--urdf` ($HOME/thesis_ws/fer_flat_effort.urdf),
`--q-start` (default = the run's start joints `-0.000009,-0.786572,0.000002,-2.378914,-0.000006,3.141609,0.785398`, i.e. NOT
`RobotModel::readyPose()` (q6 = pi/2); `ready` selects readyPose), `--omega-deg-s -2`, `--depth 0.0125`, `--follow-s 3.0`,
`--psi-max-deg 5`, `--psi-step-deg 1`, `--theta-step-deg 2`, `--seed-spread N` (0 = off),
`--theta-extra-deg a,b,...` (extra thetas, all k, e.g. `-163.5,144`), `--refine-top-n 6`, `--refine-step-deg 0.1`, `--refine-halfwidth-deg 2.0`, `--out landscape.csv`,
`--rollout all|psi0|none` (default all: one `checkKinematicFeasibility` per IK-feasible row; slow), plus scenario values needed to
reproduce the run: `--center 0.70,0.0,0.77`, `--theta-snap-rad -1.6727670737402633`, `--t-snap-s 18.318` (RUN15; the tool uses t_now = snap.t_s, so `--theta-snap-rad` must be theta_now = theta_snap + omega*elapsed = -1.626193073 + (-0.034906585*1.334242)), `--min-delay-s 5.0`,
`--posthoc-velocity 0` (the campaigns ran without the velocity check).
The scenario (params path, grasp_radius_m, depth, omega, center, snapshot, q_start, demo) is printed on stderr and written to `<out>.meta` (key=value, read by `tools/s3_landscape_report.py`).
`--selftest` with the default scenario also checks the RUN15 winner (k=0, theta -163.5 deg, d_pos 7.1713 mm, wait 9.3548 s, w_hat 0.8238; SELFTEST FAIL, exit 3 otherwise); with a modified scenario it prints `RUN15 reference non applicabile (scenario modificato)`.

psi: roll about the local z of the TCP = approach axis z_tcp = -R(theta) n_body; TCP position unchanged (checked, printed).

CSV (one row per k, theta, psi; k = 0..3 means 0/90/180/270 deg):
`k, theta_deg, psi_deg, accepted` (a-priori constraints pass: mask & 15 == 0), `reject_first` (first set bit in the order below, `none`),
`reject_mask`, `ik_ok_A` / `ik_ok_B` (nominal / nominal*Rz(pi) converged), `branch_selected` (0 nominal, 1 flipped), `d_pos_mm`, `t_wait_s`
(planLaunch, for ALL rows), `J_current` = J_static of the selector (d_mm + kappa_w*(-ln w_full), t_wait = 0; the column keeps its old name), `w_hat`, `w_trans`, `w_rot` (3x7 angular block), `w_full` (6x7), `smin_trans`,
`smin_rot` (smallest singular values of the two 3x7 blocks), `margin_rollout_rad` (`checkKinematicFeasibility.min_joint_margin_rad`, the
planner's path-check margin; NaN if not computed / IK infeasible), `margin_follow_rad` (min of margin_rollout and the margins of 7 follow-arc samples,
t_j = j*follow_s/6, pose rotated by omega*t_j about the satellite axis through the centre, hot-started IK on the same branch, nullspace
reference q_start; if the rollout was not computed: follow samples only; NaN if the IK fails), `follow_ik_ok`, `path_ok`, `w_full_other`,
`margin_other` (static IK margin of the NOT selected branch), `margin_ik_rad` (static IK margin of the selected branch = the margin of the S3
constraint), `dv_m_s`, `w_full_seedmin/max`, `margin_seedmin/max`, `n_seed_conv` (psi = 0 accepted rows, `--seed-spread N`, seeds q_start +- 0.3 rad uniform,
mt19937 seed 12345).

`reject_mask` bits: 1 = ik, 8 = r_trust, 128 = start_goal (a-priori, `S3RejectBit`, decide `accepted`; bits 2 and 4, w_hat / static margin, are retired); 16 = velocity
(dv > delta_v_max), 32 = path (joint limits along the rollout), 64 = follow-arc IK failure (diagnostics; they do not change `accepted`).
Branch rule and constraints are the planner's (`evaluateGraspPoseDetailed`, `s3AprioriRejectMaskV2`); only the coarse grid (theta step) is exported.

Theta grids (CSV column `grid`): `coarse` = the selector's grid (theta = -pi + i*step, unchanged); `fine` = for each k the top `--refine-top-n`
coarse candidates by `d_pos` (psi = 0, IK ok on the chosen branch, `reject_mask` bit 1 clear) re-evaluated at theta_c + j*`--refine-step-deg`,
|j*step| <= `--refine-halfwidth-deg`, j != 0, for ALL psi of the band (duplicates between neighbourhoods are skipped); `extra` = the
`--theta-extra-deg` values (all k, all psi; skipped if the theta was already evaluated). `--selftest` compares the coarse psi = 0 rows with the
selector's log and prints separately the winner over all psi = 0 rows (coarse + fine + extra), checking that the selector's chosen (k, theta)
is among them (expected for the 15/10 run: k=0, theta=-163.5, d_pos 7.171 mm). The fine/extra rows add about 4*6*40*|psi band| evaluations
(rollout + follow included): use `--rollout psi0` for a quick pass.

# s3_clearance

Clearance (signed distance gripper <-> rotating satellite over the window [0, tau], see `docs/S3_SPEC.md` section 10) of ONE candidate,
at a given satellite rate. Same scenario inputs as `s3_landscape` (`--weights-dir`, `--params`, `--urdf`, `--q-start`, `--depth`,
`--center`, `--theta-snap-rad`, `--t-snap-s`; defaults = the 15/10 run) plus the candidate: `--k 0..3`, `--theta-deg`, `--flip 0|1`
(wrist branch, forced) and `--omega-deg-s` (signed; it is the snapshot's omega AND the one of the clearance). `--hand-mesh <stl>`
overrides the `fer_hand` collision mesh read from the URDF through `AMENT_PREFIX_PATH`. `s1.clearance_dt_s` and `s1.gripper_open_m`
are read from `--params` (defaults 0.05 s, 0.060 m).

    ros2 run satellite_grasp_planner s3_clearance --k 2 --theta-deg 145.3 --flip 1 --omega-deg-s -4.0
    ros2 run satellite_grasp_planner s3_clearance --selftest

Output (stdout, one line; details on stderr):
`CLEARANCE k=.. theta=.. flip=.. omega=.. min_mm=.. coppia=.. t_rel_s=.. min_mm_senza_ultimi_2s=..` - `min_mm`, `coppia` (robot part - satellite
part) and `t_rel_s` (= t - tau) refer to the whole window; `min_mm_senza_ultimi_2s` is the value the constraint uses.
The trajectory is the planner's (`timedApproachTrajectory`: start pose, ProDMP + SLERP, executor time grid), goal = grasp position +
`contact_to_end_offset`. A candidate whose IK does not converge is still evaluated (only the TCP path matters) with a warning.
`--selftest` (no URDF, no demo): synthetic hand, real fingers and satellite, known values 100 mm and 10 mm; exit 3 on mismatch.

## s3_followmargin_check

Riproduce con `followMargin()` i margini previsti di G16 / step 0c (13.26, 17.67, 35.00, 35.00, 20.84, 28.63 gradi, tolleranza 0.1) e stampa
joint e istante per il confronto con `_data/diag_audit/_diag_1019/g16/g16_results.csv`. Scenario e provenienza dei valori nel commento in testa a
`analysis/s3_followmargin_check.cpp`. Da eseguire dalla radice del workspace (`--weights-dir`, `--urdf` con gli stessi default di `step0c_margins`).
Nota: `margin_follow_rad` del landscape NON e' questa grandezza (IK statica sull'arco, non simulazione): vedi docs/S3_SPEC.md §10.8.
