# Design Notes

Short technical notes on design decisions / non-issues that came up during
investigation but did not warrant (or did not need) a code change. These
supplement, and do not replace, any existing documentation.

## 2026-09-11 — Quaternion normalization in `learn_and_test_dmp.cpp`'s local CSV loader

**Context**: `demo_csv_io::readDemoCsv` (used by `learn_and_test_prodmp.cpp`)
normalizes the quaternion read from each CSV row
(`Eigen::Quaterniond(...).normalized()`). The local loader inside
`tools/dmp_offline_test/common/src/learn_and_test_dmp.cpp` does **not**
normalize (`s.orientation = Eigen::Quaterniond(vals[4], vals[5], vals[6], vals[7]);`).
This matters in principle because `QuaternionDMP::log_q(q)` computes
`theta = arccos(qw)`, which requires `|qw| <= 1` — only guaranteed for a
truly unit quaternion.

**Diagnosis performed** (read-only investigation, standalone Python scripts,
no production code touched):

1. Measured the deviation of the raw (pre-normalization) quaternion norm
   from 1.0, `|1 - sqrt(qw^2+qx^2+qy^2+qz^2)|`, per row, for the three demo
   CSVs at the repo root:

   | demo | rows | max dev | mean dev | p50 | p95 | p99 |
   |---|---|---|---|---|---|---|
   | demo_raw_trajA.csv | 62633 | 6.932e-7 | 2.357e-7 | 2.097e-7 | 5.382e-7 | 6.401e-7 |
   | demo_raw_trajC.csv | 68245 | 7.675e-7 | 2.356e-7 | 1.952e-7 | 5.592e-7 | 6.635e-7 |
   | reach_task_baseline.csv | 60900 | 6.907e-7 | 2.393e-7 | 2.082e-7 | 5.491e-7 | 6.437e-7 |

   All three CSVs are already unit-norm to ~1e-6, i.e. essentially
   float32-precision-limited text (likely round-tripped through a
   single-precision device/driver before being written as text). There is
   no evidence in these datasets of quaternions meaningfully off unit norm.

2. Refit classic DMP (`tools/dmp_offline_test/build/learn_and_test_dmp`,
   config: n_basis=200, alpha_x=4.6, alpha_z=25, beta_z=6.25, ridge
   regression with lambda=1e-6, velocity filter window 0.20s/0.20s — the
   exact config recorded in the already-verified `real_trajC_nbasis200.yaml`
   / `dmp_weights_trajC.yaml` weight files) on each demo CSV as-is
   (non-normalized loader, unchanged) and again on a pre-normalized copy of
   each CSV (each quaternion row divided by its own norm; a new file, the
   originals untouched).

   | demo | RMSE non-norm (mm) | RMSE pre-normalized (mm) | Δ | final pos err non-norm (mm) | final pos err pre-norm (mm) | Δ | final orient err non-norm (deg) | final orient err pre-norm (deg) | Δ |
   |---|---|---|---|---|---|---|---|---|---|
   | trajC | 0.296986 | 0.296986 | 0.000000 | 0.0798907 | 0.0798907 | 0.000000 | 0.0735663 | 0.0735663 | 0.000000 |
   | trajA | 0.234995 | 0.234995 | 0.000000 | 0.162079 | 0.162079 | 0.000000 | 0.015209 | 0.015209 | 0.000000 |
   | reach_task_baseline | 0.208075 | 0.208075 | 0.000000 | 0.402913 | 0.402913 | 0.000000 | 0.101490 | 0.101490 | 0.000000 |

   The trajC numbers reproduce the previously documented reference values
   (RMSE 0.2970 mm, final position error 0.0799 mm, final orientation
   error 0.0736°) to 4 decimal places. The pre-normalized fit is identical
   to the non-normalized fit at that precision on all three demos; the raw
   per-sample replay CSVs differ only at the ~1e-6–1e-7 text-representation
   level (last printed digit), consistent with the input deviation
   magnitude above.

**Conclusion**: for the demo CSVs actually in use, the missing
normalization in `learn_and_test_dmp.cpp`'s local loader has **no
measurable effect** on RMSE or final position/orientation error at the
precision already used in thesis material (4 decimal places, mm/deg).
`arccos(qw)` never saw `|qw| > 1` for these inputs; no NaNs were produced.

**Recommendation**: do **not** modify `learn_and_test_dmp.cpp`'s loader (or
`demo_csv_io`) at this time — that loader produced all currently-verified
reference numbers, and re-verifying them against a new loader is not
justified by a discrepancy that doesn't exist in practice. This is
documented here as a known asymmetry between the two loaders, worth
revisiting only if a future demo CSV is captured with visibly worse
quaternion precision (e.g. from a noisier source) where `|1 - ||q|||`
could approach a magnitude that risks `arccos(qw)` seeing `|qw| > 1`. If/when
that happens, the fix is a one-line `.normalized()` call in
`learn_and_test_dmp.cpp`'s local loader (mirroring `demo_csv_io::readDemoCsv`),
followed by re-verification of any reference numbers derived from it.

**Reproducibility**: diagnostic scripts (not part of the build, standalone,
read-only w.r.t. all existing files):
- `analyze_quat_deviation.py` — computes the per-row unit-norm deviation
  stats for one or more demo CSVs.
- `normalize_quat_csv.py` — writes a normalized copy of a demo CSV without
  touching the original.

Both were run from a scratch directory outside the repo for this
investigation; equivalent copies are not committed to the repo as part of
this diagnosis-only task.
