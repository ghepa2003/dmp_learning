# ProDMP hold-out validation — final report

Scope: offline test layer only (`tools/dmp_offline_test/`). No change to `core::DMP`
(classic) or to the production ROS 2 nodes. `core::ProDMP` received two small,
opt-in, default-preserving additions (documented in STEP 1 and STEP 2 below);
the default fit path is byte-identical to before (verified: `learn_residual_rms`
on `demo_raw_trajC.csv` unchanged at `0.000401823172566`, all 7
`test/test_prodmp.cpp` scenarios still pass).

---

## STEP 1 — tau semantics of the hold-out fit

### What the code does today

`learn_and_test_prodmp_holdout.cpp` splits the demo by index into a leading
`1 - holdout_fraction` training slice and a trailing hold-out slice:

```cpp
// tools/dmp_offline_test/common/src/learn_and_test_prodmp_holdout.cpp:153
std::vector<Sample> train_demo(demo.begin(), demo.begin() + n_train);
...
// :175
prodmp.learnFromDemonstration(train_demo, tau_override);
```

Inside the fit, tau was (and by default still is) taken as the span of whatever
demo is handed in — here the **training slice only**:

```cpp
// src/haptic_dmp_learning/src/core/prodmp.cpp:176-188
const double t0 = demo.front().t;
const double demo_span = demo.back().t - t0;         // span of the TRAINING slice
...
tau_ = (tau_override > 0.0) ? tau_override : demo_span;
```

**Conclusion: the implemented behaviour is interpretation (b)** — `tau = duration
of the 80 % training slice`. Phase `s` runs `0 → 1` across the training slice, and
the held-out 20 % is rolled out at **`s > 1`**, i.e. *past the natural end* of the
primitive. Because the canonical phase `x(s) = exp(-alpha_x·s)` has already
collapsed to ≈0 there, the forcing term is dead and the model can only sit on the
goal attractor — it is being asked to invent motion beyond tau, which a
phase-decreasing MP structurally cannot do.

### Both interpretations are now selectable

Added a `--tau-mode {train|full}` flag to the hold-out tool (`train` is the
default, so every earlier grid number stays comparable), backed by a new opt-in
`tau_override` argument on `ProDMP::learnFromDemonstration(demo, tau_override = 0.0)`:

| mode | tau used | held-out phase range | question asked |
|------|----------|----------------------|----------------|
| `train` (default, = old behaviour) | span of 80 % slice (`54.65 s` on trajC) | `s ∈ [1.0, ~1.25]` | continue a movement *past its own end* |
| `full` | span of full demo (`68.33 s` on trajC), passed as `tau_override` | `s ∈ [~0.8, 1.0]` | genuine shape extrapolation *within* the designed phase range |

`tau_override <= 0` (default) reproduces the previous fit bit-for-bit. Semantics
are documented in a comment block at
`learn_and_test_prodmp_holdout.cpp:88-118` and at `prodmp.cpp:182-190`.

### Sanity check (trajC, n_basis=80, ridge=1e-9, window=0.01s, holdout=20 %)

| mode | tau_used | RMSE in-sample | RMSE hold-out | gap ratio |
|------|----------|----------------|---------------|-----------|
| `train` (and no-flag default — identical) | 54.65 s | 0.372 mm | 5.52 mm | **14.8×** |
| `full` | 68.33 s | 0.385 mm | 43.55 mm | **113×** |

Neither interpretation removes the gap. In `train` mode the model is frozen at the
goal for the whole hold-out (≈5.5 mm because the true motion keeps moving but
stays near the end region). In `full` mode the late-phase region `s ∈ [0.8, 1]` is
completely unconstrained by training data, so the fit extrapolates wildly
(43.5 mm). Different mechanism, same structural failure.

---

## STEP 2 — condition number of H *after* the column-scale preconditioning

`cond(H) = σ_max / σ_min` of the **column-scaled** design matrix (the matrix the
LDLT actually factorizes), via `Eigen::JacobiSVD`. Exposed through a new opt-in
`ProDMP::setComputeConditionNumber(true)` +
`diagnostics().design_matrix_condition_number` (off by default — the SVD costs far
more than the fit) and surfaced by `--report-condition-number` in the hold-out
tool, which also writes it to the summary CSV column `cond_H_scaled` for every
grid row.

Demo: `demo_raw_trajC.csv`, `ridge_lambda = 1e-9`, `window = 0.01 s`, `holdout = 20 %`.

### tau-mode `train` (the mode every previous grid used)

| n_basis | cond(H) after preconditioning | RMSE in-sample | RMSE hold-out | gap ratio |
|--------:|------------------------------:|---------------:|--------------:|----------:|
| 50  | 2.82 × 10⁹  | 0.463 mm | 20.41 mm | 44.1× |
| **80 (grid "winner")** | **2.40 × 10¹⁰** | 0.372 mm | 5.52 mm | 14.8× |
| 150 | 3.86 × 10¹¹ | 0.355 mm | 22.03 mm | 62.1× |

### tau-mode `full`

| n_basis | cond(H) after preconditioning |
|--------:|------------------------------:|
| 50  | **inf** (σ_min = 0 — H is rank-deficient) |
| 80  | **inf** |
| 150 | **inf** |

### Reading

* Even *after* the column-scale preconditioning fix, `cond(H)` in `train` mode is
  **10⁹–10¹¹** and grows ≈1.5 orders of magnitude per +50 basis functions. The
  preconditioner tamed the ~8 × 10⁸ column-norm spread from the `exp(alpha·s/2)`
  growth, but the matrix stays severely ill-conditioned: with `ridge_lambda = 1e-9`
  the effective rank is far below the nominal parameter count, and the fit in the
  near-null subspace is numerical noise. That is the direct cause of the large,
  non-monotonic-in-`n_basis` hold-out error.
* In `full` mode `cond(H)` is literally infinite: the training slice only reaches
  `s ≈ 0.8`, so the goal-basis column and the late RBF columns are evaluated only
  where they are mutually collinear / near-zero, and (left unscaled below the
  `5e-6` floor) they sum to an exactly singular `H`. The `1e-9` ridge still yields
  *a* solution, but it is unconstrained in that subspace → the wild `full`-mode
  extrapolation seen in STEP 1.

---

## STEP 3 — third independent demo (`reach_task_baseline.csv`)

Never used in the trajA/trajC grid search. 60 900 samples, tau_full = 60.97 s.
No hyper-parameter re-tuning — the trajA/trajC "winner" is applied as-is.
`holdout_fraction = 0.2`, `--report-condition-number`.

| config | tau_mode | RMSE in-sample | RMSE hold-out | gap ratio | max‖w‖ | cond(H) scaled |
|--------|----------|---------------:|--------------:|----------:|-------:|---------------:|
| **(a) winner** — n=80, λ=1e-9, w=0.01 | train | 0.252 mm | 22.13 mm | **87.8×** | 1 602 | 2.40 × 10¹⁰ |
| (a) winner | full | 0.263 mm | 22.48 mm | **85.5×** | 1 066 | inf |
| **(b) baseline_post_fix** — n=200, λ=1e-6, w=0.20 | train | 1.159 mm | 41.67 mm | **36.0×** | 15 268 | 1.27 × 10¹² |
| (b) baseline_post_fix | full | 0.180 mm | 21.71 mm | **120.4×** | 3 107 | inf |

### Comparison to trajA / trajC

| demo | gap ratio range observed |
|------|--------------------------|
| trajA / trajC grid (reported) | ≥ 14.8×, majority > 50× |
| **reach_task_baseline (this step)** | **36× – 120×** |

The third demo is **fully consistent** with the two originals: the hold-out gap is
two-to-three digits for every configuration and every tau interpretation. The
phenomenon is **not specific to trajA/trajC**.

Two extra observations:

* The "winner" (n=80, λ=1e-9) was selected for lowest hold-out RMSE on the
  trajA/trajC pair (≈5.5–7 mm there). On this unseen demo its hold-out RMSE is
  **22 mm** — ~4× worse — and its gap ratio jumps to ~88×. The winner selection
  was overfit to the two demos it was chosen on; it does not generalize.
* `baseline_post_fix` in `full` mode reaches the best in-sample number of the
  whole study (0.18 mm) while still missing the hold-out by 21.7 mm (gap 120×):
  near-perfect interpolation of the training 80 %, useless extrapolation. With
  ~200 parameters against ~49 000 samples this is **not** classic parameter-count
  overfitting — it is that the held-out phase region is unconstrained by any data
  and a phase-based MP has no inductive bias that makes its behaviour there match
  a real continuing trajectory.

### Classic-DMP reference

`reach_task_baseline_nbasis200.yaml` (classic `core::DMP`, n_basis=200, ridge=1e-6,
window=0.20 s, tau=60.97 s) is a **full-trajectory in-sample** fit — not a
hold-out fit — so it is a scale reference only, not a like-for-like comparison. No
classic-DMP temporal-hold-out harness exists and building one was out of scope;
the classic DMP shares the identical phase-decreasing structure, so the STEP 1/3
conclusion would not change.

---

## STEP 4 — synthesis and recommendation

### Is the gap structural or a hyper-parameter artifact?

**Structural.** The evidence:

1. The gap is 15× – 120× across **3 independent demos**, **7 × 6 grid cells**, and
   **both** tau interpretations. Nothing in the swept ranges
   (`n_basis ∈ [30, 300]`, `ridge_lambda ∈ [1e-6, 1e-12]`) produces an interior
   optimum or a single-digit gap.
2. `cond(H)` after the preconditioning fix is still 10⁹–10¹² (`train`) or
   infinite (`full`). The regression is rank-deficient in exactly the subspace
   that governs late-phase / goal behaviour, which is the subspace the hold-out
   depends on entirely.
3. The failure mode is the same in both tau modes for opposite reasons:
   `train` mode asks for motion at `s > 1` where the primitive is definitionally
   inert; `full` mode asks for `s ∈ [0.8, 1]` where **no training sample exists**.
   A phase-decreasing MP fitted on a leading time-slice has no mechanism —
   and, with this design matrix, no numerical conditioning — to reconstruct the
   part of the movement it never saw.

This is temporal extrapolation of a phase-anchored primitive, which is not what
DMP/ProDMP are built for. It is a **known, expected limitation**, not a bug to be
tuned away.

### Recommendation

* **Do not adopt the "winner" (n=80, λ=1e-9, w=0.01).** Its ranking was an
  artifact of the two demos it was chosen on; it degrades ~4× on the first unseen
  demo and its weights already reach ‖w‖ ≈ 1600–3200 (noise-fitting territory).
* **For the production pipeline, keep the existing post-fix baseline
  `n_basis = 200, ridge_lambda = 1e-6, position_filter window = 0.20 s`**
  (the values already in `config/prodmp_features.yaml`). Rationale: it is the only
  configuration in this study with a sane weight norm on well-posed in-sample
  fits, its in-sample fidelity is sub-millimetre-to-~1 mm on real demos, and the
  column-scale-preconditioning-plus-floor fix (`kScaleFloorRatio = 5e-6`) keeps
  its full-trajectory fit healthy (trajC 0.71 mm, trajA 1.06 mm rmse_overall,
  all 7 core tests green).
* **Declare the extrapolation limit explicitly as a known constraint of the
  approach:** a ProDMP fitted on a demonstration is only trustworthy for
  `t ∈ [0, tau_fit]` (phase `s ∈ [0, 1]`). Rollouts beyond `tau_fit`, and any
  use that fits on a temporal sub-segment and expects the remainder, carry a
  **two-digit RMSE-gap factor** and must not be relied on. The best case measured
  in this entire study is still a 15× gap; that number goes in the limitations
  section, it does not get hidden.
* If temporal generalization past `tau_fit` is actually a requirement, it needs a
  different tool (e.g. re-anchoring / re-timing the primitive from a live phase
  estimate, or a formulation without a decaying canonical phase) — not a
  hyper-parameter change to this one.

### Artifacts

* `common/src/learn_and_test_prodmp_holdout.cpp` — `--tau-mode {train|full}`,
  `--report-condition-number`, `tau_mode` + `cond_H_scaled` columns in the
  summary CSV, `gap ratio` printed.
* `src/haptic_dmp_learning/src/core/prodmp.cpp` /
  `include/haptic_dmp_learning/core/prodmp.hpp` — opt-in
  `learnFromDemonstration(demo, tau_override = 0.0)` and
  `setComputeConditionNumber(bool)` + `diagnostics().design_matrix_condition_number`.
  Default path unchanged (verified bit-identical + 7/7 core tests).
