# Guida al codice del selettore S3 (`selectGraspS3`, modalità v2)

Destinatario: Lorenzo (tesi di automazione, spiegazione al tutor). Stato del codice: commit `d6f55f3` (HEAD di `main` all'inizio della sessione; il working tree non è stato modificato). Tutti i riferimenti sono `file:riga` relativi alla radice del workspace `/home/lorenzo/thesis_ws`.

**Convenzioni di lettura**

- Abbreviazioni dei percorsi (tutte sotto `src/satellite_grasp_planner/`):
  - `SEL` = `src/core/grasp_s3_selection.cpp`, `SELH` = `include/satellite_grasp_planner/core/grasp_s3_selection.hpp`
  - `EVAL` = `src/core/grasp_pose_eval.cpp`, `EVALH` = `include/satellite_grasp_planner/core/grasp_pose_eval.hpp`
  - `KF` = `src/core/kinematic_feasibility.cpp`, `KFH` = `include/.../core/kinematic_feasibility.hpp`
  - `JPS` = `src/core/joint_path_simulator.cpp`, `JPSH` = `include/.../core/joint_path_simulator.hpp`
  - `FM` = `src/core/follow_margin.cpp`, `FMH` = `include/.../core/follow_margin.hpp`
  - `CLR` = `src/core/approach_clearance.cpp`, `CLRH` = `include/.../core/approach_clearance.hpp`
  - `LAUNCH` = `src/core/grasp_s3_launch.cpp`, `NODE` = `src/ros/grasp_planner_node.cpp`
- **Esempio guida**: run `results/margin2_w-5.0` (ω = −5 °/s, centro (0.7, 0, 0.77), `s1.mode=v2`, `branch_rule=rotation`, `use_clearance=true`, `use_margin=true`, `finalists_n=40`; vedi `results/margin2_w-5.0/run_meta.json`). Fonti dei numeri: `planner.log`, `report.txt`, `report.json` nella stessa cartella. Vincitore: k=1, ramo A (`wrist_flip=false`), θ\*=−166.0°, d=86.881 mm, w_static=0.03807, w_rollout=0.04369, margine previsto 35.00° (giunto 6, t_rel −24.47 s), clearance prevista 12.46 mm (dito 1–collare, t_rel −8.37 s), `selection_elapsed` 2.454 s.
- Etichette: **[M]** misurato nei log, **[C]** calcolato da me a mano con le formule del codice e i numeri dei log (ricalcolabile), **ASSUNTO** = il codice o lo YAML lo dichiarano assunto, **NON CHIARO** = non si capisce dal codice (raccolti in fondo).
- **Aggiornamento a HEAD `0e51606`**: i numeri di riga e i nomi di questa guida valgono per `d6f55f3`. Da allora: `launch_delay.{hpp,cpp}` è stato fuso in `launch_plan.{hpp,cpp}` (le righe di §1.1, §1.2 e §7 su `launchDelay`/`planLaunch` sono già aggiornate); sono stati tolti `core/manipulability.cpp` e la classe `Manipulability`, i campi `S3Result::max_w_hat_passing`, `S3FinalistRecord::{j_static, clearance_full_mm, rollout_max_violation_rad}` e `FeasibilityResult::{w_trans_final, worst_step, worst_step_fraction, worst_joint_value_rad, worst_joint_limit_rad}`; `selectGraspS3` è stata spezzata in funzioni del namespace anonimo di `grasp_s3_selection.cpp` (`CandidateEvaluator`, `collectCandidates`, `posthocVelocityCheck`, `runLegacyPosthoc`, `selectStaticFinalists`, `fillRankedAndBestPerK`, `finalizeStatus`), per cui i riferimenti `SEL:riga` non corrispondono più al file attuale.
- Le righe di `docs/S3_SPEC.md` (documento esistente) citano numeri di riga di una versione precedente del codice (`sel:131-151` ecc.): non le ho riverificate; per i numeri di riga fa fede questa guida.

---

## 1. Mappa del pacchetto `src/satellite_grasp_planner/`

### 1.1 `include/satellite_grasp_planner/` (header)

| File | Che cosa fa |
|---|---|
| `core/grasp_s3_selection.hpp` | `S3Params` (tutti i parametri `s1.*`), `S3Context`, `S3Candidate`, `S3FinalistRecord`, `S3Result`; dichiara `selectGraspS3`, `s3CostV2`, `s3PickWinnerV2`, `s3ClearanceRound`, `s3Distance`, `s3DPos`. |
| `core/grasp_s3_launch.hpp` | Dichiara `planGraspAndLaunchS1`: selezione + `planLaunch` → `GraspLaunchResult`. |
| `core/grasp_launch_result.hpp` | Struct di uscita (`goal_position`, `launch`, `elapsed_s`, `cost`, …) con stato esplicito `kSelected / kNoFeasibleCandidate / kPosthocFailed`. |
| `core/grasp_pose_eval.hpp` | IK statica (`solvePoseIk`), margini di giunto, `se3Distance`, valutazione di una posa (`evaluateGraspPoseDetailed`), regola del ramo (`BranchRule`). |
| `core/manipulability.hpp` | `wTransFromJacobian` (prodotto dei 3 valori singolari del blocco lineare), `fullManipulability` (`w_trans`, `w_rot`, `w_full`). |
| `core/joint_path_simulator.hpp` | `NullspaceBiasedResolution` / `MinNormDlsResolution` (strategie di ridondanza) e `JointPathSimulator` (integrazione di Eulero esplicita). |
| `core/kinematic_feasibility.hpp` | `buildApproachTrajectory`, `timedApproachTrajectory`, `checkKinematicFeasibility`, `FeasibilityResult`. |
| `core/follow_margin.hpp` | `followMargin` (approccio + inseguimento simulato), `FollowApproachState` per riusare lo stato dell'approccio. |
| `core/approach_clearance.hpp` | Distanza con segno gripper–satellite lungo la traiettoria comandata (`approachClearance`, `ApproachClearanceModel`, loader mesh STL/URDF). |
| `core/launch_plan.hpp` | Attesa prima del lancio (`launchDelay`) e piano di lancio (`planLaunch`, `phaseAt`, `SatelliteSnapshot`). |
| `core/selection_inputs.hpp` | `SelectionInputs` + `buildSelectionInputs`: template ProDMP, robot, q0/p0, Δg demo, `w_trans_demo`, provenienza. |
| `core/start_state.hpp` | Override della configurazione di partenza (`start_joints_override_rad`), `distanceFromReadyPose`. |
| `ros/grasp_planner_node.hpp` | Nodo ROS one-shot. |
| `ros/grasp_command_conversion.hpp` | `GraspLaunchResult` → messaggio `GraspCommand`. |
| `ros/grasp_planner_checks.hpp` | Controlli puri di avvio/primo campione (base robot, assi, centro, `q_ref`, ω). |

### 1.2 `src/`

| File | Righe | Che cosa fa |
|---|---|---|
| `core/grasp_s3_selection.cpp` | 923 | Il selettore: scansione θ, finalisti, rollout, vincoli margine/clearance, scelta del vincitore. |
| `core/grasp_s3_launch.cpp` | 101 | Costruisce `S3Context`, chiama `selectGraspS3`, misura `elapsed_s`, chiama `planLaunch`. |
| `core/grasp_pose_eval.cpp` | 243 | IK statica a iterazioni, margini, `w_hat`, scelta del ramo di polso. |
| `core/kinematic_feasibility.cpp` | 154 | Traiettoria d'approccio ProDMP+SLERP, simulazione, margine minimo di approccio. |
| `core/joint_path_simulator.cpp` | 160 | Strategie di ridondanza e integrazione di Eulero. |
| `core/follow_margin.cpp` | 115 | Margine di giunto su approccio + inseguimento (300 passi). |
| `core/approach_clearance.cpp` | 544 | Geometria coal (mano convessa, dita box, collare 24 box, cubo), distanza con segno, lettura STL. |
| `core/launch_plan.cpp` | 44 | Formula dell'attesa e piano di lancio. |
| `core/selection_inputs.cpp`, `core/start_state.cpp` | 173, 73 | Assemblaggio degli ingressi, validazione di q0. |
| `core/manipulability.cpp` | 19 | Wrapper `Manipulability::wTrans`. |
| `ros/grasp_planner_node.cpp` | 452 | Parametri ROS → `S3Params`, primo campione odom → thread di selezione → pubblicazione. |
| `ros/grasp_command_conversion.cpp` | 72 | Riempie `GraspCommand`. |
| `ros/grasp_planner_checks.cpp` | 96 | Implementazione dei controlli. |
| `ros/grasp_planner_main.cpp` | 19 | `main`: `rclcpp::init`, spin del nodo. |

### 1.3 `config/`, `analysis/`, `test/`

- `config/grasp_planner_example.yaml` (117 righe): esempio di parametri; le righe `# s1.*` sono i default del codice, l'unica attiva nel blocco v2 è `s1.rollout_q_ref` (riga 94).
- `analysis/` (compilati solo con `-DBUILD_ANALYSIS_TOOLS=ON`, `CMakeLists.txt:131-133`): `s3_landscape.cpp` (797 righe, esporta il paesaggio dei candidati in CSV), `s3_clearance.cpp` (clearance di UN candidato), `s3_followmargin_check.cpp` (riproduce i margini di G16 con `followMargin`), `orientation_law_compare.cpp` e `orientation_path_check.cpp` (confronto leggi di orientazione, non usate dal selettore), `orientation_laws.hpp`, `s3_landscape_util.hpp`, `README.md`.
- `test/`: test gtest, uno per modulo (elenco al §9).

### 1.4 Dove sono i parametri `s1.*` e come arrivano al selettore

Percorso: **YAML / riga di comando `-p` → `declare_parameter` nel nodo → `S3Params s3_params_` → `validateS3Params` → `planGraspAndLaunchS1` → `selectGraspS3`**.

- Dichiarazione: `NODE:142-211`. Ogni parametro è dichiarato con il default della struct (`dbl("s1.length_scale_m", p.length_scale_m)`, riga 146 ecc.), quindi ciò che non è nello YAML/CLI resta al default di `S3Params` (`SELH:36-125`).
- Attenzione ai nomi: il parametro ROS `s1.kappa_w` scrive `p.kappa_w_mm` (`NODE:191`), `s1.kappa_t` scrive `p.kappa_t_mm_s` (`NODE:192`).
- Validazione all'avvio: `core::validateS3Params(p)` (`NODE:212`; corpo `SEL:70-156`): scale/passi finiti e > 0, `finalists_n ≥ 1`, `use_clearance`/`use_margin` solo con `mode=v2` (`SEL:106-109, 119-122`), `rollout_q_ref` obbligatorio e di 7 valori in v2 (`SEL:126-134`).
- Avviso: con `mode=v2` e `branch_rule=margin` il nodo avverte che conviene `rotation` (`NODE:213-217`).
- Passaggio al selettore: `NODE:324-326` → `planGraspAndLaunchS1(in, snapshot, min_delay_s_, s3_params_, …)`; in `LAUNCH:63` → `selectGraspS3(model, robot, in.q0, demo, in.delta_g_demo, ctx, s3)`.
- **Esempio (ω −5)**: `run_meta.json` → `planner_param` contiene `"s1.finalists_n": "40"`, `"s1.use_margin": "true"`, `"s1.use_clearance": "true"`, `"s1.mode": "v2"`, `"s1.branch_rule": "rotation"`, `"s1.rollout_q_ref": "[0.0,-0.785398163397,…]"`. Nel `planner.log` il nodo riscrive i valori effettivi: `finalists_n=40`, `kappa_w=7.500 mm`, `kappa_t=0.000 mm/s`, `tau_j=2.000 mm` (riga di log emessa da `NODE:315-321`).

---

## 2. Flusso a runtime

Dal messaggio con lo stato del satellite alla pubblicazione del `GraspCommand`:

| # | Funzione (file:riga) | Riceve → restituisce |
|---|---|---|
| 1 | `GraspPlannerNode::onOdom` (`NODE:266`) | `nav_msgs/Odometry` del satellite. Gestisce **solo il primo campione**: `if (dispatched_.exchange(true)) return;` (riga 267). |
| 2 | `checkFirstOdomSample` (riga 272), `PhaseTracker::update` (275-277), `checkOmegaConsistency` (281) | posizione, quaternione, twist → fase `phase.theta_rad`; verifica che ω misurata coincida col parametro. |
| 3 | **Snapshot** (`NODE:284-289`) | `SatelliteSnapshot{center, axis, omega_rad_s (da parametro, riga 287), theta_rad = phase.theta_rad (288), t_s = header.stamp (289)}`. È qui che si "prende lo snapshot". |
| 4 | `std::thread(&runSelection, snapshot)` (riga 291) | La selezione gira fuori dal thread di callback. |
| 5 | `buildSelectionInputs` (`NODE:300`; corpo `selection_inputs.cpp:49-155`) | file demo + URDF + snapshot → `SelectionInputs` (ProDMP template, robot, `q0`, `p0`, `delta_g_demo`, `contact_to_end_offset`, `tau_contact_s`, `w_trans_demo`). **Non** rientra nel tempo di selezione (vedi punto 7). |
| 6 | `planGraspAndLaunchS1` (`NODE:324`; `LAUNCH:15`) | controlli di coerenza snapshot/ingressi (`LAUNCH:20-35`), `tau_launch = contact_assumed_at_end ? tau del template : tau_contact_s` (`LAUNCH:32`). |
| 7 | **Durata della selezione** | `t_start = steady_clock::now()` (`LAUNCH:37`) … `elapsed_s = now − t_start` (`LAUNCH:64`), cioè costruzione di `CubeSatelliteModel` + `robot.update` + `selectGraspS3`; `out.elapsed_s = elapsed_s` (`LAUNCH:69`). Poi `GraspCommand.selection_elapsed_s = result.elapsed_s` (`grasp_command_conversion.cpp:56`). Esempio: 2.453902782 s (`report.json`). |
| 8 | `selectGraspS3` (`SEL:269-920`) | `S3Result` (vincitore, finalisti, contatori). Dettaglio al §3. |
| 9 | `planLaunch` finale (`LAUNCH:84-85`) | `t_now_s = snapshot.t_s + elapsed_s` (riga 84) → `LaunchPlan{delay_s, launch_time_s, contact_time_s, phase_at_contact_rad}`. È il ritardo "provvisorio". |
| 10 | `toGraspCommandMsg` (`NODE:439`; `grasp_command_conversion.cpp:8-59`) | `GraspLaunchResult` + snapshot + `now` → `GraspCommand` con `provisional_delay_s = launch->delay_s` (riga 37). |
| 11 | Attesa clock (`NODE:423-434`) e `pub_->publish(msg)` (riga 440) | Il nodo attende (max 5 s reali) che l'orologio del nodo raggiunga `snapshot.t_s`; poi pubblica **un solo** messaggio (QoS reliable, transient_local, depth 1: `NODE:250-251`) e resta inattivo. |

**Cosa fa davvero partire il braccio.** Il ritardo `provisional_delay_s` è solo diagnostico: l'executor lo ignora e scatta sull'incrocio della fase θ_trig = wrapPi(θ\* − ω·τ) (`src/haptic_dmp_learning/include/haptic_dmp_learning/core/satellite_intercept.hpp:596-600, 676`, `InterceptTrigger` righe 249-286; commento in `grasp_planner_node.hpp:26-29` e in `satellite_intercept.hpp:555`). Dettaglio al §7.

**Esempio (ω −5).** Snapshot: θ = 13.115° a t = 20.765 s; `GraspCommand` pubblicato a t = 23.222 s [M, `report.txt`]. Selezione 2.454 s [M]. Ritardo provvisorio 8.895 s [M], misurato pub→TRIGGER 8.893 s [M].

---

## 3. `selectGraspS3`, passo per passo (modalità v2)

### 3a. Ingressi e parametri

**Ingressi di `selectGraspS3`** (`SELH:322-324`): `CubeSatelliteModel` (geometria cubo/collare, `graspPoseAt(k, θ)`), `RobotModel& robot`, `q_start`, `DemoReference demo`, `delta_g_demo`, `S3Context ctx`, `S3Params params`.

`S3Context` (`SELH:160-173`):

| Campo | Tipo | Significato |
|---|---|---|
| `prodmp_template` | `const ProDMP*` | Template ProDMP appreso, obbligatorio (`SEL:35`); mai modificato (si lavora su copie). |
| `contact_to_end_offset` | `Vector3d` [m] | Obiettivo = posizione di presa + offset (dalla demo). Nel run vale (0,0,0) [M, `planner.log`]. |
| `tau_contact_s` | double [s] | ≤0 ⇒ usa tau del template; serve al controllo di velocità. |
| `tau_launch_s` | double [s] | Deve essere > 0 (`SEL:36`); è la tau passata a `planLaunch`. Nel run: 24.474 s. |
| `snapshot` | `SatelliteSnapshot` | ω, centro, asse, θ e t dell'acquisizione. |
| `t_now_s` | double [s] | Istante in cui si calcola l'attesa dei candidati: `t_now_override.value_or(snapshot.t_s)` (`LAUNCH:53`). |
| `min_delay_s` | double [s] | ≥ 0 (`SEL:38`). |
| `eval_log` | `vector<S3EvalRecord>*` | Log opzionale per-candidato (nullptr = spento). |
| `clearance_eval` | `ClearanceEvaluator` | Obbligatorio se `use_clearance` (`SEL:293-295`); costruito in `LAUNCH:56-61`. |

**`S3Params`** (`SELH:36-125`; default nel codice; ROS: `NODE:146-211`). "Usato nel run" = valore in `results/margin2_w-5.0/planner.log` dove stampato, altrimenti default.

| Campo | Tipo | Default | Unità | Significato | Marcatura |
|---|---|---|---|---|---|
| `length_scale_m` | double | 0.10 | m | scala ℓ della distanza | — |
| `orientation_weight` | double | 0.0 | – | peso dell'orientamento in `d` (0 = solo posizione) | — |
| `angle_scale_rad` | double | 1.0 | rad | scala angolare | — |
| `epsilon_wait_per_s` | double | 1e-4 | 1/s | peso dell'attesa nel costo `d + ε·wait` (legacy; in v2 serve a ordinare a parità: `SEL:335`, `lessCost` `SEL:62-66`) | — |
| `w_hat_min` | double | 0.5 | – | soglia su `ŵ = w_trans/w_trans_demo` (solo legacy, `SEL:172`) | origine non indicata nel codice |
| `margin_min_rad` | double | 0.3 | rad | soglia margine di giunto statico (solo legacy, `SEL:173`) | origine non indicata |
| `r_trust_m` | double | 0.30 | m | raggio di fiducia su `d_pos` (legacy e v2) | origine non indicata |
| `delta_v_max_m_s` | double | 3e-3 | m/s | soglia velocità relativa post-hoc | origine non indicata |
| `theta_step_deg` | double | 2.0 | deg | passo della griglia grossa | — |
| `theta_refine_step_deg` | double | 0.1 | deg | passo del raffinamento | — |
| `ik` (`IkParams`) | struct | `pos_tol_m 1e-4`, `ori_tol_rad 1e-3`, `max_iter 300`, `damping 0.05`, `nullspace_gain 1.0`, `max_step_rad 0.2` (`EVALH:27-34`) | m, rad, – | parametri dell'IK statica | — |
| `posthoc_path_check` | bool | true | – | controllo percorso post-hoc (ignorato in v2: il rollout è il controllo) | — |
| `posthoc_velocity_check` | bool | true | – | controllo di velocità sul vincitore | usato nel run: **false** (`tools/s1_run.py:550`) [dal runner; nel log: `dv=0`] |
| `max_posthoc_candidates` | int | 10 | – | massimo numero di controlli post-hoc | — |
| `posthoc_cluster_deg` | double | 10.0 | deg | cluster di θ per i finalisti (v2) e per il post-hoc (legacy) | — |
| `force`, `force_k`, `force_theta_deg`, `force_psi_deg`, `force_ignore_constraints`, `force_flip` | bool,int,double,double,bool,int | false,0,0,0,false,−1 | deg | modalità di test: valuta UN solo candidato (`SEL:373-392`) | — |
| `branch_rule` | string | `"margin"` | – | regola del ramo di polso: `margin` o `rotation` | YAML: ASSUNTO (`config:86`); usato nel run: `rotation` |
| `mode` | string | `"legacy"` | – | `legacy` o `v2` | YAML: ASSUNTO; run: `v2` |
| `max_start_goal_deg` | double | 180.0 | deg | scarta se l'angolo ramo scelto–FK(q_start) lo supera (180 = off) | YAML: ASSUNTO |
| `finalists_n` | int | 12 | – | numero massimo di finalisti | YAML: ASSUNTO; run: **40** |
| `rollout_q_ref` | vector<double> | vuoto (obbligatorio in v2) | rad | target nullspace del rollout, 7 valori | YAML: DERIVATO (= `readyPose`, `config:94`); run: `[0,−0.785398,0,−2.35619,0,1.5708,0.785398]` |
| `use_w` | bool | true | – | false ⇒ `J = d_mm (+ κ_t·t_wait)` | YAML: ASSUNTO |
| `kappa_w_mm` | double | 7.5 | mm | peso di `−ln w_full` | codice: **ASSUNTO** (`SELH:92`); run 7.500 |
| `kappa_t_mm_s` | double | 0.0 | mm/s | peso dell'attesa in J | YAML: ASSUNTO; run 0.000 |
| `tau_j_mm` | double | 2.0 | mm | banda di spareggio su J | codice: **ASSUNTO** (`SELH:94`); run 2.000 |
| `use_clearance` | bool | false | – | vincolo di clearance (solo v2) | YAML: ASSUNTO; run true |
| `min_clearance_mm` | double | 2.0 | mm | soglia di clearance | codice: **ASSUNTO** (`SELH:104`); run 2.000 |
| `clearance_dt_s` | double | 0.05 | s | passo di campionamento della finestra | YAML: DERIVATO (predittore a 20 Hz); run 0.050 |
| `gripper_open_m` | double | 0.060 | m/dito | apertura delle dita; deve coincidere con l'executor | YAML: DERIVATO; run 0.0600 |
| `use_margin` | bool | false | – | vincolo di margine previsto (solo v2); attiva il percorso lazy se `!force` | YAML: ASSUNTO; run true |
| `min_margin_deg` | double | 24.0 | deg | soglia di margine previsto | codice: **ASSUNTO** (`SELH:119`: i dati G16 hanno fallimenti a 13.26–20.84° e riusciti a 28.63–35.00°) |
| `margin_follow_s` | double | 3.0 | s | durata del follow simulato dopo AT_END | YAML: DERIVATO (`post_grasp_follow_s`); run 3.000 |
| `margin_follow_dt_s` | double | 0.01 | s | passo del follow | YAML: DERIVATO; run 0.0100 |

Campi di sola uscita (non parametri) al §3h.

### 3b. Valutazione statica

**Griglia di θ** (`SEL:367-370, 394-415`):

```cpp
const double step = params.theta_step_deg * kPi / 180.0;
const double refine = params.theta_refine_step_deg * kPi / 180.0;
const int n_theta = static_cast<int>(std::ceil(2.0 * kPi / step - 1e-9));  // [-pi, pi)
const int n_refine = static_cast<int>(std::floor(step / refine + 1e-9));
...
for (int i = 0; i < n_theta; ++i) {
    const double theta = -kPi + i * step;
    S3Candidate c = evalOne(k, theta, false, 0.0, -1);
```

- Estremi: θ_i = −π + i·2°, i = 0…179, cioè [−180°, +178°] (il +180° non c'è).
- k ∈ {P0, P90, P180, P270} (`kAllK`, `SEL:29-30`), cioè quattro punti di presa sul collare a φ_k = 0/90/180/270° (`cube_satellite_model.hpp:10`).
- **Come si arriva a 880** [C, confermato da `evaluations=880` nel log]: `n_theta = ceil(2π/step − 1e-9) = 180` ⇒ 4×180 = **720** valutazioni grosse. Raffinamento (`SEL:408-414`): `n_refine = floor(2.0/0.1 + 1e-9) = 20`, indici j = −20…+20 con j ≠ 0 ⇒ 40 per k ⇒ 4×40 = **160**. Totale 720+160 = **880**.
- **L'880 è un massimo**: il raffinamento di un k avviene solo se quel k ha almeno un candidato accettato nella griglia grossa (`if (!have_best) continue;`, `SEL:408`). Il numero effettivo è 880 − 40 per ogni k senza accettati. Nel run `evaluations=880` ⇒ tutti e quattro i k avevano almeno un accettato (deduzione da [M]).
- Il raffinamento è centrato sul **migliore per `lessCost`** del k (`SEL:403-405, 409`), cioè per `d + ε·wait` (`SEL:62-66, 335`), non per `d` né per J. Per questo i finalisti #1 (θ=145.3°, k2) e #2 (θ=−163.2°, k0) hanno θ non multipli di 2°: vengono dal raffinamento a 0.1°.
- j = ±20 cade a θ_best ± 2°, già presente nella griglia grossa (coincidenza fino all'arrotondamento): 8 valutazioni duplicate (annotato anche in `docs/S3_SPEC.md` §1.3). Non cambia i finalisti perché il raggruppamento per cluster (§3e) le assorbe.

**Valutazione di un candidato** (`evalOne`, `SEL:304-365`):

1. `evaluateGraspPoseDetailed(model, k, θ, ψ=0, robot, q_start, demo, ℓ, θ_s, ik, need_extra, flip=−1, branch_rule)` (`SEL:308-310`, `EVAL:149-230`). `need_extra = logging || v2` (riga 307): in v2 si calcola `w_full` per entrambi i rami (`EVAL:217-225`).
2. `pose = model.graspPoseAt(k, θ)` (`EVAL:171`): posizione `center + R(θ)·p_body`, con `p_body = (side/2 + standoff − depth)·n + grasp_radius·r_k`; orientamento con `z_tcp = −R n`, `y_tcp = −R r_k` (`cube_satellite_model.cpp:54-74`).
3. Due rami di polso: `orient[0]` = orientamento nominale, `orient[1]` = nominale ruotato di π attorno a `z_tcp` (`EVAL:173-177`). Per ciascuno `solvePoseIk(robot, p, orient[i], q_start, q_start, ik)` (`EVAL:187`): seme e riferimento nullspace = `q_start`.
4. **IK statica** (`EVAL:53-101`): iterazione `q ← clamp(q + dq)` con `dq` di `NullspaceBiasedResolution` (kp 0.5, saturazioni disabilitate, `EVAL:57-65`), passo massimo `max_step_rad` (riga 92), fino a `pos_err < 1e-4 m` e `ori_err < 1e-3 rad` oppure 300 iterazioni. `converged` = limiti rispettati e tolleranze raggiunte (riga 98).
5. **Regola del ramo** `branch_rule=rotation` (`better()`, `EVAL:32-43`): (i) vince il ramo che converge; (ii) se convergono entrambi e le rotazioni differiscono più di `kRotationTieRad = 1e-3 rad`, vince il ramo con **angolo geodetico minore tra la sua orientazione obiettivo e `demo.orientation`**; (iii) altrimenti regola del margine (tolleranza 1e-3 rad sul margine statico, poi `‖q−q_start‖` minore). `demo.orientation` è FK(`q_start`) (`LAUNCH:41-44`). L'angolo è `branch_rot_rad[i]` (`EVAL:179-183`).
   - **Esempio (ω −5)**: per il vincitore `start_goal_deg[A]=90.71`, `start_goal_deg[B]=91.01` [M, `planner.log`]. Differenza 0.30° = 5.2 mrad > 1 mrad ⇒ vince il ramo A (`wrist_flip=false`) [C].
6. `d_pos`, `d` (§3c), poi la **maschera di scarto v2** `s3AprioriRejectMaskV2` (`SEL:184-193`):

```cpp
if (!eval.feasible_basic) mask |= kRejectIk;                    // IK non convergente o fuori limiti
if (!(d_pos_m <= params.r_trust_m)) mask |= kRejectRTrust;      // d_pos > 0.30 m
if (params.max_start_goal_deg < 180.0 && eval.dist_to_demo.ori_rad*180/pi > params.max_start_goal_deg) mask |= kRejectStartGoal;
```

   Un candidato è **accettato** se `mask == 0` (`SEL:331-336`): allora `passes_constraints = true`, `wait_s = planLaunch(snapshot, t_now, θ, tau_launch, min_delay).delay_s` (riga 334), `cost = d + ε·wait` (riga 335). La ragione di scarto registrata è la prima nell'ordine `ik, w_hat, margine, r_trust, start_goal` (`SEL:321-331`).
   - **In v2 `w_hat_min` e `margin_min_rad` NON sono applicati** (`SELH:75-77`, riga di log `NODE:318`: "w_hat_min and margin_min_rad NOT applied"). Nel run `max_start_goal_deg=180` ⇒ la condizione `start_goal` è disattiva; restano IK e `r_trust`.
   - Le soglie legacy sono in `s3AprioriRejectMask` (`SEL:169-176`): `ik`, `!(w_hat >= 0.5)`, `!(min_margin >= 0.3 rad)`, `!(d_pos <= 0.30)`.
7. Tutti gli accettati vanno in `accepted` (`SEL:402, 413`) e, a fine scansione, `std::stable_sort(accepted, lessCost)` (`SEL:417`).

### 3c. Grandezze per candidato

- **`d_pos`** (`s3DPos`, `SEL:263-267`): `goal = position_world + contact_to_end_offset; d_pos = ‖(goal − p0) − Δg_demo‖` [m], con `p0 = FK(q_start)` (`SEL:288`). Misura quanto la presa dista dalla deformazione della singola demo.
- **`d`** (`s3Distance`, `SEL:158-167`): `d = sqrt( (d_pos/ℓ)² + w_o·(ori/θ_s)² )`, adimensionale; con `orientation_weight = 0` è `d_pos/ℓ`. Poi `d_mm = d·ℓ·1000` (`SEL:515`, commento: "= d_pos in mm when orientation_weight = 0").
- **`w_hat`** (`EVAL:196-197`): `ŵ = w_trans(q\*) / w_trans_demo`, dove `w_trans = ∏ σ_i(J_t)` è il prodotto dei **3 valori singolari del blocco lineare 3×7** (`manipulability.hpp:51-59`) e `w_trans_demo` è lo stesso indice alla configurazione finale della demo (`selection_inputs.cpp:124-127`). È normalizzata (adimensionale).
- **`w_full`** (`manipulability.hpp:67-93`): prodotto dei **6 valori singolari del Jacobiano completo 6×7** = `sqrt(det(J Jᵀ))`; righe 0–2 lineari [m], 3–5 angolari [rad]; **non normalizzata** e con unità miste (scalare le righe angolari di c moltiplica `w_full` per c³, commento `manipulability.hpp:62-66`).
- **Differenza esatta `w_hat` / `w_full` e dove sono usate**:
  - `w_hat` = `w_trans`(3 sv, blocco lineare)/`w_trans_demo`; usata **solo** nel vincolo legacy `ŵ ≥ w_hat_min` (`SEL:172`) e come grandezza informativa/di log (`max_w_hat_passing` `SEL:333`, `GraspCommand.w_hat`, `report.txt: w_hat=0.9474`).
  - `w_full` (6 sv, Jacobiano completo, non normalizzata) è usata **solo nel costo v2** (`s3CostV2`, `SEL:199`), valutata (a) alla soluzione IK statica (`w_static`, `SEL:313`) per ordinare i finalisti e (b) a fine rollout (`w_rollout`, `SEL:577`) per il J finale/log.
  - Nessun modo del codice usa insieme vincolo su `w_hat` e costo con `w_full` (legacy: costo senza w; v2: costo con `w_full` senza vincolo su `w_hat`). Vedi l'appendice A.
- **Margine statico** (`jointMargins`, `EVAL:103-110`): `margin_j = min(q_j − lo_j, hi_j − q_j)` [rad], `min_margin` = minimo sui 7 giunti, alla soluzione IK del ramo scelto. In v2 è solo informativo (`report.txt: margine giunti min=47.207 deg`, cioè `GraspCommand.min_joint_margin_rad`, `LAUNCH:95`); **non** è il margine "previsto" del vincolo (§4–5).
- **Esempio (ω −5)**: `d_pos = 86.8809 mm` [M, report] ⇒ `d = 0.868809`, `d_mm = 86.881`. `ŵ = 0.9474` e `w_trans_demo = 0.184282343` [M] ⇒ `w_trans(q\*) ≈ 0.17459` [C]. `w_static = 0.03807` e `w_rollout = 0.04369` [M, log]: sono `w_full` (6 sv), perciò numericamente piccoli e non confrontabili con `ŵ`.

### 3d. Costo

- **`s3CostV2`** (`SEL:195-202`):

```cpp
double s3CostV2(double d_mm, double w_full, double t_wait_s, const S3Params& params) {
    double w_term = 0.0;
    if (params.use_w && params.kappa_w_mm > 0.0) {
        if (!(w_full > 0.0)) return std::numeric_limits<double>::infinity();
        w_term = params.kappa_w_mm * (-std::log(w_full));
    }
    return d_mm + w_term + params.kappa_t_mm_s * t_wait_s;
}
```

  Formula: **J = d_mm + κ_w·(−ln w_full) + κ_t·t_wait**, unità: mm + mm·(adim.) + (mm/s)·s = mm. `w_full ≤ 0` o NaN ⇒ +∞ (un candidato singolare non batte mai uno regolare). `ε` e `ℓ` non compaiono in J: `ℓ` entra in `d_mm`; `ε` solo in `cost = d + ε·wait` usato per spareggi.
- **`s3CostLegacy`: non esiste come funzione** (`grep` nel codice). Il costo legacy è scritto in linea: `c.cost = c.d + params.epsilon_wait_per_s * c.wait_s;` (`SEL:335`; anche `SEL:351, 384`), con `d` adimensionale (`d_pos/ℓ`) e `ε` in 1/s: `cost = d + ε·wait`. Il legacy non ha termine in `w`.
- **Due J, due momenti** (non confonderli):
  - `j_static = s3CostV2(d_mm, w_full_static, 0.0, params)` (`SEL:516`): attesa passata come 0.0; serve **solo a ordinare e scegliere i finalisti** e a stabilire l'ordine lazy.
  - `j = s3CostV2(d_mm, w_rollout, t_wait_s, params)` (`SEL:579`): è quello che il log stampa come `J=` e che diventa `cand.cost` (`SEL:580-581`, `GraspCommand.cost`).
- **Esempio (ω −5), κ_w = 7.5 mm, κ_t = 0** [C]:
  - `J_static(#20) = 86.881 + 7.5·(−ln 0.03807) = 86.881 + 7.5·3.2683 = 111.393`
  - `J(#20) = 86.881 + 7.5·(−ln 0.04369) = 86.881 + 7.5·3.1306 = 110.361`, uguale al `J=110.361` del log e al `cost=110.360752` di `report.txt`.
  - Costo legacy dello stesso candidato (solo per confronto): `0.868809 + 1e-4·11.349 = 0.86994` [C].

### 3e. Ordinamento e costruzione dei finalisti

Codice `SEL:506-536`:

```cpp
std::stable_sort(order.begin(), order.end(), [&](const StaticItem& a, const StaticItem& b) {
    if (a.j_static != b.j_static) return a.j_static < b.j_static;
    return lessCost(accepted[a.idx], accepted[b.idx]);
});
std::vector<StaticItem> groups;
for (const StaticItem& it : order) {
    if (groups.size() >= static_cast<std::size_t>(params.finalists_n)) break;
    ... // duplicato se stesso k, stesso wrist_flip e thetaDistRad < cluster_rad
    if (!dup) groups.push_back(it);
}
```

- Ordine per `j_static` crescente (riga 518-521); a parità `lessCost` (cioè `d + ε·wait`, poi k, poi θ: `SEL:62-66`).
- **Raggruppamento**: si scorre in ordine di J statico e si tiene un candidato solo se non esiste già un rappresentante con **stesso k, stesso ramo (`wrist_flip`) e distanza circolare in θ < `posthoc_cluster_deg` (10°)** (`SEL:527-534`). Il rappresentante è quindi il migliore per J statico del suo gruppo.
- Si ferma a `finalists_n` gruppi (riga 524). Nel run: 40; i finalisti sono in `groups`, in ordine di J statico.
- **Conseguenza**: tutto ciò che sta oltre il N-esimo gruppo non viene mai valutato; vedi §3g.
- **Esempio (ω −5)**: i primi 20 finalisti hanno J_static crescente da 26.965 (#1) a 111.393 (#20) [C]; l'ordine per `w_rollout` sarebbe 1…14, 16, 15, 17, 18, 20, 19 [C], ma il codice usa quello statico.

### 3f. I due percorsi

**Definizione**: `lazy = use_margin && !params.force` (`SEL:292`). `use_clearance` da solo **non** rende lazy il percorso (rollout fatto su tutti i finalisti), si veda sotto.

#### Percorso LAZY (`use_margin && !force`) — `SEL:654-743`

Il vincitore è il **primo finalista ammissibile in ordine di J statico**. Pseudocodice con righe:

```
602-609  per ogni g in groups (ordine J statico): rec = makeRecord(g); NON si fa il rollout (if (!lazy) saltato)
654      if (lazy):
657        for fi = 0 .. fin.size()-1  while chosen_idx < 0:
660          runRollout(rec)                        # rollout + w_rollout + rec.j
661-664      if !rollout_feasible: discard "rollout"; continue
666          runMargin(rec)                          # followMargin su [start, AT_END+3 s]
667-670      if margin_rejected (margin_deg < 24): discard "margin"; continue
671-676      if use_clearance: clearanceOf(fi); if clearance_rejected (<2 mm): discard "clearance"; continue
678          any_admissible = true
679-683      if max_posthoc_candidates == 0:  chosen = rec; winner; break
684          if posthoc_checks >= max_posthoc_candidates: break      # esce senza vincitore
685          ++posthoc_checks
691-709      if posthoc_velocity_check and dv > delta_v_max: discard "velocity"; continue
715-718      chosen_idx = rec.accepted_index; rec.winner = true      # USCITA DAL CICLO (condizione del for: chosen_idx < 0)
720-741    se nessun vincitore e almeno un rollout ok ma nessun ammissibile: clearance_no_candidate = true, nota (kNoCandidate)
```

- **Condizione di uscita**: `chosen_idx >= 0` (riga 657, nel `for`), impostata alle righe 680 o 715; oppure `break` alla riga 684 (limite dei controlli post-hoc).
- Il J con `w_rollout` (`rec.j`) è calcolato (riga 579) ma **non è usato in nessuna condizione** del ciclo (righe 657-719). `w_rollout` quindi non cambia quale finalista vince: cambia al più il valore stampato e `cost`/`ranked`.
- La banda `tau_j` **non si applica**; lo dichiara anche il log iniziale `NODE:218-224` ("the wait tie-break (tau_j_mm) is not applied").
- I finalisti dopo il vincitore non vengono valutati ("evaluated: 20 of 40", `NODE:343-344`).
- Ordine dei vincoli per finalista: rollout → margine → clearance → velocità. Un finalista scartato per margine non arriva alla clearance (log: `clearance=n/v DISCARDED(margin)`).

#### Percorso NON lazy (nessun `use_margin`, oppure `force`) — `SEL:602-609, 744-827`

Vince il **J con `w_rollout`**, con spareggio sull'attesa entro `tau_j`:

```
602-609  per ogni g: rec = makeRecord(g); runRollout(rec) per TUTTI i finalisti (if (use_margin) runMargin(rec) solo se force)
612-613  alive[i] = rollout_feasible
744      while (!lazy && chosen_idx < 0):
745-753    if use_clearance && !force:
             s3ClearanceRound(js_all = fin[i].j, alive, clr_done, clr_mm, clearanceOf, min_clearance, tau_j)
754-761    pool = { i : alive[i] };  js = fin[i].j ; ts = fin[i].t_wait_s
762        pick = s3PickWinnerV2(js, ts, tau_j)
763        if pick < 0: break
767-786    (force) clearance/margine calcolati solo come WARN
787-791    if max_posthoc_candidates == 0: scelto; break
792-793    if posthoc_checks >= max: break; ++posthoc_checks
799-817    se dv > delta_v_max: alive[fi] = 0; continue      # si riprova col prossimo
823-824    chosen_idx = rec.accepted_index; winner; esce dal while (chosen_idx >= 0)
```

- **`s3PickWinnerV2`** (`SEL:204-221`): `j_min` = minimo dei J finiti; poi, tra gli indici con `J ≤ j_min + tau_j` sceglie il **`t_wait` minimo**, a parità `J` minore (riga 216). Restituisce −1 se nessun J finito.
- **`s3ClearanceRound`** (`SEL:223-261`): ordina gli `alive` con J finito per J crescente (riga 236); valuta la clearance in ordine fino al primo con `clearance ≥ min_clearance` (righe 248-254), il cui J è `J*`; poi valuta anche tutti gli altri vivi con `J ≤ J* + tau_j` (righe 256-259), cioè la **stessa banda** di `s3PickWinnerV2`; i bocciati hanno `alive = 0`. Restituisce false se nessuno è ammissibile. NaN conta come violazione (riga 242). Cache: `done[]`, `clearance_mm[]` (la funzione di valutazione gira al più una volta per finalista).
- **`tau_j`**: banda di 2 mm (ASSUNTO) su J entro cui l'attesa minore decide. Si applica **solo** nel percorso non lazy, anche con `kappa_t = 0` (con `κ_t = 0` l'attesa è fuori da J, ma lo spareggio sull'attesa resta). Esempio con i numeri del run [C]: #19 e #20 differiscono di 0.25 mm in J ⇒ sarebbero nella stessa banda; il vincitore sarebbe quello con attesa minore (#20: 11.349 s; #19: 20.949 s [M]) tra i vivi dopo la clearance.
- **Con la sola `use_clearance`** (`use_margin=false`): è non lazy; il rollout è fatto su tutti i finalisti (righe 604-607), la clearance è valutata pigramente da `s3ClearanceRound` e `tau_j` si applica.
- **Con `force`**: un solo finalista; margine e clearance non scartano, producono solo avvisi (`SEL:767-786`, `SEL:606`).

### 3g. `kNoCandidate`, `force`, `finalists_n` piccolo

- **kNoCandidate** (`SEL:892-900`): (a) `clearance_no_candidate` (lazy: righe 720-741; non lazy: 829-845), con `note` che descrive quanti sono stati scartati per margine/clearance e il migliore; (b) `accepted.empty()` ⇒ "no candidate satisfies the v2 constraints (ik, d_pos <= r_trust, start_goal <= max_start_goal_deg)" (riga 898). **Nessun fallback**: se nessun finalista è ammissibile non si guarda oltre.
- **kPosthocFailed** (`SEL:901-914`): `accepted` non vuoto ma nessuno scelto (ad es. finalisti non fattibili al rollout, o raggiunto `max_posthoc_candidates`).
- **kSelected** (`SEL:915-917`): `res.chosen = accepted[chosen_idx]`.
- **`force`** (`SEL:373-392`): non fa le scansioni; valuta `(force_k, force_theta_deg, force_psi_deg)` con `force_flip`. Se viola i vincoli a priori: `kNoCandidate`, a meno di `force_ignore_constraints` (viene scelto e le violazioni sono in `forced_violations`, `SEL:381-386, 846-850`). In v2 con force: `lazy=false`, un solo finalista, margine/clearance solo avvisi.
- **`finalists_n` piccolo**: i finalisti sono i migliori `N` gruppi per J statico (`SEL:524`). Se tutti vengono scartati dai vincoli, l'esito è `kNoCandidate` anche se candidati ammissibili esistono oltre l'N-esimo; il codice non estende l'elenco. Esempio: nel run, i 19 finalisti prima del #20 sono tutti scartati; con `finalists_n < 20` il risultato sarebbe `kNoCandidate` [C, per costruzione del ciclo]. Il default del codice è 12 (`SELH:87`); il run usa 40.

### 3h. Risultato (`S3Result`, `S3FinalistRecord`) e log

`S3Result` (`SELH:243-263`): `status` (`kSelected/kNoCandidate/kPosthocFailed`), `chosen`, `ranked` (primi 20), `best_per_k[4]`, `evaluations`, `max_w_hat_passing`, `posthoc_checks`, `posthoc_dv_m_s`, `posthoc_min_margin_rad`, `posthoc_log`, `note`, `forced`, `forced_violations`, `v2_finalists` (finalisti in ordine di J statico), `clearance_stage_ms`, `warnings`, `rollouts_done`, `margin_evals_done`.

`S3FinalistRecord` (`SELH:189-226`): `cand`, `accepted_index`, `d_mm`, `w_static`, `j_static`, `rollout_done/feasible`, `w_rollout`, `j`, `t_wait_s`, `start_goal_deg`, `rollout_pos_err_mm`, `rollout_ori_err_deg`, `rollout_max_violation_rad`, `rollout_min_margin_rad`, `rollout_end` (+`rollout_end_valid`), `winner`, `clearance_done/rejected/mm/pair/t_rel_s/full_mm`, `margin_done/rejected/deg/joint/t_rel_s`, `discard_reason` ("rollout", "margin", "clearance", "velocity"; vuoto in non lazy).

Il log del nodo (tutte le righe e i campi) è al §8.

---

## 4. Rollout: `checkKinematicFeasibility` e `JointPathSimulator`

**Chiamata nel selettore** (`runRollout`, `SEL:554-583`): `checkKinematicFeasibility(*ctx.prodmp_template, c.eval.pose.position_world, orient, robot_ptr, q_start, kDt=0.005, kRolloutViolTolRad=1e-4, q_rollout_ref)`.

1. **Traiettoria** (`buildApproachTrajectory`, `KF:15-41`): copia del template ProDMP; `setRelativeGoal(false)`, `setInitialConditions(0, p0, 0)`, `setGoal(target_position)` (righe 25-28); `n_steps = ceil(tau/dt)`; per i = 0…n−1: `position = prodmp.step(dt)`, `orientation = slerpShortestPath(quat0, target_orientation, s)` con `s = min(1, (i+1)·dt/tau)` (righe 32-38). Il campione i appartiene al tempo `(i+1)·dt`. `p0` e `quat0` = FK(`q0`) (`KF:83-86`).
   - Esempio (ω −5): tau = 24.474 s, dt = 0.005 ⇒ `ceil(24.474/0.005) = 4895` campioni [C].
   - Differenza: la traiettoria è solo Cartesiana ideale (posizione ProDMP + SLERP); non è quella dell'executor, che per l'orientazione usa `QuaternionDMP` (secondo `docs/S3_SPEC.md` §6 D11; non l'ho verificato qui).
2. **Cinematica** (`JointPathSimulator::simulate`, `JPS:125-157`): per ogni campione: `robot.update(q)`; registra `Step{q, w_trans}`; `dq_cmd = strategy.computeJointVelocity(model, target, q, q_ref)`; `q += dq_cmd·dt` (Eulero esplicito, **nessun clamp ai limiti**: le violazioni si misurano, non si impediscono). `q_after_last_step` salvato alla fine (riga 155).
3. **Strategia** (`NullspaceBiasedResolution`, `JPS:46-80`, parametri di default `KF:91`):

```
twist  = desiredTwist(err, ff)      # v = v_ff + Kp·e, con saturazioni (velocity_ik_solver.cpp:10-34)
dq_p   = Jᵀ (J Jᵀ + λ² I)⁻¹ twist, ogni componente clampato a ±max_joint_speed   (velocity_ik_solver.cpp:36-52)
N      = I − Jᵀ(J Jᵀ + λ_ns² I)⁻¹ J                                       (JPS:64-69)
dq_ns  = N · k_ns · (q_ref − q), con (q_ref − q)[0] moltiplicato per 10      (JPS:74-77)
dq     = dq_p + dq_ns
```

   Guadagni (default: `JPSH:84-101` e `velocity_ik_solver.hpp:40-47`): `kp_linear = kp_angular = 1.0 s⁻¹`, `damping_lambda = 0.05`, `max_linear_speed = 0.15 m/s`, `max_angular_speed = 0.5 rad/s`, `max_joint_speed = 1.5 rad/s`, `nullspace_gain k_ns = 0.2236 s⁻¹`, `joint1_nullspace_gain_scale = 10`, `nullspace_damping = 0.05`. In questo rollout le saturazioni sono **attive** (a differenza dell'IK statica, che le disabilita: `EVAL:60-62`).
4. **Riferimento nullspace**: in v2 il seme è `q_start` e il riferimento è `q_rollout_ref = params.rollout_q_ref` (`SEL:276-278, 558`; passato come `q_ref` a `simulate`, `KF:97`). **Precisazione**: non è `readyPose()` per costruzione; coincide numericamente con essa solo perché lo YAML del run pone `rollout_q_ref = [0, −0.785398, 0, −2.35619, 0, 1.5708, 0.785398]` (`config:94`, e `readyPose` in `franka_cartesian_control/src/core/robot_model.cpp:79`). Il default `readyPose()` dell'argomento `q0` di `checkKinematicFeasibility` (`KFH:103-104`) non è usato in v2. Nel post-hoc legacy (`SEL:478`) non si passa `q_ref`, quindi vale `q_ref = q_start` (`KF:97`).
5. **`FeasibilityResult`** (`KFH:17-49`, riempito in `KF:99-150`):

| Campo | Significato |
|---|---|
| `feasible` | `max_joint_violation_rad < tol` (1e-4) (`KF:149`). Il solo criterio. |
| `max_joint_violation_rad` | massima uscita dai limiti URDF su tutto il percorso. |
| `pos_err_final_mm` | errore FK finale vs obiettivo; **solo diagnostico** (contiene l'artefatto del ritardo di inseguimento del simulatore, `KFH:89-96`). |
| `q_final` | configurazione del **penultimo** istante (quella PRIMA dell'ultima integrazione: `steps.back().q`, `KF:141`). |
| `w_trans_final` | `w_trans` dell'ultimo `Step`. |
| `worst_joint/worst_step/worst_step_fraction/worst_joint_value_rad/worst_joint_limit_rad` | diagnostica della violazione peggiore; restano ai default se non c'è violazione. |
| `min_joint_margin_rad` | minimo su passi e giunti di `min(q−lo, hi−q)` (negativo se un limite è superato); diagnostico. |
| `min_margin_joint` | giunto (0–6) del minimo, il primo in caso di parità; −1 se nulla simulato. |
| `min_margin_step` | indice del passo del minimo (primo in caso di parità); tempo del campione relativo alla fine dell'approccio = `(min_margin_step+1)·dt − tau`. |
| `q_end_post_step` | configurazione DOPO l'ultima integrazione = stato esatto a AT_END da cui parte il follow. |

6. **`w_rollout`** (`SEL:573-577`): `robot.update(fr.q_final, 0)` poi `fullManipulability(robot.jacobian()).w_full`, quindi `w_full` alla configurazione `q_final` (quella prima dell'ultimo passo, un `dt = 5 ms` prima di `q_end_post_step`).
7. **Margine di approccio vs margine con follow**:
   - *Margine di approccio* = `min_joint_margin_rad` del rollout (`rec.rollout_min_margin_rad`, `SEL:564`): minimo di `min(q−lo, hi−q)` sui campioni dell'approccio, `t ∈ (0, tau]`. È `path_margin` nel log di stato (`NODE:328-329`, `posthoc_min_margin_rad`).
   - *Margine con follow* = `followMargin().min_margin_rad` (`SEL:597`, §5): lo stesso minimo esteso ai 3 s di follow dopo AT_END. È quello confrontato con `min_margin_deg` (`SEL:600`).
   - *Margine statico IK* = margine della soluzione IK statica (§3c): tutt'altra grandezza.
8. **Esempio (ω −5)**: `rollouts=20`, `path_margin=0.611 rad` [M]. Fine-approccio: `rollout_err_mm = 7.14`, `rollout_err_deg = 3.729` per #20 [M] (tracciamento del simulatore, non criterio). `q0` ha `q6 = 3.141609` [M, `planner.log`]; il limite superiore di `fer_joint6` è 3.7525 (`fer_flat_effort.urdf:189`) ⇒ margine iniziale `3.7525 − 3.141609 = 0.610891 rad = 35.00°` [C]. Il margine di approccio 0.611 rad coincide quindi con quello della configurazione di partenza: il minimo cade al primo campione (`(0+1)·0.005 − 24.474 = −24.469 s`, riportato come "t_rel −24.47 s" [C]) e l'approccio non scende mai sotto.

---

## 5. `followMargin`

File `FM:16-112`, parametri `FMH:30-34`, risultato `FMH:36-49`. Nel selettore: `SEL:585-601`.

Algoritmo, passo per passo:

1. **Controlli** (`FM:22-33`): modello non nullo, passi finiti > 0, `follow_s ≥ 0`, ω finita, asse **solo +z** (`FM:31-33`; l'executor ruota con `rotatePoseAboutZ`).
2. **Strategia**: `NullspaceBiasedResolution` con parametri di default (`FM:35`), stessi del rollout.
3. **Stato iniziale**:
   - Se il chiamante passa `FollowApproachState` (nel selettore: `rec.rollout_end_valid ? &rec.rollout_end : nullptr`, `SEL:594`), l'approccio **non** viene ripetuto: si parte da `approach->q_end` e si prende il minimo di approccio da lì (`FM:56-64`). `rollout_end` è riempito in `runRollout` da `q_end_post_step`, margine/giunto/tempo del minimo (`SEL:565-571`, con `t_rel = (min_margin_step+1)·dt − tau`). Vale solo se seme, `q_ref`, bersaglio, dt e template sono gli stessi (non verificato dal codice, `FMH:52-56`). Il test `test_follow_margin` verifica che il risultato sia identico bit a bit.
   - Altrimenti (`FM:65-79`) si rifà l'approccio da FK(`q_seed`) con `buildApproachTrajectory`, campionando il margine di `q` prima di ogni passo con etichetta `(i+1)·dt − tau`.
4. **Follow** (`FM:82-99`): `goal_at_end = {target_position, target_orientation}`; `n_follow = round(follow_s / dt_follow) = round(3.0/0.01) = 300` passi. Per `step = 1…300`: `t_follow = step·0.01`; bersaglio = `rotatePoseAboutZ(center, goal_at_end, ω·t_follow)` (il bersaglio co-ruota col satellite); feed-forward lineare `(−ω r_y, ω r_x, 0)` con `r = pos_ruotata − centro` e angolare `(0, 0, ω)` (`FM:92-94`); `robot.update(q)`; **campiona** il margine di `q` etichettandolo `t_follow`; integra `q += dq·0.01`.
5. **`t_rel`**: tempo del campione limitante **relativo ad AT_END** (fine dell'approccio, istante `tau` dal TRIGGER): nell'approccio vale `(i+1)·dt − tau` (≤ 0), nel follow vale `t_follow ∈ (0, follow_s]` (`FMH:40-43`). Convenzione: il margine della configurazione PRIMA di un passo ha l'etichetta del tempo finale di quel passo; la configurazione raggiunta dopo l'ultimo passo del follow **non** viene campionata (`FMH:41-43`).
6. Il minimo usa confronto stretto (`m < min_all`): a parità vince il primo (`FM:46`). Restituisce `min_margin_rad`, `joint`, `t_rel_s`, `approach_min_margin_rad`, `q_at_end`, `q_final`.

**Cosa verifica**: solo il margine dai limiti di giunto lungo approccio + 3 s di co-rotazione simulata con il proxy cinematico dell'impedenza (Eulero esplicito).
**Cosa NON verifica** (dal codice): collisioni; velocità o coppie dei giunti (le velocità sono solo saturate dal solver); errore di inseguimento cartesiano (non entra nel criterio); manipolabilità/singolarità lungo il follow; contatto o forze; ciò che accade dopo `margin_follow_s`; variazioni di ω.

**Esempio (ω −5)**: #20: `margin=35.00 deg limiting=fer_joint6 t_rel=-24.47 s` [M]. Come per il margine di approccio, il minimo è al primo campione (configurazione iniziale, 35.00° = margine di `q6` a `q0`, §4.8); il follow di 300 passi non scende sotto. Valori diversi per altri finalisti: #13 `13.26° (fer_joint7, t_rel 3.00 s)`, cioè limitato a fine follow; #18 `8.65° (fer_joint6, t_rel 3.00 s)` [M]. Soglia 24° ⇒ #13, #14 (22.42°), #16 (17.67°), #18 (8.65°) scartati [M].

---

## 6. Clearance

Funzioni: `approachClearance` (`CLR:263-383`), `ApproachClearanceModel` (`CLR:176-216`), loader mesh (`CLR:399-541`), valutatore (`CLR:385-393`). Nel selettore: `clearanceOf` (`SEL:621-650`), soglia a riga 648.

**Traiettoria comandata** (`SEL:627-630`): `timedApproachTrajectory(template, p0, quat0, goal = position_world + contact_to_end_offset, goalOrientation(cand.eval), kDt)` (`KF:43-65`): primo campione `t = 0` (`p0`, `quat0`), poi campioni a `(i+1)·dt` con posizione clampata al bersaglio per `t ≥ tau`. È la traiettoria Cartesiana **ideale** di posizione ProDMP + SLERP dell'orientazione, non il percorso a livello di giunti del rollout (`CLRH:6-12`: il modello riproduce `predict_distance.py`, predittore T, errore medio 0.35 mm / massimo 1.46 mm su 17 run senza contatto [dichiarato nel commento, non riverificato]). Il tempo `t = 0` è l'inizio dell'approccio (il TRIGGER).

**Moto del satellite**: `θ(t) = θ\* − ω·(τ − t)` (`CLR:352`); cubo e collare ruotano di `θ(t)` attorno all'asse per il centro (righe 353-359).

**Finestra e passo** (`CLR:315-327`): tempi `k·dt` per `k·dt < τ − 1e-9`, più `t = τ`; `dt = clearance_dt_s = 0.05`. Finestra "senza coda": `t ≤ τ − 2 + 1e-4` (`kClearanceTailExcludeS = 2.0`, `CLRH:51`; `tail_limit` riga 327). Esempio: τ = 24.474 ⇒ 491 campioni (490 + τ) [C]; quelli "senza coda" fino a t ≈ 22.474 s.

**Geometria del robot** (righe 335-350):
- **Mano**: mesh `collision/hand.stl` del link `fer_hand` (`fer_flat_effort.urdf:240`), risolta con `AMENT_PREFIX_PATH` (`CLR:440-458, 532-541`). Nel log: `HAND_MESH vertices=102 triangles=200 … max_convexity_violation_mm=0.739692` [M, `planner.log`]; coerente con Euler V−E+F = 102 − 300 + 200 = 2 [C]. Convertita in `coal::Convex<coal::Triangle>` da `makeCoalConvex` (`CLR:83-91`) dopo `validateHandMesh` (indici validi, triangoli non degeneri, orientazione verso l'esterno, convessità entro 1 mm: `CLR:95-127`, `kHandMeshConvexityTolM`). Il sistema della mano è quello del TCP traslato di `kHandTcpOffsetM = 0.1034 m` lungo z: `p_hand = p_tcp − R·(0,0,0.1034)` (riga 339).
- **Dita**: due, 4 box ciascuna (`kFingerBoxes`, `CLR:42-47`): supporto vite 22×15×20 mm, carrello 22×8.8×3.8 mm, dito diagonale 17.5×7×23.5 mm (ruotato di π/6 intorno a x), punta in gomma 17.5×15.2×18.5 mm. Origine del dito = `p_hand + R·(0, ±gripper_open_m, 0.0584)` (riga 345; `kFingerOriginZM`). "dito 1" = sinistro, +y (`side 0`); "dito 2" = destro, −y.
- Totale shape robot: 1 + 8 = 9 (`robot.reserve(9)`, riga 331).

**Geometria del satellite**: cubo box lato `cube_side_m` (0.150 m) al centro; **collare** = 24 box (`kCollarSegments = 24`, `CLRH:48`) di dimensioni `standoff × wall × tangenziale`, con `tangenziale = 2·R·tan(π/24)·(1+0.05)` (`CLR:200-201`, `kCollarOverlap = 0.05`), centri a `collar_xc = side/2 + standoff/2` lungo la normale e raggio medio `collar_rm` (righe 202-203, 302-313). Shape satellite: 1 + 24 = 25 ⇒ 225 coppie per istante.

**Distanza con segno** (`signedDistanceM`, `CLR:61-68`): coal con `enable_signed_distance = true`; negativa in penetrazione. Il minimo su tutte le coppie e tutti i tempi (righe 363-380); le coppie con sfere di inviluppo disgiunte e più lontane del minimo corrente sono saltate (righe 365-367).

**"No-tail"**: `min_mm_no_tail` (`ClearanceResult`, `CLRH:74-83`) è il minimo sulla finestra senza gli ultimi 2 s ("min_mm_senza_ultimi_2s"): **è il valore che il vincolo usa** (`f.clearance_mm = cr.min_mm_no_tail`, `SEL:644`; `rejected = !(min_mm_no_tail >= min_clearance_mm)`, `SEL:648`). `min_mm` (tutta la finestra) è registrato in `clearance_full_mm` (`SEL:647`) solo informativo. Motivo: negli ultimi 2 s la mano deve arrivare al collare (commento `CLRH:50`).

**Soglia**: `params.min_clearance_mm = 2.0` mm (ASSUNTO), `SELH:104`, applicata a `SEL:648`.

**Esempio (ω −5)**: #20: `clearance=12.46 mm, dito 1-collare, t_rel=-8.37 s` [M] ⇒ 12.46 ≥ 2.0, ammissibile. Bocciati per clearance, ad es.: #5 (k0A −152°): −1.17 mm (dito 1–collare, t_rel −5.37 s); #1: −14.03 mm; #19: −17.12 mm [M]. La stage di clearance è stata chiamata su 16 finalisti (20 meno i 4 scartati per margine) in 243.3 ms ⇒ circa 15 ms ciascuno [M e C].

---

## 7. Legge di lancio

**Attesa** (`launchDelay`, `src/core/launch_plan.cpp:16-27`):

```cpp
const double T = 2.0 * kPi / std::abs(omega_rad_s);
const double t = (theta_star_rad - theta_now_rad) / omega_rad_s - tau_contact_s;
double r = std::fmod(t - min_delay_s, T);
if (r < 0.0) r += T;
if (r >= T - 1e-9) r = 0.0;
return min_delay_s + r;
```

Cioè `t = (θ\* − θ_now)/ω − τ`, ridotta modulo il periodo `T = 2π/|ω|` nell'intervallo **[min_delay, min_delay + T)** (ω con segno).

**Piano di lancio** (`planLaunch`, `launch_plan.cpp:29-41`): `θ_now = phaseAt(snapshot, t_now)` = `wrapPi(θ_snap + ω(t_now − t_snap))` (righe 11-14); `delay = launchDelay(θ_now, θ\*, ω, τ, min_delay)`; `launch_time = t_now + delay`; `contact_time = launch_time + τ`.

**`θ_trig`** (usato dall'executor): `θ_trig = wrapPi(θ\* − ω·τ)` (`satellite_intercept.hpp:596-600`, `InterceptTrigger`, costruttore riga 254, con `wrap2Pi`). Identità: `θ_now + ω·t_wait ≡ θ_trig (mod 2π)` (`docs/S3_SPEC.md` §2.3).

**Due istanti `t_now` diversi** (non confonderli):
- *Nel costo dei candidati* (`SEL:334`): `t_now = ctx.t_now_s = snapshot.t_s` (`LAUNCH:53`), cioè senza il tempo di selezione.
- *Nel ritardo pubblicato* (`LAUNCH:84-85`): `t_now = snapshot.t_s + elapsed_s`.

**Ritardo minimo `min_delay_s`**: parametro obbligatorio senza default (`NODE:96`); `config/grasp_planner_example.yaml:31` ha 0.05 s; il runner delle campagne lo pone a **5.0 s** (`tools/s1_run.py:549`). Se l'attesa necessaria `t` è inferiore a `min_delay_s`, `fmod(t − D, T)` è negativo e si aggiunge **un periodo intero** `T` (per ω = 5 °/s: 72 s). Nel nodo succede se `selection_elapsed` fa scendere `t` sotto `D` (la selezione consuma tempo): il ritardo provvisorio salta di un giro.
**Ma l'executor non usa questo ritardo**: scatta sull'incrocio di `θ_trig` (`InterceptTrigger::update`, `satellite_intercept.hpp:259-279`: "fires on the first sample with e ≥ 0 (omega > 0) / e ≤ 0 (omega < 0) after a sample on the other side has been seen. If the very first sample is already past the threshold, the next lap is awaited"). Quindi nell'esecuzione reale: se alla ricezione il satellite ha già passato θ_trig, aspetta il giro successivo; nessun minimo di 5 s è imposto dall'executor (cercato in `satellite_intercept.hpp`; non ho letto per intero `prodmp_gazebo_executor_node.cpp`: NON CHIARO se l'executor aggiunge un anticipo minimo altrove).

**Esempio (ω −5), formule con i numeri del log** [C]:
- Nel costo: `θ_now = 13.115°` (snapshot), `θ\* = −166°`, `τ = 24.474 s`, `ω = −5 °/s`, `T = 72 s`: `t = (−166 − 13.115)/(−5) − 24.474 = 35.823 − 24.474 = 11.349 s`; con `D = 5`: `r = fmod(6.349, 72) = 6.349` ⇒ `wait = 5 + 6.349 = 11.349 s` = `wait_s=11.3490` del log [M].
- Nel lancio: `t_now = snapshot.t_s + 2.454` ⇒ `θ_now = 13.115 − 5·2.454 = 0.845°`; `t = (−166 − 0.845)/(−5) − 24.474 = 8.895 s` = `provisional_delay (wait_s)=8.895 s` [M, report].
- `θ_trig = wrapPi(−166 + 5·24.474) = −43.63°`; l'attesa effettiva pub→TRIGGER misurata è 8.893 s e l'angolo al TRIGGER −43.635° (atteso −43.630°) [M, `report.txt`].

---

## 8. Nodo ROS

**Lettura parametri**: `NODE:47-260`. Obbligatori (fail loud, `NODE:60-81`): `demo_params_path`, `min_delay_s`, `cube_geometry.*` (7), `robot_base_world`, `satellite_rotation_center/axis/angular_velocity_deg_s`, `satellite_q_ref`, `satellite_odom_topic`, `omega_consistency_tol_deg_per_s`. Opzionali: `urdf_path` (default `$HOME/thesis_ws/fer_flat_effort.urdf`, `NODE:87-95`), `q_ref_consistency_tol_deg`, `robot_base_world_tol_m`, `grasp_command_topic` (`/grasp_command`), `cube_geometry.grasp_radius_m`/`grasp_depth_m`, `start_joints_override_rad`, tutti gli `s1.*`. Richiede `use_sim_time=true` (`NODE:52-58`). Controlli all'avvio: `checkRobotBaseWorldIsZero`, `checkCubeAxisMatchesRotationAxis`, `checkCubeCenterMatchesRotationCenter`, `checkQRefOnAxis` (`NODE:244-247`).

**Costruzione del contesto**: `buildSelectionInputs` (`NODE:300`) e, dentro `planGraspAndLaunchS1`, `S3Context` (`LAUNCH:47-61`) con `demo.position = p0 + Δg_demo`, `demo.orientation = FK(q0)` (`LAUNCH:41-45`) e il valutatore di clearance. Modello di clearance (mesh) costruito una volta all'avvio se `use_clearance` (`NODE:225-240`).

**Chiamata**: `NODE:324-326`. **Pubblicazione**: `NODE:439-440` (`toGraspCommandMsg`, `grasp_command_conversion.cpp:8-59`); campi: `status`, `goal_pose` (posizione goal = presa + offset, orientazione `goalOrientation`), `k`, `theta_star_rad`, `wrist_flip`, `provisional_delay_s`, `cost`, `d_pos_m`, `w_hat`, `min_joint_margin_rad`, `posthoc_dv_m_s`, `start_position`, `contact_time_s = tau_launch`, `omega_rad_s`, `snapshot_theta_rad`, `snapshot_t_s`, `weights_sha256`, `evaluations`, `selection_elapsed_s`.

**Righe loggate e campi** (formati in `NODE`; il testo del log in `results/margin2_w-5.0/planner.log`):

| Log | Riga | Campi |
|---|---|---|
| `s1.use_margin=true …` | 218-224 | `min_margin_deg`, `margin_follow_s`, `margin_follow_dt_s`; ricorda che è lazy e che `tau_j` non si applica. |
| `s1.use_clearance=true …` | 235-238 | `min_clearance_mm`, `clearance_dt_s`, `gripper_open_m`. |
| `HAND_MESH vertices=… triangles=… bbox_min … bbox_max … max_convexity_violation_mm` | 239 (`describeHandMesh`, `CLR:129-145`) | descrive la mesh. |
| `ready, waiting for the first sample …` | 258 | topic odom, configurazione di partenza. |
| `start state: q0 source …` | 302 (`describeStartState`, `selection_inputs.cpp:157-170`) | `q0`, `p0`, distanza da `readyPose`, provenienza (tau_contact, Δg, offset, w_trans_demo, ω, centro, asse). |
| `FORCED k=… theta=… psi=…` | 305-309 | solo con `force`. |
| `s1.mode=v2 branch_rule=… use_w … kappa_w … kappa_t … tau_j … finalists_n … max_start_goal_deg … r_trust_m … posthoc_cluster_deg … rollout_q_ref=[…]` | 315-321 | parametri v2 effettivi. |
| `s1 status=… evaluations=… rollouts=… margin_evals=… posthoc_checks=… dv=… path_margin=… note:` | 327-333 | contatori di `S3Result`. |
| `v2 finalists (J_static order): …` intestazione | 335-338 | nomi delle colonne. |
| `v2 finalists evaluated: N of M` | 343-344 | quanti hanno fatto il rollout. |
| `v2 finalist i k=… A/B theta=… d_mm=… w_static=… w_rollout=… J=… wait_s=… start_goal_deg=… err=… mm/… deg [margin=…] [clearance=…] [ROLLOUT_INFEASIBLE] [DISCARDED(…)] [WINNER]` | 374-382 | un record per finalista valutato; `i` = rango J statico; `J` = `f.j` (con `w_rollout`) o `n/a`. |
| `v2 clearance stage: X ms` | 385-387 | `clearance_stage_ms`. |
| `WARN …` (avvisi) | 388-390 | soglie violate con `force`. |
| `selected k=… theta=… psi=… wrist_flip=… (force_flip=…) branch_rule=… start_goal_deg[A]=… [B]=… wait_s=…` | 391-401 | vincitore. |
| `selected winner: predicted margin … deg (fer_joint… at t_rel …), clearance … mm (… at t_rel …)` | 402-415 | margine e clearance previsti del vincitore. |
| `FORCED candidate VIOLATES constraints …` (ERROR) | 417-421 | solo `force_ignore_constraints`. |
| `published GraspCommand (status=… , evaluations=…), now idle.` | 441-444 | pubblicazione. |
| `selection failed: …` (FATAL) | 446 | eccezione ⇒ `rclcpp::shutdown()`. |

Nota: nel ciclo di log `if (!f.rollout_done) continue;` (riga 347) rende codice morto il ramo `jbuf = "n/v"` (righe 349-350).

---

## 9. Strumenti offline e di test

**Eseguibili di analisi** (`analysis/`, `-DBUILD_ANALYSIS_TOOLS=ON`; dettagli in `analysis/README.md`):

| Strumento | A cosa serve | Esempio |
|---|---|---|
| `s3_landscape` | Esporta in CSV, per ogni (k, θ, ψ), vincoli, `d`, `w_hat`, `w_full`, margini, attesa… (paesaggio del selettore) per valutare formulazioni alternative del costo; `--selftest` confronta con il log reale. | `ros2 run satellite_grasp_planner s3_landscape --selftest` ; `ros2 run satellite_grasp_planner s3_landscape --seed-spread 0 --out landscape.csv` |
| `s3_clearance` | Clearance di UN candidato a ω dato. | `ros2 run satellite_grasp_planner s3_clearance --k 2 --theta-deg 145.3 --flip 1 --omega-deg-s -4.0` |
| `s3_followmargin_check` | Riproduce con `followMargin` i margini G16 (13.26, 17.67, 35.00, 35.00, 20.84, 28.63°). | eseguire dalla radice del workspace |
| `orientation_law_compare`, `orientation_path_check` | Confronto di leggi di orientazione (SLERP, minjerk, QDMP) e loro fattibilità lungo il percorso; non usati dal selettore. | vedi `analysis/README.md` |

**Script Python** (`tools/`):

| Script | Funzione |
|---|---|
| `s1_run.py` | UN run S1 completo (bringup Gazebo, cubo rotante, bag, executor, planner) + report. Esempio: `python3 tools/s1_run.py --weights-dir runs/<run> --depth 0.0125 --kt 2000 --tag prova1` |
| `s1_campaign.py` | Griglia di run. Esempio: `python3 tools/s1_campaign.py --weights-dir runs/<run> --outroot s1_runs/camp1 --depth 0.0 0.0125 --kt 2000 --spawn-yaw -73 --squeeze default 0.002` |
| `s1_report.py` | Analisi di UN bag già registrato → `report.json/report.txt`. Esempio: `python3 tools/s1_report.py BAG_DIR --executor-log executor.log` |
| `s1_kinematic_metrics.py` | Grandezze cinematiche continue di un run (confronto con `w_full` previsto). Esempio: `python3 tools/s1_kinematic_metrics.py BAG_DIR --executor-log executor.log` |
| `s1_env.py` | Riordina `AMENT_PREFIX_PATH` per i run. Esempio: `eval "$(python3 tools/s1_env.py --root /root/thesis_ws/install)"` |
| `s3_landscape_report.py` | Analisi offline del CSV di `s3_landscape` (formulazione `J_new = d_mm + κ_w(−ln w_full)`, mappe in κ_w/κ_t). Esempio: `s3_landscape_report.py --csv landscape.csv --outdir DIR --kappa-list 0,1,2,5,10,20,50,100` |

**Test C++ (gtest)**, uno per modulo, in `test/` (registrati in `CMakeLists.txt:204-340`): `test_manipulability`, `test_joint_path_simulator`, `test_kinematic_feasibility`, `test_follow_margin`, `test_selection_inputs`, `test_start_state`, `test_launch_delay`, `test_launch_plan`, `test_grasp_planner_checks`, `test_grasp_command_conversion`, `test_grasp_s3_selection` (1480 righe; classi `GraspS3Test`, `GraspS3PsiTest`, `GraspS3ForceTest`, `GraspS3FlipTest`, `GraspS3BranchRuleTest`, `GraspS3V2Test`, `GraspS3ClearanceTest`, `GraspS3MarginTest`; ad es. `BelowThresholdDiscardedAdmissibleChosenLazy` riga 1331, `MarginIsCheckedBeforeClearance` riga 1428), `test_theta_convention`, `test_grasp_pose_eval`, `test_approach_clearance`, `test_s3_landscape_log`. Comando: `colcon test --packages-select satellite_grasp_planner` (non eseguito).
**Test Python** (`tests/`): `test_s1_env.py`, `test_s1_interrupt.py`, `test_s1_kinematic_metrics.py`, `test_s1_omega_campaign.py`, `test_s1_report.py`, `test_s1_run_planner_param.py`, `test_s3_landscape_report*.py`. Comando: `python3 -m pytest tests/` (non eseguito).

---

## 10. Tabella dei parametri

Il "valore usato nelle prove" è quello del run `margin2_w-5.0` quando risulta dal `planner.log`/`run_meta.json`; altrimenti "default". `min_delay_s` e `posthoc_velocity_check` vengono dal runner (`tools/s1_run.py:549-550`), non dal `planner.log` del run (non stampati nel log analizzato).

| Parametro (ROS) | Default nel codice | Usato nelle prove | Unità | File:riga | ASSUNTO |
|---|---|---|---|---|---|
| `s1.length_scale_m` | 0.10 | default | m | `SELH:37`; `NODE:146` | no |
| `s1.orientation_weight` | 0.0 | default | – | `SELH:38`; `NODE:147` | no |
| `s1.angle_scale_rad` | 1.0 | default | rad | `SELH:39`; `NODE:148` | no |
| `s1.epsilon_wait_per_s` | 1e-4 | default | 1/s | `SELH:40`; `NODE:149` | no |
| `s1.w_hat_min` | 0.5 | non applicato in v2 | – | `SELH:41`; `NODE:150` | non indicato |
| `s1.margin_min_rad` | 0.3 | non applicato in v2 | rad | `SELH:42`; `NODE:151` | non indicato |
| `s1.r_trust_m` | 0.30 | 0.300 [M] | m | `SELH:43`; `NODE:152` | non indicato |
| `s1.delta_v_max_m_s` | 3e-3 | default (controllo spento) | m/s | `SELH:44`; `NODE:153` | non indicato |
| `s1.theta_step_deg` | 2.0 | default (880 valutazioni) | deg | `SELH:45`; `NODE:154` | no |
| `s1.theta_refine_step_deg` | 0.1 | default | deg | `SELH:46`; `NODE:155` | no |
| `s1.ik.*` | 1e-4 m / 1e-3 rad / 300 / 0.05 / 1.0 / 0.2 | default | – | `EVALH:27-34`; `NODE:159-163, 204` | no |
| `s1.posthoc_path_check` | true | ignorato in v2 | – | `SELH:48`; `NODE:156` | no |
| `s1.posthoc_velocity_check` | true | **false** (runner) | – | `SELH:49`; `NODE:157`; `tools/s1_run.py:550` | no |
| `s1.max_posthoc_candidates` | 10 | default | – | `SELH:50`; `NODE:205` | no |
| `s1.posthoc_cluster_deg` | 10.0 | 10.00 [M] | deg | `SELH:53`; `NODE:158` | no |
| `s1.force*` | false … | false | – | `SELH:60-68`; `NODE:165-178` | no |
| `s1.branch_rule` | `"margin"` | `rotation` | – | `SELH:72`; `NODE:180` | sì (YAML) |
| `s1.mode` | `"legacy"` | `v2` | – | `SELH:83`; `NODE:182` | sì (YAML) |
| `s1.max_start_goal_deg` | 180.0 | 180.00 [M] | deg | `SELH:86`; `NODE:183` | sì (YAML) |
| `s1.finalists_n` | 12 | **40** [M] | – | `SELH:87`; `NODE:184-188` | sì (YAML) |
| `s1.rollout_q_ref` | vuoto (obbligatorio v2) | readyPose [M] | rad | `SELH:90`; `NODE:189`; `config:94` | no (DERIVATO) |
| `s1.use_w` | true | true [M] | – | `SELH:91`; `NODE:190` | sì (YAML) |
| `s1.kappa_w` (→ `kappa_w_mm`) | 7.5 | 7.500 [M] | mm | `SELH:92`; `NODE:191` | **sì** (codice) |
| `s1.kappa_t` (→ `kappa_t_mm_s`) | 0.0 | 0.000 [M] | mm/s | `SELH:93`; `NODE:192` | sì (YAML) |
| `s1.tau_j_mm` | 2.0 | 2.000 [M] (non applicata nel lazy) | mm | `SELH:94`; `NODE:193` | **sì** (codice) |
| `s1.use_clearance` | false | true | – | `SELH:103`; `NODE:195` | sì (YAML) |
| `s1.min_clearance_mm` | 2.0 | 2.000 [M] | mm | `SELH:104`; `NODE:196` | **sì** (codice) |
| `s1.clearance_dt_s` | 0.05 | 0.050 [M] | s | `SELH:105`; `NODE:197` | no (DERIVATO) |
| `s1.gripper_open_m` | 0.060 | 0.0600 [M] | m | `SELH:108`; `NODE:198` | no (DERIVATO) |
| `s1.use_margin` | false | true | – | `SELH:118`; `NODE:200` | sì (YAML) |
| `s1.min_margin_deg` | 24.0 | 24.000 [M] | deg | `SELH:120`; `NODE:201` | **sì** (codice) |
| `s1.margin_follow_s` | 3.0 | 3.000 [M] | s | `SELH:122`; `NODE:202` | no (DERIVATO) |
| `s1.margin_follow_dt_s` | 0.01 | 0.0100 [M] | s | `SELH:124`; `NODE:203` | no (DERIVATO) |
| `min_delay_s` | nessun default (obbligatorio) | 5.0 (runner) | s | `NODE:96`; `tools/s1_run.py:549`; YAML esempio 0.05 (`config:31`) | non indicato |

Costanti non parametrizzate nel selettore: `kDt = 0.005 s` (`SEL:27`), `kRolloutViolTolRad = 1e-4` (`SEL:28`), `kRankedSize = 20` (`SEL:26`), `kHandTcpOffsetM = 0.1034 m`, `kFingerOriginZM = 0.0584 m`, `kClearanceTailExcludeS = 2.0 s`, `kCollarSegments = 24`, `kCollarOverlap = 0.05` (`CLRH:41-51`), `kHandMeshConvexityTolM = 1e-3 m` (ASSUNTO, `CLRH:96`).

---

## 11. Cosa il codice NON fa (limiti noti)

1. **Un solo campione, una sola selezione, nessun ripianificare**: solo il primo messaggio odom è gestito (`NODE:267`); dopo la pubblicazione il nodo resta inattivo (`grasp_planner_node.hpp:9-12`). Nulla si aggiorna dopo il lancio (`launch_plan.hpp`: "nothing is updated after the launch").
2. **ω costante e nota**: `phaseAt` (`launch_plan.cpp:13-16`) e il moto in `approachClearance` (`CLR:352`) assumono ω costante; ω viene da un parametro, solo verificata contro il twist (`NODE:281`).
3. **Asse di rotazione +z per il follow**: `followMargin` rifiuta ogni altro asse (`FM:31-33`).
4. **Il lazy non usa `tau_j` e non ha fallback oltre `finalists_n`**: `SEL:657` (scansiona solo `fin`), `SEL:720-741` (nessun ammissibile ⇒ `kNoCandidate`); `NODE:221-222` lo dichiara.
5. **`w_rollout` non influenza la scelta nel lazy**: `rec.j` non compare nelle righe 657-719.
6. **`w_full` mescola unità** (metri e radianti, nessuna scala): `manipulability.hpp:62-66`. Il peso `κ_w = 7.5 mm` (ASSUNTO) non è quindi derivato.
7. **Margine di giunto = solo limiti di giunto**: niente velocità/coppie/collisioni/contatto (`FM:16-112`); soglia 24° ASSUNTA su 6 dati G16 (`SELH:119`).
8. **La clearance considera solo mano e dita contro cubo e collare**, non i link del braccio (il vettore `robot` contiene mano + 8 box, `CLR:341-350`); usa la traiettoria Cartesiana ideale, non quella a livello di giunti (`CLRH:6-12`); ignora gli ultimi 2 s (`CLR:327`); soglia 2 mm ASSUNTA.
9. **Rollout = proxy cinematico**: Eulero esplicito, senza clamp (`JPS:152`); l'errore di posizione finale (qui 7.14 mm) non entra nel criterio (`KF:147-149`, `KFH:89-96`); l'orientazione è SLERP (l'executor, secondo `S3_SPEC` D11, usa `QuaternionDMP`: non verificato qui).
10. **`contact_to_end_offset` non entra in IK/rollout/margine**: la posa passata a IK e rollout è `pose.position_world` (`SEL:557-558`, `EVAL:187`), mentre `d_pos` e la clearance usano `position_world + contact_to_end_offset` (`SEL:315`, `SEL:628-630`; `S3_SPEC` D10). Nel run l'offset è (0,0,0) ⇒ nessun effetto [M].
11. **Attesa dei candidati calcolata a `t_now = snapshot.t_s`** (`LAUNCH:53`), non a `+ elapsed`: gli spareggi sull'attesa (non lazy) usano attese sfalsate di `elapsed` rispetto a quelle poi pubblicate (`LAUNCH:84`).
12. **Il ritardo pubblicato non comanda il lancio**: l'executor scatta per fase (`satellite_intercept.hpp:259-279`); `min_delay_s` e il suo "giro in più" valgono solo per la stima.
13. **Nessun controllo di velocità nelle campagne**: `s1.posthoc_velocity_check=false` (`tools/s1_run.py:550`); log `dv=0 m/s`.
14. **Controllo post-hoc limitato**: massimo `max_posthoc_candidates = 10` controlli (`SEL:684, 792`); `kPosthocFailed` non significa che tutti i candidati abbiano fallito.
15. **`ψ` fisso a 0 nelle scansioni** (`SEL:400, 412`): l'orientazione di presa è quella nominale o ruotata di π (nessun roll continuo); `ψ` solo con `force`.
16. **Le soglie legacy (`w_hat_min`, `margin_min_rad`) non sono applicate in v2** (`SEL:184-193`).
17. **Commento obsoleto**: `grasp_planner_node.hpp:9` dice "~5.2 s Release / ~500 s unoptimized"; la selezione misurata è 2.454 s [M].

---

## 12. Esempio guidato: ω = −5 °/s, 20 finalisti valutati

Setup [M]: `evaluations=880`, 40 finalisti, `rollouts=20`, `margin_evals=20`, `posthoc_checks=1`, `dv=0`, percorso lazy (`use_margin=true`, `force=false`). κ_w = 7.5 mm, κ_t = 0.

I finalisti sono in ordine di **J_static** (calcolato da me con `d_mm` e `w_static` del log, formula §3d [C]). La colonna "margine" è il margine previsto (`followMargin`), "clearance" è `min_mm_no_tail` [M]. Soglie: margine ≥ 24°, clearance ≥ 2 mm.

| # | k | ramo | θ [°] | d [mm] | J_static [C] | margine [°] | clearance [mm] | esito |
|---|---|---|---|---|---|---|---|---|
| 1 | 2 | B | 145.3 | 7.194 | 26.965 | 30.61 | −14.03 | scartato: clearance |
| 2 | 0 | A | −163.2 | 7.208 | 31.034 | 27.46 | −6.82 | scartato: clearance |
| 3 | 2 | B | 134.0 | 31.942 | 50.139 | 32.64 | −14.37 | scartato: clearance |
| 4 | 2 | B | 156.0 | 29.264 | 51.198 | 29.06 | −12.82 | scartato: clearance |
| 5 | 0 | A | −152.0 | 31.831 | 52.867 | 32.26 | −1.17 | scartato: clearance |
| 6 | 0 | A | −174.0 | 29.374 | 55.651 | 27.26 | −13.70 | scartato: clearance |
| 7 | 0 | A | −142.0 | 58.255 | 77.499 | 33.54 | −1.34 | scartato: clearance |
| 8 | 2 | B | 122.0 | 63.642 | 80.919 | 33.57 | −14.55 | scartato: clearance |
| 9 | 0 | A | 174.0 | 61.059 | 85.507 | 28.11 | −13.01 | scartato: clearance |
| 10 | 2 | B | 168.0 | 60.948 | 86.106 | 27.44 | −12.96 | scartato: clearance |
| 11 | 1 | B | 170.0 | 64.074 | 89.170 | 35.00 | −17.36 | scartato: clearance |
| 12 | 1 | B | 158.0 | 72.225 | 95.353 | 35.00 | −17.02 | scartato: clearance |
| 13 | 1 | A | −178.0 | 69.959 | 95.557 | **13.26** (fer_joint7) | n/v | scartato: **margine** |
| 14 | 3 | A | 168.0 | 73.872 | 99.266 | **22.42** | n/v | scartato: **margine** |
| 15 | 0 | A | −132.0 | 84.475 | 102.459 | 34.06 | −2.35 | scartato: clearance |
| 16 | 3 | A | 178.0 | 75.622 | 103.576 | **17.67** | n/v | scartato: **margine** |
| 17 | 3 | A | 156.0 | 82.958 | 105.917 | 28.14 | −15.43 | scartato: clearance |
| 18 | 3 | B | −178.0 | 78.691 | 106.645 | **8.65** | n/v | scartato: **margine** |
| 19 | 1 | B | 146.0 | 90.435 | 111.388 | 35.00 | −17.12 | scartato: clearance |
| **20** | **1** | **A** | **−166.0** | **86.881** | **111.393** | **35.00** | **+12.46** | **WINNER** |

Note di lettura:

- **Perché vince il #20**: il ciclo lazy (`SEL:657-719`) percorre i finalisti in ordine di J_static; ogni finalista precedente fallisce un vincolo (margine < 24° per #13, #14, #16, #18; clearance < 2 mm per tutti gli altri). Il #20 supera rollout, margine (35.00° ≥ 24°) e clearance (12.46 mm ≥ 2 mm); con `posthoc_velocity_check=false` il controllo di velocità è saltato (`dv=0`) e `chosen_idx` è impostato (`SEL:715`), il ciclo esce. I finalisti #21–#40 non vengono valutati ("evaluated: 20 of 40").
- **Ordine dei due ultimi**: J_static(#19) = 111.388 < J_static(#20) = 111.393 (differenza 0.005 mm [C]) ⇒ #19 viene prima; con `w_rollout` l'ordine sarebbe invertito (J(#20) = 110.361, J(#19) = 110.611 [M]) ma il lazy usa il J statico. Il #19 è comunque scartato (clearance −17.12 mm), quindi il vincitore non cambierebbe in questo caso.
- **Attenzione alla numerazione**: nei log la riga "#13" è k1A −178° (w_static=0.03294) e "#20" è k1A −166° (w_static=0.03807, w_rollout=0.04369).
- **Margini identici a 35.00°**: #11, #12, #19 e #20 mostrano 35.00° su `fer_joint6` a t_rel −24.47 s, cioè il margine della configurazione iniziale (§4.8); non sono valori "migliori" ottenuti dal percorso.
- **Il vincitore** (da `report.txt`): `θ\*=−166°`, `d_pos=86.8809 mm`, `cost=110.360752`, `w_hat=0.9474`, `margine giunti min=47.207 deg` (margine statico IK, non il margine previsto), `provisional_delay=8.895 s`.
- **Tempi**: `selection_elapsed = 2.454 s`; `clearance_stage_ms = 243.3 ms` (16 valutazioni). La ripartizione del resto (880 IK a 2 rami, 20 rollout da 4895 passi, 20 `followMargin` da 300 passi) non è registrata separatamente: NON CHIARO.

---

## Appendice A. Verifica delle affermazioni del revisore

| Affermazione | Esito | Riga e motivo |
|---|---|---|
| Ordine dei finalisti per J statico con `w_static` e attesa 0 | **VERO** | `SEL:516` (`s3CostV2(d_mm, w_full_static, 0.0, params)`); ordinamento `SEL:518-521` (a parità `lessCost`). |
| Il vincitore del percorso lazy è il primo ammissibile in ordine di J statico | **VERO** | `SEL:657` (ciclo su `fin` in ordine di J statico, `SEL:602-609`), uscita `SEL:680/715` con `chosen_idx`. Eccezione: `break` a `SEL:684` se si è raggiunto `max_posthoc_candidates` (nessun vincitore). |
| `w_rollout` non cambia il vincitore nel lazy | **VERO** | `rec.j` (con `w_rollout`, `SEL:579`) non è letto in nessuna condizione di `SEL:657-719`. Può cambiare solo il valore loggato, `cost` e l'ordine di `res.ranked` (`SEL:872`). |
| Con la sola `use_clearance` il rollout è fatto su tutti i finalisti e lo spareggio `tau_j` si applica | **VERO** | `lazy = use_margin && !force` (`SEL:292`) ⇒ falso; `SEL:604-607` fa `runRollout` per tutti; `SEL:750` (`s3ClearanceRound`) e `SEL:762` (`s3PickWinnerV2` con `tau_j`). Con `force` non vale (nessun round di clearance, `SEL:745`). |
| I vincoli statici "V1" usano `w_hat` (normalizzata) mentre il costo usa `w_full` | **FALSO così com'è** (parzialmente vero per modalità separate) | L'etichetta "V1" non compare nel codice né nei documenti letti (cercata con `grep`). Legacy: vincolo `ŵ ≥ w_hat_min` (`SEL:172`) ma costo `d + ε·wait` senza `w` (`SEL:335`). V2: costo con `w_full` (`SEL:199`) ma la maschera `s3AprioriRejectMaskV2` (`SEL:184-193`) **non** applica `w_hat_min` né `margin_min_rad` (log `NODE:318`). Le due cose non convivono in nessuna modalità. È invece vero che `w_hat` = 3 sv del blocco lineare / `w_trans_demo` (normalizzata) e `w_full` = 6 sv del Jacobiano completo (non normalizzata): `EVAL:196-197`, `manipulability.hpp:67-93`. |
| La griglia di θ ha 880 valutazioni | **VERO come massimo, con precisazione** | 4×180 + 4×40 = 880 (`SEL:369-370, 398-414`); il raffinamento di un k avviene solo se ha ≥ 1 accettato (`SEL:408`), altrimenti −40 per k. 880 è il valore misurato nel run (`evaluations=880`). Le "valutazioni" comprendono anche le 8 duplicate (j = ±20). La griglia grossa da sola ha 720. |

Altre precisazioni su cose dette in precedenza (nella risposta alle domande 1–7 della sessione): `s3CostLegacy` non esiste come funzione (§3d); "lazy" non è `use_clearance e/o use_margin` ma solo `use_margin && !force` (`SEL:292`).

---

## Domande aperte (NON CHIARO)

1. **"Rilascio rispetto al comando"** (richiesta del §6): non so a quale concetto si riferisca; nel codice di clearance l'unico tempo di riferimento è `t = 0` = inizio dell'approccio e `t_rel = t − τ` (`CLRH:53-58, 74-83`). Non c'è una nozione di "rilascio" del gripper nel modulo.
2. **Valore `min_mm` (tutta la finestra) del vincitore**: il log stampa solo `clearance_mm = min_mm_no_tail` (`NODE:359-360`); `clearance_full_mm` esiste (`SELH:216`) ma non è loggato, quindi per il #20 il minimo includendo gli ultimi 2 s è NON CHIARO.
3. **Numero effettivo di finalisti prima dei vincoli e perché 40**: il log dice "evaluated: 20 of 40" ma non quanti dei 40 siano rollout-fattibili né il loro J, perché il lazy non li valuta.
4. **Log di `margin_w-5.0/planner.log`**: stampa solo 12 righe di finalisti pur con `rollouts=20`. Non so se dipenda da una versione precedente del nodo (non ho la storia di quel binario); in `margin2_w-5.0` ci sono 20 righe. Ipotesi, non verificata.
5. **Origine delle soglie legacy** (`w_hat_min = 0.5`, `margin_min_rad = 0.3`, `r_trust = 0.30`, `delta_v_max = 3e-3`): il codice non le marca ASSUNTE né le deriva; `docs/S3_SPEC.md` §3 e D7 le discute, non riverificato qui.
6. **Se l'executor impone un anticipo minimo**: ho verificato `satellite_intercept.hpp` (`InterceptTrigger`, `validateGraspCommand`) ma non ho letto per intero `prodmp_gazebo_executor_node.cpp`; non vedo un minimo di 5 s lato executor, ma non posso escluderlo.
7. **Orientazione dell'executor** (`QuaternionDMP`) rispetto a SLERP del rollout (D11 di `S3_SPEC`): non verificata qui; se diversa, margine e clearance previsti valutano un'orientazione non identica a quella eseguita.
8. **Ripartizione dei 2.454 s** tra IK statica, rollout, `followMargin` e clearance: solo `clearance_stage_ms` (243.3 ms) è misurato.
9. **Predittore di clearance** ("errore medio 0.35 mm, max 1.46 mm su 17 run"): affermazione in `CLRH:9-12`; i dati (`_diag_1019/interference/predict_distance.py`) non sono stati riletti.
10. **Accuratezza di `q_ref = rollout_q_ref` vs controllore reale**: il commento `SELH:88-89` richiede che coincida con il target nullspace del controllore all'attivazione; la corrispondenza con `franka_arm.ros2_control.xacro:51` citata nello YAML (`config:94`) non è stata riverificata.
11. **Margine 35.00° come tetto**: ho dedotto che 35.00° è il margine iniziale di `q6` a `q0` da `q0` (log) e dal limite URDF; non so se ci siano altri finalisti con un margine di approccio migliore del 35.00° per altri `q0` (dipende da `q_start`, qui fisso).
12. **Fattibilità del rollout dei finalisti #21–#40**: mai valutati nel lazy; non deducibile.
