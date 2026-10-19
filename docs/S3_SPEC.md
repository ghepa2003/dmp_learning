# Specifica verificabile del selettore S3 (`selectGraspS3`)

Stato del sorgente: HEAD `b06134a` (main), `src/satellite_grasp_planner` e `src/haptic_dmp_learning` senza modifiche locali.
Percorsi relativi a `src/` salvo dove indicato. Etichette: **MISURATO** (letto da un file di log/analisi, non rieseguito), **CALCOLATO** (derivato qui a mano dal codice), **ASSUNTO** (scelta senza misura nel repo), **DERIVATO** (formula del codice da altri valori), **NON VERIFICATO**.

Abbreviazioni: `SP` = `satellite_grasp_planner`, `HDL` = `haptic_dmp_learning`. File principali:
`SP/src/core/grasp_s3_selection.cpp` (sel), `SP/src/core/grasp_pose_eval.cpp` (eval), `SP/src/core/grasp_s3_launch.cpp` (launch), `SP/src/core/selection_inputs.cpp` (inputs).

> Nome: nel codice la funzione è `selectGraspS3` (file `grasp_s3_*`), ma `DESIGN_NOTES.md:83`, `GraspCommand.msg:1`, `grasp_s3_launch.hpp:5`, `planGraspAndLaunchS1` e i parametri `s1.*` la chiamano "S1". Qui: S3 = `selectGraspS3`. Vedi §6.

---

## 1. Algoritmo

### 1.1 Ingressi

| Ingresso | Origine nel codice | Note |
|---|---|---|
| `q_start` (seme IK, riferimento nullspace, partenza del percorso) | `in.q0`, `inputs:78-92`: `start_joints_override_rad` > `demo.start_joints_rad` > `RobotModel::readyPose()` | **non** è la postura finale della demo (§6 D1) |
| `p0` | FK(`q_start`), `sel:87-88` (`inputs:95`) | mai `start_position_demo_frame_m` |
| `delta_g_demo` (Δg) | `demo.delta_g_demo_m` (posizione finale − iniziale della demo), `inputs:106` | rifiutato se `‖Δg‖ < 1e-9` (`inputs:108`) |
| `contact_to_end_offset` (off) | `demo.contact_to_end_offset_m`, `inputs:107` | nelle demo di produzione = (0,0,0) (`runs/20261015_095418_fit_demo_20261014T150851/weights_demo_params.yaml`) |
| `w_trans_demo` | `wTransFromJacobian` a `demo.end_joints_rad`, `inputs:118-128` | rifiutato se `end_joints_rad` è null |
| `tau_contact_s` | `demo.t_contact_s` o 0, `inputs:105` | |
| `tau_launch_s` (τ) | `contact_assumed_at_end ? tau_template : tau_contact_s`, `launch:30` | |
| `omega` (ω, con segno) | **parametro** `satellite_rotation_angular_velocity_deg_s` (`node:96-97`, `node:205`); l'odometria lo convalida soltanto (`checkOmegaConsistency`, `node:199`) | −2 deg/s = −0.0349066 rad/s |
| snapshot (θ_snap, t_snap, centro c, asse a) | `PhaseTracker` sul primo campione di odometria, `node:193-207` | `t_now_s` = `snapshot.t_s` di default (`launch:51`) |
| `min_delay_s` (D) | parametro obbligatorio, YAML 0.05 (`config:30`); `tools/s1_run.py:433` usa **5.0** | |
| geometria del collare | `CubeSatelliteModel::Params` da `cube_geometry.*` (`node:84-117`) | §2.1 e §3 |
| `depth` | `cube_geometry.grasp_depth_m`, default 0.0 (`node:117`); `tools/s1_run.py:680` usa **0.0125** | |
| template ProDMP | pesi + `weights_sha256` verificato (`inputs:56-64`) | |
| riferimento demo per `se3Distance` | `demo.position = p0 + Δg`, `demo.orientation = FK(q_start).orientation` (`launch:39-43`) | posizione = fine demo, orientazione = **inizio** (§6 D3); conta solo se `orientation_weight > 0` |

### 1.2 Pseudocodice

```
selectGraspS3(model, robot, q_start, demo, Δg, ctx, params):           # sel:80
  validateS3Params(params); validateCtx(ctx)                              # sel:46-67, sel:26-36
  robot.update(q_start, 0); p0 = robot.eePosition()                       # sel:87-88

  evalOne(k, θ):                                                          # sel:95-124
      c.eval   = evaluateGraspPose(model,k,θ,robot,q_start,demo,l,θs,ik)  # eval:133   ++evaluations
      goal     = c.eval.pose.position_world + off                         # sel:100
      c.d_pos  = ‖(goal - p0) - Δg‖                                       # sel:101
      c.d      = s3Distance(d_pos, eval.dist_to_demo.ori_rad, l, w_o, θs) # sel:102
      # vincoli, in cascata (primo che fallisce = motivo)                 # sel:104-113
      if   !eval.feasible_basic               : reason="ik"
      elif !(eval.w_hat >= w_hat_min)         : reason="w_hat"
      elif !(eval.margins.min_margin >= margin_min_rad): reason="margine"
      elif !(c.d_pos <= r_trust_m)            : reason="r_trust"
      else: passes=true; max_w_hat_passing=max(.., w_hat)
            c.wait = planLaunch(snapshot, t_now, θ, τ, D).delay_s         # sel:115
            c.cost = c.d + ε·c.wait                                       # sel:116
      if !passes and (primo per k, o d_pos < least_bad[k].d_pos): least_bad[k]=c   # sel:118-122
      return c

  step   = theta_step_deg·π/180 ;  refine = theta_refine_step_deg·π/180  # sel:126-127
  n_theta  = ceil(2π/step − 1e-9)      # 180 per step=2°                   sel:128
  n_refine = floor(step/refine + 1e-9) # 20  per 2°/0.1°                   sel:129

  per k in {0,90,180,270} (ordine fisso):                                 # sel:131
     best=∅
     per i in 0..n_theta-1:  θ = −π + i·step     # [−180°,178°]            sel:134
         c=evalOne(k,θ); se passes: accepted.push(c); best=min_lessCost(best,c)
     se best=∅: continue (nessun raffinamento per questo k)               # sel:144
     θb = best.θ
     per j in −n_refine..n_refine, j≠0:                                   # sel:146-150
         c=evalOne(k, θb + j·refine); se passes: accepted.push(c)

  stable_sort(accepted, lessCost)   # cost, poi k, poi θ                  sel:153, sel:38-42
  # post-hoc, in ordine di costo                                          sel:155-236
  se posthoc_velocity_check: ProDMP copia, goal assoluto p0+Δg, cond. iniz. (t=0,p0,v=0),
        n = max(1, ceil(τc/0.005 − 1e-9)) passi da 0.005 s,  τc = tau_contact>0 ? tau_contact : tau_template
  per i in accepted, finché non scelto E posthoc_checks < max_posthoc_candidates:
     se (k,θ) entro posthoc_cluster_deg di uno già controllato (stesso k): salta (non conta)   sel:172-184
     ++posthoc_checks
     se velocity_check: dv = ‖v_t − v_ee‖;  se !(dv <= δv_max): scarta ("velocita' relativa …"), log, continua
     se path_check:  fr = checkKinematicFeasibility(…); se !fr.feasible: scarta ("percorso: violazione limiti …"), log, continua
     accetta: chosen_idx=i; log; stop
  se max_posthoc_candidates==0 e accepted≠∅: chosen_idx=0 (nessun controllo)   sel:238
  stato: accepted=∅ → kNoCandidate | chosen_idx<0 → kPosthocFailed | altrimenti kSelected   sel:256-268
  uscite: ranked = primi 20 di accepted (annotati post-hoc), best_per_k, evaluations, posthoc_log   sel:240-254
```

Dopo `selectGraspS3`, `planGraspAndLaunchS1` (`launch:54-89`) ricalcola `planLaunch` con `t_now = snapshot.t_s + tempo reale di selezione` (`launch:77-78`): il `provisional_delay_s` del messaggio differisce da `chosen.wait_s` di `elapsed_s`. **MISURATO**: 139.799 − 138.28389 = 1.5151 s = `selection_elapsed_s` 1.515108 (`analysis_s1_1015/report_final2.txt`).

### 1.3 Conteggio delle valutazioni (880)

- Griglia grossa: `n_theta = ceil(2π/step − 1e-9) = 180` per k, θ_i = −π + i·2° → 4 × 180 = **720**.
- Raffinamento: `n_refine = floor(2/0.1 + 1e-9) = 20`; j ∈ {−20..20}∖{0} = 40 per k → 4 × 40 = **160**.
- Totale `evaluations` = 720 + 160 = **880**, **solo se ogni k ha almeno un candidato accettato nella griglia grossa**; per ogni k senza accettati −40. Ogni valutazione = 2 `solvePoseIk` (due rami) + 1 `wTransAt` → 1760 IK.
- 880 **MISURATO** in `report_final2.txt` (GraspCommand) e in `analysis_demo_1009/s3_diag.txt`.
- Attenzione per l'audit: j = ±20 cade su θb ± 2° = punti già valutati nella griglia grossa (differenza solo di arrotondamento ~1e-16). I punti θ distinti sono 720 + 4×38 = 872; 8 valutazioni sono duplicate e, se passano, entrano due volte in `accepted`/`ranked` (assorbite dal clustering post-hoc, non da `ranked`). **CALCOLATO**.
- Il raffinamento è intorno al migliore **per costo** (incluso ε·wait) di ciascun k, non per d.
- Il raffinamento può uscire da [−π, π) (θb ± 2°); tutte le formule sono periodiche.

### 1.4 Arresti e stati

| Condizione | Effetto | Riferimento |
|---|---|---|
| scansione | nessun arresto anticipato | `sel:131-151` |
| k senza accettati | nessun raffinamento per quel k | `sel:144` |
| post-hoc | si ferma al primo accettato, a `max_posthoc_candidates` controlli, o a fine lista | `sel:181` |
| `max_posthoc_candidates == 0` | sceglie `accepted[0]` senza alcun controllo (anche con check abilitati) | `sel:238` |
| `kNoCandidate` | `accepted` vuoto → `STATUS_NO_FEASIBLE_CANDIDATE` | `sel:256` |
| `kPosthocFailed` | `accepted` non vuoto, nessuno scelto; **non** significa che tutti i candidati abbiano fallito: solo i primi `posthoc_checks` (es. 10 su 392, **MISURATO** in `s3_diag.txt`) | `sel:259-264` |
| `kSelected` | `chosen = accepted[chosen_idx]` | `sel:266` |

Enum `S3Result::Status` = {kSelected=0, kNoCandidate=1, kPosthocFailed=2} (`hpp:90`); il messaggio usa {1,2,3} (`GraspCommand.msg:11-13`). Il log del nodo stampa il valore dell'enum (`node:226-228`).

---

## 2. Formule (simboli del codice)

### 2.1 Posa di presa con depth — `CubeSatelliteModel::graspPoseAt`, `HDL/src/core/cube_satellite_model.cpp:54-74`

```
a    = normalize(axis_world);  n_b = normalize(face_normal_body)        (ctor cpp:20-21)
R(θ) = AngleAxis(θ, a)                                                   (:39-41)
u    = normalize(n_b × a);  v = a                                        (:43-48)
φ_k  = idx(k)·π/2;   r_k = cos φ_k·u + sin φ_k·v
p_body = (cube_side_m/2 + standoff_m − grasp_depth_m)·n_b + ρ_g·r_k      (:57-59)
ρ_g  = grasp_radius_m  (se non impostato: collar_radius_m)
p_grasp(k,θ) = center_world + R(θ)·p_body                                (:70)
z_tcp = −R n_b ;  y_tcp = −R r_k ;  x_tcp = y_tcp × z_tcp                (:61-63)
R_nom = [x_tcp y_tcp z_tcp]  → quaternione normalizzato                  (:64-71)
ramo "wrist_flip": R_flip = R_nom · Rz(π)   (applyRoll, HDL/include/…/tcp_roll.hpp:16)
goal g_cand = p_grasp + contact_to_end_offset                            (sel:100)
```

Vincolo ctor: `0 ≤ grasp_depth_m ≤ standoff_m` (`cube_satellite_model.cpp:30-32`). `collar_radius_m` (raggio esterno) non entra nella posa se `grasp_radius_m` è impostato. Valori di produzione: 0.075 + 0.08 − depth = 0.155 − depth (con depth 0.0125 → x_c = 0.1425 m dal centro lungo n_b); distanza dall'asse: 0.155 (k=90,270) o √(0.155²+0.068²)=0.1693 (k=0,180) con depth 0 (**CALCOLATO**, coincide con `HDL/test/test_cube_satellite_model.cpp:268-293`).

Nota: il modello **non** usa il raggio ρ = 0.05 m né r = 0.18 m del brainstorming (§6 D5).

### 2.2 Distanza e costo

```
d_pos = ‖(g_cand − p0) − Δg_demo‖                          (sel:101)   [m]
ori   = 2·atan2(‖vec(q1⁻¹ q2)‖, |w(q1⁻¹ q2)|)              (eval:117-118) tra ramo scelto e demo.orientation
d     = sqrt( (d_pos/l)² + w_o·(ori/θ_s)² )                (sel:75-77)   l=length_scale_m, w_o=orientation_weight, θ_s=angle_scale_rad
J     = cost = d + ε·wait_s                                (sel:116)     ε = epsilon_wait_per_s
```
Con `w_o = 0` (default) `d = d_pos/l`. `d_pos` (selettore) ≠ `eval.dist_to_demo.pos_m` (= ‖p_grasp − (p0+Δg)‖) quando `off ≠ 0` (eval:168 non include l'offset).
Scala di ε: ε·180 s = 0.018 ≙ d_pos = 1.8 mm (l = 0.1 m). **CALCOLATO**.

### 2.3 Attesa, θ_trig, finestra

```
θ_now   = wrapPi(θ_snap + ω·(t_now − t_snap))                      (launch_plan.cpp:13-16);  t_now = snapshot.t_s ⇒ θ_now = θ_snap
T       = 2π/|ω|                                                    (launch_delay.cpp:14)   = 180 s per |ω| = 2°/s
t       = (θ* − θ_now)/ω − τ                                        (launch_delay.cpp:15)
r       = fmod(t − D, T);  se r<0: r += T;  se r ≥ T − 1e-9: r = 0  (:16-20)
t_wait  = wait_s = D + r        ∈ [D, D + T)                        (:21)
θ_trig  = wrapPi(θ* − ω·τ)                                          (HDL/include/…/satellite_intercept.hpp:598-600; usato dall'executor)
```
Identità: θ_now + ω·t_wait ≡ θ_trig (mod 2π). Il selettore non calcola θ_trig: usa solo `t_wait`. `wait_s` è calcolato **solo per i candidati che passano i vincoli** (altrimenti 0). Finestra: [D, D+180) s → con D = 0.05 (YAML) fino a 180.05 s; con D = 5.0 (campagne S1) fino a 185 s. Il brainstorming dice "al massimo 180 s" (§6 D4). L'executor non usa `t_wait`: scatta sull'incrocio di fase e, se il primo campione è già oltre θ_trig, attende il giro successivo (`satellite_intercept.hpp:260-262`, 283-286).

### 2.4 Manipolabilità, ŵ, margine

```
w_trans(q) = Π_i σ_i(J_t(q)) = sqrt(det(J_t J_tᵀ))      J_t = righe 0..2 del Jacobiano geometrico, frame fer_hand_tcp, LOCAL_WORLD_ALIGNED   (manipulability.hpp:50-58, eval:103-106)
ŵ          = w_trans(q*) / w_trans_demo                  (eval:166-167);  q* = ik.q del ramo scelto
w_trans_demo = w_trans(end_joints_rad)                   (inputs:127)
margin_j   = min(q*_j − lo_j, hi_j − q*_j)   [rad, ASSOLUTO]    (eval:96)
min_margin = min_j margin_j                              (eval:98)
```
Limiti URDF (`fer_flat_effort.urdf`): j1 ±2.8973, j2 ±1.7628, j3 ±2.8973, j4 [−3.0718, −0.0698], j5 ±2.8973, j6 [−0.0175, 3.7525], j7 ±2.8973.

### 2.5 IK statica e scelta del ramo (origine di ŵ e margine)

`solvePoseIk` (`eval:44-92`): `q ← clamp(q + cap(dq))`, `dq = NullspaceBiasedResolution(kp=0.5 lineare e angolare, λ=damping, saturazioni 1e9, nullspace_gain, joint1_scale=1, nullspace_damping=1e-3)`, `cap`: se `max|dq_j| > max_step_rad` si scala; arresto a `pos_err < 1e-4 m` e `ori_err < 1e-3 rad` o `max_iter`. `converged = limits_ok ∧ errori sotto soglia` (:89-90). Seme e riferimento nullspace: `q_start` per entrambi i rami (:157). Nessun warm-start tra candidati: ogni (k,θ) riparte da `q_start`, quindi il risultato non dipende dall'ordine di valutazione.

Rami: `orient[0] = R_nom`, `orient[1] = R_nom·Rz(π)` (:151-154). `better(a,b)` (:30-40): converge prima; se nessuno converge, minore `pos_err` poi `ori_err`; se entrambi convergono, maggior `min_margin` se differisce di più di 1e-3 rad (`kMarginTieRad`), altrimenti minore `‖q − q_start‖`. Il ramo 1 vince solo se strettamente migliore (:161).

### 2.6 Controllo di velocità (post-hoc) — `sel:191-209`

```
v_ee = prodmp_at_contact.velocityForCandidateGoal(g_cand)       (HDL/src/core/prodmp.cpp:526+)
v_t  = (ω·a) × (p_grasp(k,θ) − center)                          (HDL/include/…/surface_velocity.hpp:19-24)
dv   = ‖v_t − v_ee‖ ;  accettato se  dv ≤ delta_v_max_m_s
```
`velocityForCandidateGoal` sostituisce il termine di goal nella formula di velocità chiusa (lineare nel goal) mantenendo le condizioni iniziali (p0, v=0) e il supporto Δg; non modifica lo stato. Il controllo usa `g_cand` **con** offset.

### 2.7 Controllo del percorso (post-hoc) — `sel:210-230`, `SP/src/core/kinematic_feasibility.cpp:15-102`

```
orient   = wrist_flip ? R_nom·Rz(π) : R_nom
rollout  : ProDMP copia, goal assoluto = p_grasp  (SENZA offset), cond. iniz. (p0, 0), n=ceil(τ_template/0.005) passi
SLERP    : q_i = slerpShortestPath(quat0=FK(q_start), orient, s_i),  s_i = min(1,(i+1)·dt/τ_template)
sim      : JointPathSimulator, NullspaceBiasedResolution con parametri di DEFAULT (nullspace_gain 0.2236, joint1 scale 10,
           VelocityIkSolver kp=1, λ=0.05, v_lin ≤ 0.15 m/s, ω ≤ 0.5 rad/s, v_joint ≤ 1.5 rad/s), Eulero esplicito, riferimento nullspace = q_start
max_viol = max(0, max_{i,j} max(lo_j − q_ij, q_ij − hi_j))
feasible = max_viol < 1e-4 rad
```
Il controllo accetta/rifiuta **solo** sulle violazioni dei limiti; `margin_min_rad` non è applicato lungo il percorso (il margine minimo del percorso è riportato in `posthoc_min_margin_rad`). Gli errori di posizione finale sono diagnostici.

---

## 3. Parametri

Default nel codice = `S3Params` (`SP/include/…/grasp_s3_selection.hpp:33-51`); YAML = `config/grasp_planner_example.yaml` (tutti gli `s1.*` sono **commentati**: righe 57-76; i valori mostrati coincidono con il codice). Dichiarazione nel nodo: `node:132-157`.

| Nome | Codice | YAML | Origine | file:riga |
|---|---|---|---|---|
| `length_scale_m` (l) | 0.10 | 0.10 (:57) | ASSUNTO (scala) | hpp:34 |
| `orientation_weight` (w_o) | 0.0 | 0.0 (:58) | ASSUNTO (scelta: distanza solo posizione) | hpp:35 |
| `angle_scale_rad` (θ_s) | 1.0 | 1.0 (:59) | ASSUNTO | hpp:36 |
| `epsilon_wait_per_s` (ε) | 1e-4 | 1e-4 (:60) | ASSUNTO | hpp:37 |
| `w_hat_min` | 0.5 | 0.5 (:61) | ASSUNTO ("soglia da giustificare" nel brainstorming) | hpp:38 |
| `margin_min_rad` | 0.3 | 0.3 (:62) | ASSUNTO | hpp:39 |
| `r_trust_m` | 0.30 | 0.30 (:63) | ASSUNTO (brainstorming: "qualche cm") | hpp:40 |
| `delta_v_max_m_s` | 3e-3 | 3e-3 (:64) | ASSUNTO; nessuna misura Gazebo nel repo (**NON VERIFICATO**) | hpp:41 |
| `theta_step_deg` | 2.0 | 2.0 (:65) | ASSUNTO (numerico) | hpp:42 |
| `theta_refine_step_deg` | 0.1 | 0.1 (:66) | ASSUNTO (numerico) | hpp:43 |
| `posthoc_path_check` | true | true (:67) | scelta | hpp:45 |
| `posthoc_velocity_check` | true | true (:68) | scelta; **le campagne S1 usano `false`** (`tools/s1_run.py:434`) | hpp:46 |
| `max_posthoc_candidates` | 10 | 10 (:69) | ASSUNTO | hpp:47 |
| `posthoc_cluster_deg` | 10.0 | 10.0 (:70) | ASSUNTO | hpp:50 |
| `ik.pos_tol_m` | 1e-4 | 1e-4 (:71) | ASSUNTO | eval.hpp:26 |
| `ik.ori_tol_rad` | 1e-3 | 1e-3 (:72) | ASSUNTO | eval.hpp:27 |
| `ik.max_iter` | 300 | 300 (:73) | ASSUNTO | eval.hpp:28 |
| `ik.damping` (λ) | 0.05 | 0.05 (:74) | ASSUNTO | eval.hpp:29 |
| `ik.nullspace_gain` | 1.0 | 1.0 (:75) | ASSUNTO | eval.hpp:30 |
| `ik.max_step_rad` | 0.2 | 0.2 (:76) | ASSUNTO | eval.hpp:31 |
| IK `kp` | 0.5 (costante) | — | ASSUNTO (passo unitario) | eval.cpp:48-49 |
| `kMarginTieRad` | 1e-3 (costante) | — | ASSUNTO | eval.cpp:18 |
| `kProjectorDamping` | 1e-3 (costante) | — | ASSUNTO | eval.cpp:21 |
| `kRankedSize` | 20 (costante) | — | ASSUNTO | sel:22 |
| `kDt` (rollout velocità) | 0.005 s | — | DERIVATO (200 Hz, `joint_path_simulator.hpp:135`) | sel:23 |
| path `dt`, tol | 0.005 s, 1e-4 rad | — | DERIVATO/ASSUNTO | kinematic_feasibility.hpp:64-65 |
| path `nullspace_gain` | 0.2236 | — | DERIVATO (0.5·√0.2 dal controllore a impedenza) | joint_path_simulator.hpp:92 |
| path `joint1_nullspace_gain_scale` | 10 | — | DERIVATO (√100) | joint_path_simulator.hpp:97 |
| `min_delay_s` (D) | obbligatorio, senza default | 0.05 (:30) | ASSUNTO (ritardo comunicazione + calcolo); campagne: 5.0 | node:82 |
| `cube_geometry.cube_side_m` | obbligatorio | 0.150 (:37) | MISURATO (CAD, commento in `cube_satellite_model.hpp:34`) — NON riverificato | node:86 |
| `cube_geometry.standoff_m` | obbligatorio | 0.08 (:38) | MISURATO (CAD) — NON riverificato | node:87 |
| `cube_geometry.collar_radius_m` | obbligatorio | 0.0725 (:39) | MISURATO (CAD/SDF) — NON riverificato; non entra in `graspPoseAt` se `grasp_radius_m` impostato | node:88 |
| `cube_geometry.wall_thickness_m` | obbligatorio | 0.009 (:45) | MISURATO (CAD) | node:91 |
| `cube_geometry.grasp_radius_m` | opzionale | 0.068 (:42) | DERIVATO: (0.0725 + (0.0725−0.009))/2 = 0.068 (metà parete) | node:115 |
| `cube_geometry.grasp_depth_m` | 0.0 | commentato (:43) | ASSUNTO; run: 0.0125 (d_pos 17.39 → 7.17 mm, **MISURATO** in `report_final2.txt`) | node:117 |
| `satellite_rotation_angular_velocity_deg_s` | obbligatorio | −2.0 (:21) | impostato nel simulatore (setpoint), convalidato con tol 0.1 deg/s | node:96 |
| `robot_base_world` | obbligatorio | 0 (:31) | vincolo (`checkRobotBaseWorldIsZero`): il codice assume base robot = mondo | node:93 |
| centro satellite | obbligatorio | (0.75,0,0.35) (:20) | YAML esempio; **le campagne usano (0.70,0,0.77)** (`tools/s1_run.py:58`) | node:94 |

### R_trust: uso e possibilità di vincolare d

- **Dove**: unico uso = quarto test della cascata, `sel:110-111` (`!(d_pos <= r_trust_m)` → "r_trust"); validazione `sel:48-49` (>0); nodo `node:138`. Non entra in `d`, `cost`, post-hoc né nel messaggio.
- **Vincola d?** Come filtro su `d_pos` sì, in linea di principio: scarta i candidati con `d_pos > 0.30 m` ⇔ `d > 3.0` (l = 0.1). Sulla circonferenza dei punti di presa (diametro ≤ 2·0.1693 = 0.339 m) esistono θ con `d_pos` > 0.30 se il goal della demo traslato è sul lato opposto (**CALCOLATO**, solo geometria, non provato con l'IK).
- **Nei dati noti non è mai attivo**: scansione grossa (180 θ × 4 k) in `analysis_demo_1009/s3_diag.txt` (e identica in `s1b2_s3diag.txt`, `s1d_s3diag.txt`): scarti `ik=480`, `margine=8`, **`r_trust=0`, `w_hat=0`** su 720 (**MISURATO**, demo stageA). Il vincitore dei run finali ha `d_pos` 7–17 mm (≪ 300 mm). Limiti di questa evidenza: (i) la cascata registra solo il primo motivo, quindi un candidato scartato per `ik` non mostra se avrebbe violato `r_trust`; (ii) solo la griglia grossa; (iii) una sola demo/geometria.
- **Conseguenza sul minimo**: poiché il selettore minimizza `cost ≈ d`, un vincolo `d_pos ≤ R` non attivo sul vincitore non cambia il vincitore; può solo togliere candidati con `cost ≳ 3` dalla lista post-hoc. Con 0.30 m è ~10× più largo di quanto assunto nel brainstorming ("qualche cm"): oggi non è una "trust region" nel senso del brainstorming.

---

## 4. Formula → test

I test sono stati **letti, non eseguiti**. I test del selettore saltano (`GTEST_SKIP`) se manca `$HOME/thesis_ws/fer_flat_effort.urdf` o se la fixture non produce candidati (`test_grasp_s3_selection.cpp:26-31`, :118, :163, :180); usano un ProDMP sintetico, Δg = (0.1,0,0), 5°/1°, `looseParams()` con **path check e vincoli spenti** (:82-92).

| Formula / comportamento | Test | Copertura |
|---|---|---|
| Posa di presa p_grasp, R_nom (k, θ) | `GraspPoseEvalTest.GraspPoseConventionByHand` (valori a mano, θ=0 e 90°); `CubeSatelliteModelRealGeometry.RimPointsMatchHandComputedCadDimensions`; `CubeSatelliteModelPositions.ThetaZeroDefaultParamsMatchAnalytical`; `ThetaConventionTest.ModelGraspPointRotatesRigidlyWithTheta` | sì |
| `grasp_depth_m` | `CubeSatelliteModelGraspDepth.DefaultZeroUnchangedAndDepthShiftsAlongCollarAxis`, `.RejectsDepthOutsideZeroToStandoff` | sì |
| `grasp_radius_m` non impostato = `collar_radius_m` | `CubeSatelliteModelRealGeometry.UnsetGraspRadiusEqualsCollarRadius` | sì |
| Ramo flip `R_nom·Rz(π)` (`applyRoll`) | `TcpRoll.AxesAndComposition` | sì |
| `se3Distance` (ori geodetica) | `GraspPoseEvalTest.Se3Distance` | sì |
| `s3Distance` | `GraspS3Test.DistanceIgnoresOrientationWithZeroWeight` (valore 0.5 a mano con w_o=0; con w_o>0 solo disuguaglianza) | parziale: la forma sqrt(a²+w_o b²) con w_o>0 non è verificata numericamente |
| `cost = d + ε·wait` | `GraspS3Test.WaitTimeWithinOnePeriod` (EXPECT_NEAR) | sì (solo sul vincitore) |
| `launchDelay`, finestra [D, D+T) | `LaunchDelay.HandComputedAndRange`; `LaunchPlanPlan.MinDelayWrapsOnePeriod`; `GraspS3Test.WaitTimeWithinOnePeriod` | sì |
| `phaseAt`, `planLaunch` (fase al contatto = θ*) | `LaunchPlanPhase.KnownValuesAndWrap`, `LaunchPlanPlan.BaseCase/FasterRotation/ContactPhaseAlwaysMatchesTarget` | sì |
| θ_trig = wrapPi(θ* − ωτ) | `GraspCommandTheta.ThetaTrigIsWrapPiOfThetaStarMinusOmegaContact`; `SatelliteInterceptTrigger.ThetaTrigIsModulo360` (HDL) | sì (lato executor); equivalenza `t_wait` ↔ θ_trig coperta solo indirettamente da `ContactPhaseAlwaysMatchesTarget` |
| `w_trans = Πσ_i` | `ManipulabilityTest.MatchesValidatedPythonValuesForAllFiveGoals`; `GraspPoseEvalTest.WTransAtMatchesManipulability` | sì |
| ŵ = w_trans/w_trans_demo | `GraspPoseEvalTest.EvaluateGraspPoseEndToEnd` | sì |
| `w_trans_demo` da `end_joints_rad` | `SelectionInputsTest.WTransDemoFromEndJoints`, `.NoEndJointsIsRejectedAskingToRedoTheFit` | sì |
| Margini di giunto | `GraspPoseEvalTest.JointMargins` | sì |
| Precedenza q0 / p0 da FK | `SelectionInputsTest.P0FromKinematics`, `.StartOverrideWinsOverDemoStartJoints`, `StartState.*` | sì |
| `tau_contact`, Δg, offset | `SelectionInputsTest.ContactTime`, `.ReferencesAndProvenance`, `.ZeroDeltaGThrows` | sì (lettura); non l'uso nel selettore |
| `solvePoseIk` (convergenza, limiti, irraggiungibile) | `GraspPoseEvalTest.IkReachesFkGeneratedPose`, `.UnreachablePoseDoesNotConverge`, `.ResultNeverOutsideLimits` | sì |
| Scelta del ramo `better()` | `GraspPoseEvalTest.WristFlipIsConsistent` | **parziale**: solo "converge prima" e determinismo; spareggio per margine (>1e-3) e per ‖q−q_start‖ **NESSUN TEST** |
| `surfaceVelocity` | `SurfaceVelocity.MagnitudesDirectionsAndSign`, `.MagnitudeIndependentOfTheta` | sì |
| `velocityForCandidateGoal` | `ProDMPCandidateGoalTest.CurrentGoalReproducesVelocity`, `.DifferenceIsLinearInGoalWithKnownFactor`, `.ThrowsBeforeAnyStepOrWhenNotLearned`, `.DoesNotAlterState` | sì |
| dv = ‖v_t − v_ee‖ e rifiuto | `GraspS3Test.PosthocVelocityRejectsFirstCandidate`, `.PosthocChecksLimitedToMaxAndLogged` (dv = ‖v_t−v_ee‖) | sì (soglia estrema 1e-6; confine di accettazione non testato) |
| Preparazione di `prodmp_at_contact` (supporto p0+Δg, passi fino a τc) | — | **NESSUN TEST** (nessun v_ee ricalcolato in modo indipendente) |
| `checkKinematicFeasibility` isolato | `KinematicFeasibilityTest.NearbyTargetFromReadyPoseIsFeasible`, `.StartBeyondJointLimitIsInfeasibleWithExactViolation`, `.DoesNotModifyProDmpTemplate` | sì |
| Controllo del percorso **dentro** `selectGraspS3` (ramo flip, rifiuto "percorso…") | — | **NESSUN TEST** (`looseParams` pone `posthoc_path_check=false` e nessun test lo riattiva) |
| Clustering e limite di controlli | `GraspS3Test.PosthocClusterSkipsNearbyThetaOfSameK`, `.PosthocChecksLimitedToMaxAndLogged` | sì |
| Stati kNoCandidate / kPosthocFailed / kSelected | `GraspS3Test.ImpossibleConstraintGivesNoCandidate`, `.PosthocChecksLimitedToMaxAndLogged`, `.PosthocVelocityCheckDisabledSelectsCheapest` | sì |
| Ordinamento per costo, `ranked` ≤ 20 | `GraspS3Test.WinnerPassesConstraintsAndIsCheapest` | sì (spareggio per k poi θ: **NESSUN TEST**) |
| Determinismo | `GraspS3Test.Deterministic` | sì |
| `d_pos = ‖(g_cand − p0) − Δg‖` | — | **NESSUN TEST** (compare solo come `d_pos ≤ r_trust` e come `d = d_pos/l`) |
| Cascata vincoli: ordine ik→w_hat→margine→r_trust e stringhe `reject_reason` | — | **NESSUN TEST** (il solo `w_hat_min=5` in `ImpossibleConstraintGivesNoCandidate` non verifica il motivo) |
| Rifiuto per `margine` e per `r_trust` | — | **NESSUN TEST** |
| Griglia: `n_theta`, `n_refine`, conteggio 880, raffinamento solo per k con accettati | — | **NESSUN TEST** (nessuna asserzione su `evaluations`, solo `> 0` e uguaglianza fra due run) |
| `least_bad` / `best_per_k` quando nessun candidato passa | `GraspS3Test.BestPerKHasFourDistinctPoints` (solo le etichette k) | **NESSUN TEST** del contenuto (reason, d_pos minimo) |
| `max_w_hat_passing` | — | **NESSUN TEST** |
| `max_posthoc_candidates == 0` (sceglie `accepted[0]`) | — | **NESSUN TEST** |
| `planGraspAndLaunchS1` (riferimento demo `p0+Δg`/orientazione FK(q0), τ_launch, goal = pos + off, `t_now` = snapshot + elapsed, controllo snapshot) | nessun test chiama `planGraspAndLaunchS1` (`grep` su `SP/test`, `HDL/test`) | **NESSUN TEST** |
| Mappatura su `GraspCommand` | `GraspCommandConversionTest.*` | sì (campi), non la selezione |

**Formule del codice non coperte** (riassunto): `d_pos`; cascata dei vincoli e rifiuti `margine`/`r_trust`; conteggio/raffinamento della griglia; spareggi di `better()` (margine, ‖q−q_start‖) e di `lessCost`; preparazione di `prodmp_at_contact`; path check dentro il selettore; `least_bad`; `max_w_hat_passing`; `max_posthoc_candidates==0`; `s3Distance` con `w_o>0` in valore; tutta `planGraspAndLaunchS1`.

---

## 5. Contatori di scarto: cosa emette il planner

In `S3Result` (`hpp:89-102`): `evaluations`, `max_w_hat_passing`, `posthoc_checks`, `posthoc_log` (un `S3PosthocEntry` per controllo: k, θ, wait, dv, v_ee, v_target, path_ok, min_margin, accepted, reason), `ranked` (≤ 20 passanti, annotati), `best_per_k[4]`, `note`.

- **Contatori per motivo** (ik / w_hat / margine / r_trust): **non esistono**. Il motivo è una stringa in `S3Candidate::reject_reason` (`sel:105-111`) ma il candidato scartato viene conservato solo se è il "meno cattivo" del suo k (`least_bad`, minimo `d_pos` fra gli scartati, **indipendente dal motivo**, `sel:118-122`) e compare in `best_per_k` solo se quel k non ha passanti (`sel:252`). Gli altri scarti sono persi.
- **Scarti post-hoc** (velocità, percorso): per candidato in `posthoc_log` (solo i controllati, non i saltati dal clustering) e nelle stringhe `"velocita' relativa X m/s > Y"`, `"percorso: violazione limiti X rad (giunto J)"` (J a base 0, `kinematic_feasibility.hpp:26`).
- **Nodo**: logga solo `status` (valore enum 0/1/2), `evaluations`, `posthoc_checks`, `dv`, `path_margin`, `note` (`node:225-230`). `GraspCommand` porta solo il vincitore.
- **Esportazione per candidato (k, θ, J, motivo) senza toccare la logica**: **non c'è flag né parametro** (nessun `s1.dump_*`). Esistono strumenti esterni, tutti non tracciati da git e fuori da CMake (`git status`: `?? analysis_*`), con percorsi fissi `/root/thesis_ws`:
  - `analysis_demo_1009/s3_diag.cpp` e `analysis_s1_1013/s3_diag.cpp`: eseguono `selectGraspS3` e stampano testo (vincitore, `posthoc_log`, `best_per_k`, `ranked`); la parte "coarse grid" **reimplementa** la cascata (`s3_diag.cpp` riga ~190) e conta i motivi sulla sola griglia grossa. Niente CSV, niente raffinamento, niente `wait`/J per gli scartati, e la cascata duplicata può divergere dal codice. L'etichetta di stato in `analysis_s1_1013/s3_diag.cpp:31-33` (0/1/2) non coincide con il messaggio (1/2/3).
  - `analysis_demo_1009/s1a_diag.cpp` → `s1a_diag.csv`: CSV per candidato su griglia 5° con campi di `evaluateGraspPose` (converged, margin, w_trans, w_hat, `dist_to_demo.d/pos/ori`, q1..q7), ma **riferimento diverso** (demo = FK(end_joints), non p0+Δg), nessun `d_pos` del selettore, nessun vincolo/motivo/wait/J.
  - `analysis_s1_1015/calc_top5*.cpp`: classifica "top 5" con una **reimplementazione** (21 punti di raffinamento attorno a *ogni* punto grosso, `wait` ricalcolato a mano, `wrist_flip` fissato a `false` e stampato "nominale" senza misurarlo: `calc_top5_final2.cpp:123,163`). Non è il ranking di `selectGraspS3`.

---

## 6. Discrepanze note codice ↔ documentazione

Brainstorming = `~/Desktop/materiale_tesi/riassunti-checklist/Brainstorming ottimizzazione posa sul collare.md` (7 ott 2026, fuori repo).

| # | Codice | Documentazione | Impatto |
|---|---|---|---|
| D1 | Seme IK e riferimento nullspace = `q_start` = `in.q0` (override > `start_joints_rad` > readyPose), entrambi i rami (`eval:157`, `launch:54`, `inputs:78-92`); documentato in `DESIGN_NOTES.md:238` | Brainstorming: "seme = postura finale della demo, stesso ramo della demo" | `end_joints_rad` serve **solo** per il denominatore di ŵ (`inputs:124-127`). Nelle demo: inizio ≈ readyPose (q7 = 0.785), fine q7 = 1.21 e q5 = −0.33 (`weights_demo_params.yaml`): il ramo/ŵ può cambiare. ŵ e margine sono del ramo raggiunto da `q_start`, non del ramo della demo. |
| D2 | Margine assoluto in rad, soglia 0.3 rad (`eval:96`) | Brainstorming: normalizzato sul range `(q−q_min)/range` | soglie non confrontabili |
| D3 | `d` solo posizione (`w_o = 0`); riferimento = (p0+Δg, **orientazione di FK(q_start)**) (`launch:41-42`); l'header di `DemoReference` dice "posa finale della demo" | Brainstorming: `d` SE(3) con `‖log ΔR‖`, ℓ e ϑ | irrilevante finché `w_o = 0` |
| D4 | `t_wait ∈ [D, D+T)`; nel costo usa `t_now = snapshot.t_s` (`launch:51`), il lancio finale `+ elapsed` (`launch:77`); l'executor scatta sull'incrocio di fase | Brainstorming: `wrap(θ_trig − θ_ora)/ω`, "al massimo 180 s" | max 180.05 s (YAML) / 185 s (campagne); differenza costo-vs-lancio ≈ tempo di selezione (1.5 s **MISURATO**) |
| D5 | Raggio presa 0.068, bordo 0.155 → r = 0.155 / 0.1693 m, v = 5.41 / 5.91 mm/s (`test_surface_velocity.cpp:45-48`) | Brainstorming: ρ = 0.05, r = 0.18 / 0.1868, v = 6.28 / 6.52 mm/s | numeri del brainstorming da aggiornare |
| D6 | τ delle demo: 33.465 s (stageA), 24.474 s (demo finale) | Brainstorming: t_c ≈ 73.8 s | dato, non codice |
| D7 | `r_trust_m = 0.30`; `w_hat_min = 0.5`; `margin_min_rad = 0.3`; `delta_v_max = 3e-3` | Brainstorming: R_trust "qualche cm", soglie da scegliere | vedi §3 |
| D8 | Esiste un controllo a posteriori sul percorso (limiti di giunto) | Brainstorming: "il percorso non è oggetto dell'ottimizzazione", solo la velocità è a posteriori | funzionalità aggiunta, da descrivere in tesi |
| D9 | Nome: `selectGraspS3` / "S1" nei commenti e parametri | Brainstorming: S1 = scansione solo θ che massimizza ŵ; S3 = più vicino alla demo | rischio di confusione nei riferimenti |
| D10 | IK, ŵ, margine e path check usano `position_world` **senza** `contact_to_end_offset`; `d_pos`, velocità e goal comandato lo includono (`sel:100`, `sel:193` vs `sel:216`; `DESIGN_NOTES.md:215-217`) | — | nullo se off = 0 (demo di produzione: [0,0,0]); latente altrimenti |
| D11 | L'executor ignora l'orientazione del planner e usa `QuaternionDMP` (`DESIGN_NOTES.md:209-212`) | — | ŵ, margine e path check valutano un'orientazione non comandata |
| D12 | Le campagne S1 disattivano il controllo di velocità (`tools/s1_run.py:434`) e usano D = 5 s | Default del nodo: velocità attiva | i risultati delle campagne non hanno il vincolo dv |
| D13 | `kPosthocFailed` = esauriti `max_posthoc_candidates` controlli, non tutti i passanti | nome/commento "all post-hoc candidates failed" | vedi `s3_diag.txt` (10 su 392) |
| D14 | Selezione ≈ 1.3–1.5 s **MISURATO** (`s3_diag.txt`, `report_final2.txt`) | `grasp_planner_node.hpp:9`: "~5.2 s Release / ~500 s" | commento obsoleto |
| D15 | `RIM_X_C_MM = 155.1` in `tools/s1_run.py:61` | codice: 75 + 80 = 155.0 mm | 0.1 mm, origine **NON VERIFICATO** |

---

## 7. Paesaggio dei candidati (k, θ): dove nasce ŵ e cosa si può esportare

### 7.1 Dove e come si calcola ŵ

- Punto: `evaluateGraspPose`, `eval:166-167` (`w_trans = wTransAt(robot, ev.ik.q)`, `w_hat = w_trans/demo.w_trans_demo`).
- Catena per candidato (k, θ): posa di presa (`eval:149`, §2.1) → due orientazioni (nominale, flip; `eval:151-154`) → `solvePoseIk` per ciascuna con **seme `q_start`** (`eval:157`) → ramo scelto da `better()` (`eval:161`) → ŵ **sul ramo scelto**. Il ramo non scelto (q, margine, ŵ) è locale a `evaluateGraspPose` (`br[2]`) e viene scartato: nessun campo lo riporta.
- È una grandezza **per candidato** (dipende da k, θ, ramo, seme); l'unico valore unico è il denominatore `w_trans_demo` (`inputs:127`). Per un candidato con IK non convergente ŵ è calcolato comunque sul q finale non convergente (non significativo; la cascata lo scarta prima come "ik").
- ŵ non entra nella scelta del ramo (il ramo dipende da convergenza, margine, ‖q−q_start‖).

### 7.2 Flag/tool esistente per CSV su OGNI candidato

**No.** Non esiste un flag né un tool tracciato. `s3_diag` esiste solo come programma ad hoc non tracciato (§5) che non scrive CSV per candidato; `s1a_diag` scrive un CSV ma con riferimento e colonne diversi. `selectGraspS3` non conserva i candidati scartati (§5).

Cosa è già disponibile senza modifiche (strumento esterno, API pubblica): chiamare `evaluateGraspPose` su una griglia e ricalcolare fuori dalla libreria `d_pos`, cascata, `planLaunch` (come fa `s3_diag.cpp`). Poiché ogni valutazione riparte da `q_start`, i valori coincidono con quelli del selettore indipendentemente dall'ordine; la cascata però sarebbe duplicata (rischio di deriva) e il raffinamento (che dipende dal migliore per k) andrebbe riprodotto a mano.

### 7.3 Modifica minima proposta (NON implementata)

Obiettivo: un record per **ogni** chiamata a `evalOne` (880 nel caso nominale), stessa logica di selezione.

1. `SP/include/satellite_grasp_planner/core/grasp_s3_selection.hpp`: dopo `S3PosthocEntry` (riga 87) aggiungere `struct S3EvalRecord { k, theta_rad, bool refine_stage, d_pos_m, d, w_trans, w_hat, bool wrist_flip, bool ik_converged, bool limits_ok, ik_pos_err_m, ik_ori_err_rad, ik_iterations, min_margin_rad, worst_joint, wait_s, cost, bool ik_ok, w_ok, margin_ok, r_ok, bool passes, std::string reject_reason; }` e in `S3Context` (dopo riga 61) `std::vector<S3EvalRecord>* eval_log = nullptr;` (puntatore: `ctx` resta `const&`).
2. `SP/src/core/grasp_s3_selection.cpp`: aggiungere a `evalOne` (riga 95) il parametro `bool refine_stage` (chiamate a `sel:136` → `false`, `sel:148` → `true`) e, prima di `return c;` (riga 123), `if (ctx.eval_log) ctx.eval_log->push_back(...)`. I quattro booleani si calcolano con gli stessi operatori della cascata (`!(x >= y)` ecc.) senza alterarla. `wait_s` per gli scartati: se `eval_log` è attivo, chiamare `planLaunch` separatamente solo per il record (senza toccare `c.wait_s`/`c.cost`, che restano 0 per gli scartati).
3. Esito post-hoc: già in `res.posthoc_log` (join su (k, θ) bit-identico, perché `entry.theta_rad = c.eval.theta_rad`, `sel:189`).
4. Scrittore CSV: nuovo `SP/analysis/s3_landscape.cpp` (carica `buildSelectionInputs` come `launch:15-53`, popola `eval_log`, scrive colonne: k, theta_deg, stage, d_pos_m, d, w_hat, wrist_flip, min_margin_rad, worst_joint, wait_s, J, ik_ok, w_ok, margin_ok, r_ok, passes, reject_reason, dv, path_ok, post-hoc_reason), registrato in `SP/CMakeLists.txt` dentro `if(BUILD_ANALYSIS_TOOLS)` (righe 125-150), che è già disattivato di default. Opzione ulteriore: parametro di nodo `s1.dump_csv_path` in `node:150` e passaggio di `ctx.eval_log` in `launch:45-52`.
5. Verifica di non-regressione: `GraspS3Test.Deterministic` più un confronto `chosen`/`ranked`/`evaluations` con e senza `eval_log` sullo stesso input.

---

## 8. Cosa NON ho potuto verificare

- **Esecuzione**: non ho compilato né eseguito alcun gtest o binario (niente ROS/Pinocchio qui); i nomi dei test e ciò che asseriscono sono letti dal sorgente. Non so quali test siano oggi verdi, né se saltino per URDF mancante.
- **Valori MISURATI** (880, 1.515 s, 7.17 mm, contatori di scarto `ik=480 margine=8`, 10 su 392): riportati da file non tracciati (`analysis_s1_1015/report_final2.txt`, `analysis_demo_1009/s3_diag.txt`), non rieseguiti; prodotti con revisioni del codice diverse da HEAD (es. `posthoc_cluster_deg` introdotto in `b97e081`) e con altre demo/geometrie (stageA, centro (0.75,0,0.77)).
- **r_trust**: nessun run con `r_trust` attivo; evidenza solo da griglia grossa e dal primo motivo di scarto; la tesi geometrica (§3) è calcolata, non provata con l'IK.
- **Origini ASSUNTO**: non ho trovato nel repo né nel brainstorming una giustificazione per i valori di `w_hat_min`, `margin_min_rad`, `r_trust_m`, `delta_v_max`, ε, l; le quote CAD (0.150, 0.08, 0.0725, 0.009) non sono state riconfrontate con CAD/SDF (esiste `tools/check_collar_sdf.py`, non eseguito).
- **Codice non letto per intero**: `franka_cartesian_control` (`computePoseError`, `RobotModel`, convenzione del Jacobiano `LOCAL_WORLD_ALIGNED`, `VelocityIkSolver::solve`), la formula chiusa di `ProDMP` oltre l'intestazione di `velocityForCandidateGoal`, `PhaseTracker` e `demo_params` (letto solo l'header). `wrapPi` letto (`math_utils.hpp:23-27`).
- **Executor**: il comportamento del trigger è dedotto da `satellite_intercept.hpp` e dai commenti del messaggio, non dal nodo in esecuzione.
- **Brainstorming**: letto il `.md`; il PDF omonimo non confrontato. Non ho verificato se il tutor abbia deciso diversamente dopo il 7 ott.
- **Build/install**: non ho controllato che le librerie in `build_cc/`/`install/` usate nelle campagne corrispondano a HEAD.

---

## 9. Aggiornamento: log per candidato implementato (non compilato)

Implementato il §7.3 (i `file:riga` dei §1-§7 si riferiscono al sorgente PRIMA di questa modifica; i riferimenti `node:` sono stati corretti di −91 righe, erano contati sul listato concatenato con il YAML): `S3Context::eval_log` (default `nullptr`), `S3EvalRecord`, `S3RejectBit`, `s3AprioriRejectMask`, `s3DPos` (la cascata ora vive in una sola funzione, usata da `evalOne` e dal tool), `evaluateGraspPoseDetailed` (con roll ψ; `evaluateGraspPose` ne è un wrapper a ψ=0, bit-identico), `fullManipulability`, `planGraspAndLaunchS1(..., eval_log)`, tool `analysis/s3_landscape.cpp`. Vedi `analysis/README.md`.
