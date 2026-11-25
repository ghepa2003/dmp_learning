# hw/ — set-up a due Franka: stadio 1 (robot 2 "satellite") e stadio 2 (task layer del robot 1)

Contenuto:

| file | ruolo |
|---|---|
| `hw_config.yaml` | unica configurazione (scena, mockup, robot 1, robot 2, UDP, timing, percorsi, planner) |
| `hw.py` | strumenti utente: `status`, `ik`, `reset`, `plan`, `run` (`python3 hw/hw.py --help`) |
| `../tools/satellite_node.cpp` | processo del robot 2: ruota J7, pubblica lo stato via UDP |
| `real_replay --mode task` | processo del robot 1 (stadio 2): VALIDATE, START al satellite, trigger di fase, avvicinamento ProDMP |
| `../config/task_robot1_hand.yaml`, `..._dry.yaml` | config real_replay del task layer (robot e dry-run) |
| `examples/grasp_command_nominale.yaml` | GraspCommand di esempio |
| `examples/weights_sintetico_test.yaml` | pesi **sintetici, solo per i test** (contiene solo `tau`; non è un file di pesi reale) |
| `test/` | test offline (`ctest`: `hw_plan`, `satellite_dryrun_e2e`, `task_dryrun_e2e`) |

Nessun ROS. Tutto gira nel container `franka` (`/root/thesis_ws/real_replay`).

## Convenzioni

- **Frame 1** = base del robot 1. Tutto ciò che è pubblicato è espresso in questo frame.
- La base del robot 2 nel frame 1 è `scene.base2_in_base1` = `{t_m, yaw_deg}`, con `p1 = Rz(yaw)·p2 + t`. Il valore (0, 0.975, 0), yaw 0, è **assunto**.
- La FK è il modello DH del Panda (Craig, flangia d = 0.107). È la stessa in `hw.py` e in `include/real_replay/satellite_phase.hpp`. Verificata sulla posa validata: flangia (0, −0.255, 0.8035) nel frame 2.
- Centro del satellite: `c = p_flangia + flange_to_center_m · z_flangia`.
- Asse pubblicato: `axis = −z_flangia` (verso l'alto). ω > 0 = antiorario visto dall'alto, ω < 0 = orario.
- `scene.axis_tilt_deg = [a, b]` dà `axis_up = Ry(b)·Rx(a)·[0,0,1]` nel frame 1.
- Fase: `θ = atan2(x_y, x_x) + collar_phase_offset_deg`, con x = asse x della flangia nel frame 1.
- Formula da q7: `θ_q7 = q7_phase.offset_deg + q7_phase.sign · q7`, nominale `−135 − q7`.
  - Con la FK DH, q1 = −90° e yaw 0, le due formule coincidono (scarto 0.000°) su tutta la finestra.
  - Con un yaw della base ≠ 0 differiscono di yaw. `satellite_node` lo segnala con il flag `phase_mismatch` (solo avviso).
- Ogni fase sta sul **ramo [−270°, +90°)**, non su wrapPi. La finestra `psi_window_deg = [−180, 0]` ha un bordo proprio sul salto ±180° di wrapPi.
- Finestra in q7: `robot2.q7_window_deg = [−135, 45]`, equivalente a ψ ∈ [−180, 0]. Con ω < 0, q7 cresce.
- La posa validata (q7 = −135°, ψ = 0) è **sul bordo** della finestra. La partenza configurata è ψ0 = −10° (q7 = −125°), dentro il margine di 5°.

## hw.py

| comando | cosa fa |
|---|---|
| `status [--sat] [--no-robot]` | `echo_robot_state` per ogni robot (timeout 8 s, prima riga JSON). Stampa `robot_mode`, q, flangia (`O_T_EE·F_T_EE⁻¹` e FK DH) e la distanza dalla posa di partenza. Con `--sat` ascolta 2 s la porta monitor. |
| `ik [--psi0-deg X]` | IK offline del robot 2: minimi quadrati smorzati, spazio nullo verso il seme su J1–J6 (J7 libero = fase), q dentro i limiti. Rifiuta (exit 1) se un margine è < `ik_limit_margin_deg` o se l'errore supera 0.1 mm / 0.01°. |
| `reset [--robot 1\|2\|both] [--yes]` | Un robot alla volta: q attuale → q obiettivo, conferma, poi `joint_point_to_point_motion <ip> q1..q7 [rad] <speed-factor>`. L'Invio che l'esempio aspetta lo manda hw.py dopo la propria conferma. Rifiuta se `robot_mode` ≠ `Idle`, con il motivo per Reflex / User stopped / Other. |
| `plan` | Fattibilità del GraspCommand, senza muovere nulla (sotto). |
| `run --only-satellite [--dry-run] [--max-travel-deg D]` | Stadio 1: avvia **solo** `satellite_node`; START, heartbeat a `udp.hb_send_hz` e stato live li gestisce hw.py. |
| `run [--dry-run] [--grasp-command F] [--weights F] [--fault-linger-s S]` | Stadio 2, sequenza completa (sotto). `--no-plan-gate` è solo per i test. |

Ctrl-C durante `run` o `reset`: hw.py manda **un** SIGINT al figlio e attende la sua rampa di stop. SIGKILL solo dopo 15 s.

`--yes` arriva a `satellite_node` solo se stdin è un terminale e solo dopo la conferma di hw.py.

Exit code di hw.py:
- 0 = ok;
- 1 = controllo fallito / rifiutato / annullato;
- 2 = errore di config o d'ingresso;
- `run --only-satellite` = exit code di `satellite_node`;
- `run` = exit code di `real_replay --mode task` se ≠ 0, altrimenti 0 se anche `satellite_node` ha chiuso con 0, 7 se no.

### Controlli di `plan`

Notazione:
- `s = sgn(ω)`
- `T_c = τ + timing.contact_delay_s`
- `θ_trig = wrapPi(θ* − ω·T_c)`, riportato sul ramo [−270, 90) per i confronti

Controlli bloccanti (le lettere A–E sono le stesse di `real_replay --mode task`, `task::feasibility()`):
1. `status == SELECTED`, `format == grasp_command_v2`, `frame_id == world`, `start_pose`/`goal_pose` presenti e finiti.
2. `sha256(weights) == weights_sha256`.
3. `|contact_time_s − τ| ≤ plan.tau_tol_s` (1e-6). τ è la chiave `tau` dei pesi, la stessa di `prodmp_io::loadProDmpFromYaml`.
4. `|ω_comando − scene.omega_deg_s| ≤ plan.omega_tol_deg_s` (0.1°/s).
5. ω ≠ 0. Solo avviso se |ω| non è in `plan.omega_domain_deg_s` = {2, 3}°/s.
6. **A**: `s·(θ_trig − θ0) ≥ |ω|·(ramp_s/2 + omega_window_s) + |ω|·timing.margin_arm_s`.
   - `|ω|·(ramp_s/2 + omega_window_s)` è l'angolo percorso prima che `omega_valid` diventi vero.
   - `ramp_s` e `omega_window_s` vengono da `robot2.control`.
7. **B** (armamento): `s·(θ_trig − θ0) ≥ |ω|·T_arm_min_s`.
8. **C**: θ0 dentro la finestra con margine. In `plan` θ0 = θ dalla FK della posa di partenza del robot 2 (`robot2.start`: q fisso o IK); in `real_replay --mode task` θ0 è la fase **misurata** del satellite in WAIT_START.
9. **D**: `θ_end = θ* + ω·(T_close_max + T_follow + T_ramp)` dentro la finestra con margine.
10. **E** (solo avviso): |ω| in `plan.omega_domain_deg_s` = {2, 3}°/s.

Stampa anche:
- l'intervallo ammesso di θ0 (finestra con margine ∩ A ∩ B);
- il timeout del trigger dall'invio di START, `(θ_trig − θ0)/ω + ramp_s/2 + trigger_timeout_margin_s` (la rampa smoothstep del satellite fa perdere `ramp_s/2` secondi);
- "portata robot 1: NON verificata". Il punto di estensione è `check_robot1_reach()`.

`timing.contact_delay_s` vale **0**: come l'executor del simulatore (`satellite_intercept.hpp`, `plannerThetaTrigRad`), `θ_trig = wrapPi(θ* − ω·τ)`. Il braccio arriva al goal (t = τ) quando la presa ci arriva. `provisional_delay_s` del GraspCommand non è un ritardo di contatto: è l'attesa stimata fra lo snapshot del planner e lo scatto, solo diagnostica, ignorata dall'executor. Il valore effettivo lo stampa `real_replay --mode task` a fine run ("implied effective contact_delay_s").

Con la config e il comando di esempio (τ = 21.466 s sintetico):
- θ_trig = −83.9 + 2·21.466 = −40.968°;
- A = 2·(1 + 5 + 2) = 16°;
- θ0 ammesso ∈ [−24.968, −5.000]°;
- θ0 = −10° → FATTIBILE;
- θ_end = −83.9 − 2·(6 + 5 + 2) = −109.9°;
- timeout del trigger = 15.484 + 1 + 5 = 21.484 s.

## Formato YAML d'ingresso

**GraspCommand** (`format: grasp_command_v2`). Rispecchia `satellite_grasp_msgs/GraspCommand.msg`, ma usa i gradi. `real_replay` li converte in radianti **una sola volta**, in `task::loadGraspCommand()`:

```yaml
format: grasp_command_v2
status: SELECTED              # SELECTED | NO_FEASIBLE_CANDIDATE | POSTHOC_FAILED | UNSET (msg.status)
frame_id: world               # msg.header.frame_id; world = base del robot 1
stamp_unix_s: 1764000000.0    # tempo di parete di creazione (msg.header.stamp); VALIDATE: non nel futuro (tol 1 s), età <= task.cmd_max_age_s
theta_star_deg: -83.9         # msg.theta_star_rad in gradi, ramo [-270, 90)
contact_time_s: 21.466        # msg.contact_time_s (= tau dei pesi)
omega_deg_s: -2.0             # msg.omega_rad_s in gradi
weights_sha256: "<hex>"       # msg.weights_sha256
start_pose:                   # msg.start_position + orientazione (estensione: il msg ha solo la posizione)
  position_m: [x, y, z]       # TCP del robot 1 da cui il planner ha validato il rollout, frame 1
  quaternion_wxyz: [w, x, y, z]
goal_pose:                    # msg.goal_pose: TCP a fine approach (t = tau), frame 1
  position_m: [x, y, z]
  quaternion_wxyz: [w, x, y, z]
k: 0                          # msg.k, GraspPointId 0..3 (informativo nello stadio 2)
wrist_flip: false             # msg.wrist_flip (informativo nello stadio 2: l'orientazione è già in goal_pose)
```

**Weights**: il file ProDMP reale (`weights.yaml` di `runs/...`). `plan` legge solo `tau` e lo sha256 del file intero. Il file `examples/weights_sintetico_test.yaml` è un sostituto sintetico per i test (`synthetic_test_only: true`).

## real_replay --mode task (stadio 2)

```
real_replay --config config/task_robot1_hand[_dry].yaml --mode task --grasp-command F --hw-config hw/hw_config.yaml
            [--weights F] [--dry-run] [--yes] [--fault-linger-s S]
```

Stati (`include/real_replay/task_logic.hpp`):
- INIT → WAIT_CMD → VALIDATE → WAIT_TRIGGER → APPROACH → HOLD_SAFE → DONE.
- Guasti: FAULT_NOMOTION (prima dell'avvicinamento) e FAULT_MOTION (durante o dopo). In entrambi: target congelato con v = 0, STOP al satellite, heartbeat spenti, processo vivo fino a Ctrl-C (o `task.fault_linger_s` / `--fault-linger-s`).
- CLOSE_FOLLOW, SQUEEZE, RELEASE, FAULT_GRASP: stub dello stadio 3. Sono nello stesso enum, con le transizioni dichiarate in `transitionAllowed()`, ma mai raggiunti.

Sequenza:
1. **INIT**: preflight di real_replay (robot Idle, masse, FK, gravità, `start_pose_check`) e check dei limiti di giunto lungo la traiettoria con il goal (JointPathSimulator). Lo stesso check dà anche Λ_min/Λ_max al progetto dello smorzamento.
2. **WAIT_CMD**: hold della posa di attivazione. L'orientazione viene letta **una volta**, al primo ciclo.
3. **VALIDATE**: i controlli di `validateCommand()` (formato, status, frame, campi finiti, stamp, sha256, τ, start_pose vs TCP misurato ≤ 5 mm / 0.02 rad, ω vs config, `stop_at_fraction` = 1, limiti di giunto) più la fattibilità A–E con il θ0 misurato. Poi la conferma dell'operatore (o `--yes`). Se qualcosa fallisce: FAULT_NOMOTION **prima** di START. START viene inviato solo se il satellite è in WAIT_START, una volta sola.
4. **WAIT_TRIGGER**: heartbeat a `task.hb_send_hz`. Il trigger si arma solo con `omega_valid`, età ≤ `trigger_max_age_s` e |ω_meas − ω_cmd| ≤ `omega_meas_tol_deg_s`.
   - Scatto: `d_k = wrapPi(θ_k − θ_trig)`, con `sgn(ω)·d_{k−1} < 0`, `sgn(ω)·d_k ≥ 0` e `|d_k| < trigger_max_d_deg`.
   - Allo scatto il TCP misurato deve essere ancora entro 5 mm / 0.02 rad da `start_pose`.
5. **APPROACH**: `ProDmpSource` in modalità goal (sotto), avviata dalla posa di attivazione al primo ciclo a 1 kHz che vede il comando. Finisce a t = τ esattamente sul goal.
6. **HOLD_SAFE**: target congelato sul goal; dopo `task.T_hold_stage2_s`, STOP al satellite, stop controllato del loop, uscita 0.

Guasti gestiti (ognuno porta a uno stato di guasto con STOP, mai un'eccezione che chiude il processo):
- nessun datagramma valido per più di `sat_timeout_s`;
- età del campione oltre T_age in APPROACH, oltre `trigger_max_age_s` allo scatto;
- NaN, seq non crescente, `t_mono` che torna indietro;
- satellite con bit di guasto (es. heartbeat perso), oppure uscito da RUNNING, oppure non in RUNNING entro `sat_start_timeout_s`;
- ω_meas fuori banda, `omega_valid` perso dopo l'armamento;
- jitter del ciclo (`jitter_consecutive` periodi oltre `jitter_period_s`);
- loop a 1 kHz terminato (tracking guard / SafetyGuard);
- timeout del trigger;
- Ctrl-C in qualunque stato (stop controllato del loop);
- comando o START duplicati: ignorati e registrati negli eventi.

Exit code: 0 HOLD_SAFE completato; 5 FAULT_NOMOTION; 6 FAULT_MOTION; **8 FAULT_GRASP** (stadio 3); 1 INIT fallito (nessun comando al satellite); 2 annullato prima del controllo; 3 eccezione libfranka. (7 è di hw.py: `real_replay` ok ma `satellite_node` no.)

Log in `paths.log_dir` della config real_replay:
- `<ts>_task_<kind>.csv`: il log del loop esistente;
- `<ts>_task_<kind>_task.csv`: per ciclo, con stato, θ, età del campione, |e_pos|, |e_rot|;
- `<ts>_task_<kind>_events.csv`: eventi con timestamp (START, omega_valid, TRIGGER con θ e d_k, inizio/fine APPROACH, guasti, STOP).

A fine run vengono stampati i numeri del risultato:
- errore di scatto `θ_trig − θ` misurata allo scatto;
- durata trigger → fine approach;
- **errore di fase al contatto** `θ(fine approach) − θ*`, grezzo ed estrapolato con l'età del campione;
- `contact_delay_s` effettivo implicito, `−(θ_end − θ*)/ω`.

### Goal da GraspCommand e ancoraggio rigido

`ProDmpSource::setGoal(p, q)`, chiamato prima di `start()`, attiva la **modalità goal**. È la stessa catena dell'executor del simulatore in `grasp_command` (`startContinuousRollout`, con `grasp_orientation_mode = planner_goal`):
- `setRelativeGoal(true)`, poi `setInitialConditions(0, p_att, 0)`, poi `setGoal(p_goal)`;
- orientazione = SLERP da `q_att` ad `alignShortestPath(q_att, q_goal)`, con s = min(1, t/τ);
- a t = τ: `p_goal` e `q_goal` esatti, twist nullo.

Le due modalità non si sommano:
- nella modalità goal il ProDMP è già ancorato dalla sua condizione iniziale alla posa di attivazione, quindi gli offset rigidi sono l'identità (`p_off = 0`, `q_off = I`, verificato in `start()`);
- l'ancoraggio rigido (`p_off = p_att − p_raw0`, `q_off = q_att·q_raw0⁻¹`) resta, invariato, solo per `--mode replay` (nessun `setGoal`).

Nel task layer la posa di attivazione è la posa TCP **misurata** al primo ciclo, e VALIDATE (e di nuovo lo scatto) richiede che disti ≤ 5 mm / 0.02 rad dalla `start_pose` del comando.

Equivalenza (`test_prodmp_goal`, pesi `runs/20261014T150851_prodmp`, 3 goal): rollout a 1 kHz contro la catena dell'executor a 1 ms / 5 ms / 0.1 ms. Lo scarto massimo è 1.7e-4 mm e 1.1e-11°; posa finale bit per bit uguale al goal.

### hw.py run (sequenza completa)

1. `plan` con la partenza configurata: se fallisce non avvia nulla (exit 1).
2. Solo in robot mode: robot 1 e robot 2 Idle, in posa e fermi (stesso criterio di `reset`), con `echo_robot_state`.
3. Conferma dell'operatore e conferma digitata del carico (solo robot mode).
4. Avvio di `satellite_node` e handshake: attende lo stato WAIT_START sulla porta monitor.
5. Avvio di `real_replay --mode task`. È lui a inviare START, heartbeat e STOP; hw.py non invia comandi al satellite.
6. Ctrl-C manda SIGINT a entrambi e attende le rampe (SIGKILL solo dopo 15 s). Se real_replay è uscito e il satellite non lo è ancora dopo 20 s, al satellite arriva SIGINT.
7. Interpretazione degli exit code di entrambi.

Con `--dry-run` entrambi i processi girano in dry-run (config `robot1.real_replay_config_dry`). Pesi: `--weights`, default `planner.weights_yaml`, che oggi è il file **sintetico**: per un run vero va puntato al `weights.yaml` reale.

## Protocollo UDP (versione 3, little-endian, 127.0.0.1)

La versione 2 ha aggiunto il campo `flags` all'offset 116 (prima `reserved`). La **versione 3** (stadio 3) lascia i layout invariati e aggiunge il comando `STOP_RAMP` (type 4): `STOP` (type 3) diventa lo **stop duro** (rampa `robot2.control.stop_hard_ramp_s`, 0.5 s, usato per i guasti), `STOP_RAMP` è lo stop normale (rampa `stop_ramp_s`, 2 s, usato da RELEASE e a fine corsa). Un pacchetto con versione diversa viene scartato da entrambe le parti: un nodo v2 non può ricevere uno STOP_RAMP che ignorerebbe. Uno STOP duro che arriva durante una rampa normale la accorcia, ripartendo dalla fase e dalla velocità correnti (posizione e velocità continue).

| direzione | porta | contenuto |
|---|---|---|
| satellite_node → task layer | `udp.publish_port` (5601) | stato a `udp.rate_hz` (100 Hz) |
| satellite_node → hw.py | `udp.monitor_port` (5603, 0 = off) | copia identica dello stato |
| task / hw.py → satellite_node | `udp.command_port` (5602) | comandi |

**Stato**, 120 byte, `struct '<IHHIIQ5d3d3dII'`. Angoli in rad, velocità in rad/s, lunghezze in m:

| offset | tipo | campo |
|---|---|---|
| 0 | u32 | magic = `'SATS'` (0x53544153) |
| 4 | u16 | version = 1 |
| 6 | u16 | size = 120 |
| 8 | u32 | seq (+1 per datagramma) |
| 12 | u32 | state: 0 INIT, 1 WAIT_START, 2 RUNNING, 3 STOPPING, 4 HOLD, 5 DONE |
| 16 | u64 | t_mono_ns: CLOCK_MONOTONIC del ciclo di controllo da cui viene il campione |
| 24 | f64 | θ da flangia (ramo [−270°, 90°)) |
| 32 | f64 | ω_meas: pendenza ai minimi quadrati di θ su `omega_window_s` (5 s); **NaN** finché la finestra non è piena |
| 40 | f64 | θ da q7 |
| 48 | f64 | q7 |
| 56 | f64 | ω_cmd del ciclo (profilo) |
| 64 | 3×f64 | centro, frame 1 |
| 88 | 3×f64 | asse (verso l'alto), frame 1 |
| 112 | u32 | faults (bit, latched) |
| 116 | u32 | flags (bit del solo campione, non latched): bit 0 `omega_valid` |

Bit di `faults`:
- 0 window
- 1 heartbeat
- 2 tracking
- 3 safety
- 4 phase_mismatch (solo avviso)
- 5 stop_command
- 6 signal
- 7 non_finite
- 8 start_timeout
- 9 log_full

`omega_valid` è vero solo se:
- lo stato è RUNNING;
- tutta la finestra di regressione cade nella fase a velocità costante, cioè `ramp_s + omega_window_s ≤ t_run ≤` fine della fase costante;
- ω_meas non è NaN.

È falso in WAIT_START, nella rampa di partenza, nella rampa di stop nominale, in STOPPING, HOLD e DONE. **Finché `omega_valid` è falso, `omega_meas` non va usata per nessuna decisione.** hw.py la mostra come `n.v.`.

**Comando**, 24 byte, `'<IHHIIQ'`: magic `'SATC'` (0x43544153), version 3, type (1 HEARTBEAT, 2 START, 3 STOP duro, 4 STOP_RAMP), seq, riservato, t_mono_ns del mittente.

Regole:
- Prima di avviare i thread, satellite_node svuota la porta comandi: scarta i datagrammi già in coda (START/STOP vecchi) e stampa quanti ne ha scartati.
- satellite_node ruota solo dopo START (o con `--autostart`, ammesso solo in dry-run).
- START conta anche come primo heartbeat.
- Dopo il primo heartbeat, l'assenza di heartbeat per più di `T_hb_s` (0.5 s) fa partire la rampa di stop (exit 3).
- STOP (duro, 0.5 s) e STOP_RAMP (2 s) fanno partire la rampa, e l'uscita è 0 (fine ordinata richiesta). Lo stato passa da STOPPING (rampa in corso) a HOLD = **STOPPED** (a riposo); `omega_cmd` del pacchetto segue la rampa.
- Il consumatore deve scartare un pacchetto con età `now − t_mono_ns > udp.T_age_s` e deve controllare `seq`.

## satellite_node

```
satellite_node --config hw/hw_config.yaml [--dry-run] [--autostart] [--yes] [--max-travel-deg D]
               [--dry-q-deg q1,..,q7] [--pub-port P] [--monitor-port P] [--cmd-port P]
               [--log-dir DIR] [--log-tag TAG] [--countdown-s N] [--max-wait-start-s S]
```

Controllo e corsa:
- Il controllo è come `satellite_j7`: PD di giunto in coppia a 1 kHz, J1–J6 fermi su q0, J7 su un profilo smoothstep (`j7_profile.hpp`), SafetyGuard su ogni coppia inviata.
- Direzione: `q7_ref = q7_0 + sign·d`, con `sign = sgn(ω)·q7_phase.sign`.
- La corsa è limitata dalla finestra: il profilo nominale finisce `window_margin_deg` prima del bordo, e non arriva mai a 360°.
- La guardia ferma con la rampa se q7 o θ superano il **bordo**.
- Il preflight verifica che bordo + `v·T2/2` + tolleranza di tracking resti dentro i limiti di J7.

Carico:
- Il mockup è un **payload**: `setLoad(mockup.mass_kg, mockup.com_m)`.
- `m_ee` è confrontato con `robot2.m_ee_max_kg` (nessuna mano in Desk).
- `m_load` è confrontato con la massa del mockup, solo come avviso.
- Dal file `robot2.real_replay_config` (hold3_robot2_nohand.yaml) arrivano **solo** safety, collision, set_ee e le tolleranze di gravità. Il suo `robot.ip` e il suo `m_load_max_kg` (0.05) non sono usati.

Preflight geometrico (robot e dry-run):
- centro da FK vs `scene.center_m`;
- asse vs tilt;
- θ0 e q7 dentro la finestra con margine;
- θ da flangia vs θ da q7.

In robot mode il preflight confronta anche la FK DH con `O_T_EE·F_T_EE⁻¹` (≤ 1 mm, 0.5°).

Conferma del carico (robot mode):
- Prima di "Press Enter" stampa in evidenza `CARICO DICHIARATO: X kg, com ... — il mockup è montato?`.
- Senza `--yes`, l'operatore deve digitare la massa in kg. Deve coincidere con `mockup.mass_kg` entro 1 g, altrimenti exit 2 senza prendere il controllo.
- `hw.py run` passa `--yes` al nodo, quindi in robot mode chiede lui la stessa conferma prima di avviarlo.

Dry-run:
- Usa un plant sintetico, con jitter del periodo e ritardo di misura da `robot2.dry_run`, a passo di 1 kHz in tempo reale.
- Il codice di fase, ω, finestra, UDP e heartbeat è lo stesso del robot reale.

Log: `paths.log_dir/<timestamp>_satellite_node_[tag_]<dryrun|robot>.csv`, più `.meta.txt`. Colonne: θ da flangia, θ da q7, ω, stato, faults, centro.

Exit code:
- 0 = completato o STOP;
- 1 = preflight / runtime;
- 2 = argomenti / config / carico non confermato;
- 3 = fermato con rampa (Ctrl-C, heartbeat, tracking, finestra, safety, START mancato);
- 4 = rampa non completata (reflex?).


## Stadio 3: pinza, follow, rilascio, guasti

Abilitazione: `gripper.enabled: true` in `hw_config.yaml` (default **false** = comportamento dello stadio 2, APPROACH → HOLD_SAFE). Con la pinza abilitata si usa `config/task_robot1_hand_ff*.yaml` (controllore con feedforward d'attrito, vedi sotto) impostando `robot1.real_replay_config(_dry)`.

### Pinza (`gripper:`)

- `IGripper` / `GripperWorker` (thread dedicato: `grasp()` è bloccante) / `FakeGripper` (test e dry-run) / `FrankaGripper` (`franka::Gripper` su `gripper.host`, default `robot1.ip`, connessione separata dal controllo). La pinza si apre anche con il braccio in errore o in reflex: non dipende dal controllo. Non fa mai `homing`: se `max_width < 0.05 m` rifiuta.
- Apertura iniziale: `open_width_m` 0.080 m **totali** (0.040 per dito), comandata insieme a START; il trigger richiede la conferma di apertura, altrimenti FAULT_NOMOTION.
- Chiusura `grasp(grasp_width_m, speed_m_s, force_n, eps_in_m, eps_out_m)`: velocità ≤ 0.03 m/s e forza ≤ 10 N sono **rifiutate** (non troncate) in lettura della config.
- **La forza in N non è documentata da libfranka come per-dito o totale: non è assunta.** Il valore va a `grasp()` così com'è. `eps_*` sono tolleranze di `is_grasped`, non la larghezza raggiunta. `grasp_width_m` = spessore parete del collare (0.009, assunto).

### Stati

`APPROACH → (AT_END) → CLOSE_FOLLOW → SQUEEZE → HOLD → RELEASE → HOLD_SAFE → DONE`

- CLOSE_FOLLOW: parte `grasp()`, il target segue il satellite. Successo (`grasp()` vero **e** `is_grasped`) → SQUEEZE; fallimento, o nessun esito entro `T_close_max_s` (6 s) → FAULT_GRASP.
- SQUEEZE: si tiene la presa alla forza di config per `squeeze_s`. Non esiste il setpoint −0.05 m del sim (in Gazebo era una posizione oltre il limite fisico). `is_grasped` che cade → FAULT_GRASP.
- HOLD: follow attivo fino a `T_follow_s` dopo il successo del grasp (caso peggiore = condizione D di `plan`: `T_close_max + T_follow + T_ramp`).
- RELEASE: `STOP_RAMP` al satellite, il robot 1 continua a seguire la fase **misurata** (`omega_cmd` del satellite scende con la rampa); il target si congela quando il satellite segnala **STOPPED** (stato HOLD/DONE), non con `|ω_meas|` (finestra di 5 s: ritardo inutile per riconoscere l'arresto). Solo dopo STOPPED la pinza si apre. STOPPED non raggiunto entro `stop_ramp_s + 3 s` → FAULT_GRASP.

### Follow

`dθ = wrapPi(θ_sat − θ_end)`, con θ_sat = `θ_campione + ω_cmd·età` (campione fresco, età ≤ 0.5 s); con il campione più vecchio il θ continua a ω nominale dall'ultima stima fresca. `p = c + R(asse, dθ)(p_goal − c)`, `q = R(asse, dθ)·q_goal` (stessa convenzione di `reach_check.py`); c e asse dal datagramma del satellite. `twist()` = (ω × r, ω) alimenta il feedforward d'attrito e va a zero con la rampa.

### Guasti

Qualunque FAULT: (1) STOP **duro** al satellite per primo, (2) target congelato (posa di comando corrente, v = 0), (3) log in `*_events.csv`. FAULT_NOMOTION / FAULT_MOTION: la pinza resta com'è (aperta). Da CLOSE_FOLLOW in poi → FAULT_GRASP: STOP duro, poi apertura **subito dopo**, nello stesso aggiornamento, senza attendere l'arresto (ritenta fino a 3 volte). Un guasto dopo uno STOP_RAMP già inviato aggiunge lo STOP duro. All'uscita del processo (loop di controllo terminato: reflex, eccezione) la macchina vede `loop_finished`, manda STOP e apre la pinza prima di chiudere.

**Heartbeat**: non è un timer indipendente. Parte solo se la macchina lo abilita **e** il ciclo a 1 kHz ha girato negli ultimi `task.hb_stall_s` (0.1 s). Se il loop si ferma o va in eccezione, il heartbeat smette e `satellite_node` si ferma da solo dopo `T_hb_s` (0.5 s). Provato in dry-run con `real_replay --mode task --dry-run --hang-loop-after-s S` (SOLO test: il loop smette di ciclare senza eccezione).

**Guard dopo la presa** (`task.guard`, **proposte non validate**, solo robot): `K·e_pos` (K 2000 N/m: 1 mm = 2 N; warn 8 N, fault 16 N) e `|O_F_ext_hat_K|` (warn 10 N, fault 20 N), fault se superata per `debounce_s` 50 ms.

### Carico (mockup 0.4 kg)

Il carico è impostato **prima** del run (`setLoad` non è possibile durante il controllo): `end_effector.set_load/load_mass` del config robot (oggi 0.0). Il preflight accetta `m_load ≤ preflight.m_load_max_kg` (0.05 kg nel config task) e `m_ee ≤ 0.80 kg` (mano 0.73). Senza `setLoad` il peso del mockup (3.9 N) non è compensato: ≈ 2 mm statici (K = 2000 N/m), coperti dal guard. Per compensarlo servirebbero `load_mass` ≈ 0.4 e `m_load_max_kg` ≥ 0.4 **prima** della presa, con +2 mm di spinta verso l'alto prima di afferrare: **non modificato**, decisione dell'operatore.

### Feedforward d'attrito

`config/task_robot1_hand_ff.yaml` / `_ff_dry.yaml` = `task_robot1_hand*.yaml` + `friction_ff` del controllore validato il 17/11 (`*_ffa100`: α 1.0, eps 0.005 rad/s, DLS 0.05, τc [0.615 0.361 0.361 0.562 0 0 0.154], b [2.83 1.89 2.71 1.6 0 0 0.18], rampa gain 0.1→1 in 3 s). `task_robot1_hand.yaml` non lo abilita (invariato). La velocità di riferimento viene da `twist()` della sorgente: ProDmpSource in APPROACH, rotazione rigida in FOLLOW (si annulla con la rampa e dopo il congelamento).

### Altri

- `start_pose.quaternion_wxyz` è opzionale: senza, l'orientazione di partenza non è controllata (WARN in VALIDATE; `hw.py plan` lo segnala).
- `hw.py plan` esegue `reach_check.py` come check bloccante "verifica cinematica (senza carico/collisioni)": IK del goal e del follow, margine ≥ `plan.reach_min_margin_deg`, nessun salto di ramo. **Non** è una portata verificata: carico, collisioni e dinamica non sono modellati.
- `real_replay --mode task --preflight-only`: solo lettura (nessun `control()`, nessun `setEE/setLoad`, nessun comando al satellite); controlla preflight, pinza raggiungibile e omologata, e le verifiche statiche del comando.
- Target CMake: `gripper_test`, `gripper_grasp_test`, `friction_id`.
