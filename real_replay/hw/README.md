# hw/ — set-up a due Franka, stadio 1 (robot 2 "satellite")

Contenuto:

| file | ruolo |
|---|---|
| `hw_config.yaml` | unica configurazione (scena, mockup, robot 1, robot 2, UDP, timing, percorsi, planner) |
| `hw.py` | strumenti utente: `status`, `ik`, `reset`, `plan`, `run` (`python3 hw/hw.py --help`) |
| `../tools/satellite_node.cpp` | processo del robot 2: ruota J7, pubblica lo stato via UDP |
| `examples/grasp_command_nominale.yaml` | GraspCommand di esempio |
| `examples/weights_sintetico_test.yaml` | pesi **sintetici, solo per i test** (contiene solo `tau`; non è un file di pesi reale) |
| `test/` | test offline (`ctest`: `hw_plan`, `satellite_dryrun_e2e`) |

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
| `run --only-satellite [--dry-run] [--max-travel-deg D]` | Avvia **solo** `satellite_node`, poi START, heartbeat a `udp.hb_send_hz` e stato live dalla porta monitor. Senza `--only-satellite` stampa "disponibile dallo stadio 2". |

Ctrl-C durante `run` o `reset`: hw.py manda **un** SIGINT al figlio e attende la sua rampa di stop. SIGKILL solo dopo 15 s.

`--yes` arriva a `satellite_node` solo se stdin è un terminale e solo dopo la conferma di hw.py.

Exit code di hw.py:
- 0 = ok;
- 1 = controllo fallito / rifiutato / annullato;
- 2 = errore di config o d'ingresso;
- `run` = exit code di `satellite_node`.

### Controlli di `plan`

Notazione:
- `s = sgn(ω)`
- `T_c = τ + timing.contact_delay_s`
- `θ_trig = wrapPi(θ* − ω·T_c)`, riportato sul ramo [−270, 90) per i confronti

Controlli bloccanti:
1. `status == SELECTED`.
2. `sha256(weights) == weights_sha256`.
3. `|contact_time_s − τ| ≤ plan.tau_tol_s` (1e-6). τ è la chiave `tau` dei pesi, la stessa di `prodmp_io::loadProDmpFromYaml`.
4. `|ω_comando − scene.omega_deg_s| ≤ plan.omega_tol_deg_s` (0.1°/s).
5. ω ≠ 0. Solo avviso se |ω| non è in `plan.omega_domain_deg_s` = {2, 3}°/s.
6. θ0 dentro la finestra con margine. θ0 = θ dalla FK della posa di partenza del robot 2 (`robot2.start`: q fisso o IK).
7. Armamento: `s·(θ_trig − θ0) ≥ |ω|·T_arm_min_s`.
8. Condizione A: `s·(θ_trig − θ0) ≥ |ω|·(ramp_s/2 + omega_window_s) + |ω|·timing.margin_arm_s`.
   - `|ω|·(ramp_s/2 + omega_window_s)` è l'angolo percorso prima che `omega_valid` diventi vero.
   - `ramp_s` e `omega_window_s` vengono da `robot2.control`.
9. `θ_end = θ* + ω·(T_close_max + T_follow + T_ramp)` dentro la finestra con margine.

Stampa anche:
- l'intervallo ammesso di θ0 (finestra con margine ∩ armamento);
- il timeout del trigger, `(θ_trig − θ0)/ω + trigger_timeout_margin_s`;
- "portata robot 1: NON verificata". Il punto di estensione è `check_robot1_reach()`.

Con la config e il comando di esempio:
- θ_trig = −30.168°;
- condizione A = 2·(1 + 5 + 2) = 16°;
- θ0 ammesso ∈ [−14.168, −5.000]°;
- θ0 = −10° → FATTIBILE;
- θ_end = −83.9 − 2·(6 + 5 + 2) = −109.9°.

## Formato YAML d'ingresso

**GraspCommand** (`format: grasp_command_v1`). Rispecchia `satellite_grasp_msgs/GraspCommand.msg`, ma usa i gradi:

```yaml
format: grasp_command_v1
status: SELECTED              # SELECTED | NO_FEASIBLE_CANDIDATE | POSTHOC_FAILED | UNSET
theta_star_deg: -83.9         # msg.theta_star_rad in gradi, ramo [-270, 90)
contact_time_s: 21.466        # msg.contact_time_s (= tau dei pesi)
omega_deg_s: -2.0             # msg.omega_rad_s in gradi
weights_sha256: "<hex>"       # msg.weights_sha256
goal_pose: null               # stadio 2: {position_m: [x,y,z], quaternion_wxyz: [w,x,y,z]}, frame 1
k: null                       # stadio 2: GraspPointId 0..3
wrist_flip: null              # stadio 2
```

**Weights**: il file ProDMP reale (`weights.yaml` di `runs/...`). `plan` legge solo `tau` e lo sha256 del file intero. Il file `examples/weights_sintetico_test.yaml` è un sostituto sintetico per i test (`synthetic_test_only: true`).

## Protocollo UDP (versione 2, little-endian, 127.0.0.1)

La versione 2 aggiunge il campo `flags` all'offset 116 (prima `reserved`). Un pacchetto con versione diversa viene scartato.

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

**Comando**, 24 byte, `'<IHHIIQ'`: magic `'SATC'` (0x43544153), version 2, type (1 HEARTBEAT, 2 START, 3 STOP), seq, riservato, t_mono_ns del mittente.

Regole:
- Prima di avviare i thread, satellite_node svuota la porta comandi: scarta i datagrammi già in coda (START/STOP vecchi) e stampa quanti ne ha scartati.
- satellite_node ruota solo dopo START (o con `--autostart`, ammesso solo in dry-run).
- START conta anche come primo heartbeat.
- Dopo il primo heartbeat, l'assenza di heartbeat per più di `T_hb_s` (0.5 s) fa partire la rampa di stop (exit 3).
- STOP fa partire la rampa, e l'uscita è 0 (fine ordinata richiesta).
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
