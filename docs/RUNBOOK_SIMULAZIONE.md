# RUNBOOK operativo della simulazione (F0–F6)

Procedura per portare la simulazione dal container pulito al report di un run S1. Ricavata dal codice del repository e verificata con un dry run del 23–24/10/2026 (F0, F1, F3, F4 e F5 eseguite; **F2 non eseguita**, richiede lo stilo Geomagic). I punti non verificati sono nell'Appendice A. Riferimenti `file:riga` relativi alla radice del repo, riferiti a HEAD 779b746 (poi `tools/s1_campaign.py` esteso in 0ae4814): le righe possono essersi spostate.

## Convenzioni

- Host: `/home/lorenzo/thesis_ws` ≡ container `franka`: `/root/thesis_ws` (mount in `scripts/run_with_thesis_mount.sh`, riga `--volume "$THESIS_WS:/root/thesis_ws"`).
- Container `franka`: tutto ROS/Gazebo/colcon/pytest (`tests/gate.sh` usa `docker exec franka bash -c "source /root/env_thesis.sh && cd /root/thesis_ws && ..."`).
- Container `geomagic_touch` (immagine `haptic_device_geomagic_touch:latest`, rete/PID/IPC `host`, privilegiato): citato in `src/haptic_dmp_learning/DESIGN_NOTES.md:116`. Nel dry run era fermo (Exited 137); per F2 va avviato (`docker start geomagic_touch`, non nel repo). **Nessun comando/pacchetto del driver è nel repo** (Appendice A).
- Nel dry run `franka` era in esecuzione. Entrambi i container condividono il mount `/home/lorenzo/thesis_ws → /root/thesis_ws`; nessuno dei due imposta `ROS_DOMAIN_ID`.
- **Preambolo P** (da ripetere in ogni shell nuova del container; `/root/env_thesis.sh` è nel container, non nel repo; contenuto verificato: `source /opt/ros/humble/setup.bash; source /root/thesis_ws/install/setup.bash`, quindi `AMENT_PREFIX_PATH` ha già i pacchetti di thesis_ws davanti a quelli di franka_ws; l'`eval` di `s1_env.py` è ridondante — l'output con e senza è identico — ma innocuo):

```bash
source /root/env_thesis.sh
eval "$(python3 /root/thesis_ws/tools/s1_env.py --root /root/thesis_ws/install)"   # AMENT/LD_LIBRARY_PATH: thesis_ws/install davanti
cd /root/thesis_ws
```

  `s1_run.py` applica da sé questo riordino (`ros_cmd`, tools/s1_run.py:474-479); le shell manuali (F1, F2) no, ma con l'`env_thesis.sh` attuale non serve (Appendice A, punto 3).
- Shell del container: `docker exec -it franka bash` (da host). `-it` serve a F2 (tasto SPACE: `isatty`, demo_replay_sync_orchestrator_node.cpp:227).

## Percorso rapido (demo -> cubo -> replay)

Sintesi del flusso completo; i comandi lunghi sono nelle sezioni indicate. Tutto nel container `franka` con il preambolo P, salvo dove scritto.

**(a) Registrare la demo (F2, richiede lo stilo).** Con Gazebo e controller attivi (F1, shell A), in una shell TTY lanciare l'orchestratore in modalità demo (comando completo in F2): 1ª pressione = spawn del cubo, 2ª = trigger e chiusura gripper, 3ª = stop. Escono `/root/thesis_ws/demo_raw_<run_id>.csv` e i file collegati.

**(b) Fit (F3).** Troncare la demo al trigger e farne il fit:

```bash
python3 tools/trim_demo_at_trigger.py /root/thesis_ws/demo_raw_20261014T150851.csv    # esempio: sostituire con il CSV della demo nuova
```

poi il comando `mp_tool.py fit` di F3 sul CSV `_trim` (cambiando `--label`). Annotare la cartella `runs/<data>_fit_<label>/` stampata (`Run directory: ...`): è il `--weights-dir` del passo (c).

**(c) Replay con cubo in nuove condizioni (F4).**
1. Scegliere la cella: ω [°/s], centro, α (`--axis-tilt-deg`), φ (`--axis-lean-deg`). Esempio (è la C12 della campagna): ω = −2, centro 0.75,0.05,0.72, α = 5, φ = 225.
2. θ\* col driver offline (senza Gazebo): comando di F4 «Procedura in due passi» (a), dopo aver aggiunto la riga della cella al CSV («Aggiungere una cella»). Per C12 dà k0 A, θ\* = −104.0.
3. Yaw dalla formula `yaw0 = wrap(θ* − ω·45.2)`: −104.0 + 90.4 = **−13.6°**.
4. Lanciare `s1_run.py` con il comando completo di F4, cambiando solo `--weights-dir`, `--tag` e i parametri della cella, e aggiungendo `--gui` per vedere il replay in Gazebo. Per l'esempio: `--spawn-yaw-deg -13.6 --omega-deg-s -2.0 --center-xyz 0.75,0.05,0.72 --axis-tilt-deg 5.0 --axis-lean-deg 225.0 --gui`. Per la GUI serve un display X raggiungibile dal container: aprire la shell con `docker exec -it -e DISPLAY=:0 franka bash` (il runner lancia poi `DISPLAY=:0 ign gazebo -g`, s1_run.py:907-908).
5. Esito: `s1_runs/<tag>/report.txt` e `report.json` (F5).

**Come spostare o ruotare il cubo** (valori a esempio, vincoli in F4 «Regole di coerenza»): `--center-xyz X,Y,Z` posizione del centro di rotazione [m] (nominale 0.70,0.0,0.77); `--omega-deg-s` velocità angolare con segno (−2.0); `--axis-tilt-deg` inclinazione dell'asse da z, 0–20°; `--axis-lean-deg` azimut dell'inclinazione, 0–360°. Ogni cambio richiede di ricalcolare θ\* e lo yaw (passi 2–3).

---

## F0 — Ambiente

**Scopo**: container pronti, pacchetti del repo compilati in `/root/thesis_ws/install`, nessun residuo di run precedenti.

**Precondizioni**: container `franka` già creato (lo crea `scripts/run_with_thesis_mount.sh`, che richiede il nome dell'immagine e `docker_run.cfg`, entrambi fuori dal repo: Appendice A). Eventuale provisioning una tantum del bringup vendored: `bash /root/thesis_ws/scripts/full_reset_franka_container.sh` (copia gli override in `franka_ws/.../franka_gazebo_bringup/{config,launch}`, symlink, build con `--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` in `franka_ws`: scripts/full_reset_franka_container.sh:104-107) e, per il gripper prismatico, `bash /root/thesis_ws/scripts/setup_franka_description_overrides.sh`.

**Build (container `franka`, shell con P)** — i 6 pacchetti come in `tests/gate.sh:79,89`:

```bash
colcon build --packages-select franka_cartesian_control free_target_object grasp_monitoring haptic_dmp_learning satellite_grasp_msgs satellite_grasp_planner
```

- Il comando è identico a quello di `tests/gate.sh` (passo a, righe 79 e 89). Nota: `franka_cartesian_control`, `haptic_dmp_learning`, `satellite_grasp_planner` forzano Release se il tipo non è impostato (`src/*/CMakeLists.txt:4-6`); gli altri tre pacchetti non lo fanno (non C++ pesante).
- Clean build: `tests/gate.sh --clean` usa `--cmake-clean-first --cmake-force-configure --event-handlers console_cohesion+` (gate.sh:81-83).
- Variabile: `THESIS_WS` (default `/root/thesis_ws` nello script g25; `s1_run.py:42` la legge dall'ambiente, default = radice del repo).

**Controlli**

```bash
# (a) i 4 pacchetti usati da s1_run devono risolversi SOTTO /root/thesis_ws/install (preflight di s1_run.py:488-516)
for p in haptic_dmp_learning satellite_grasp_planner free_target_object grasp_monitoring; do ros2 pkg prefix $p; done
# (b) residui di run precedenti (stessi pattern di LEFTOVER_PATTERNS, s1_run.py:94-100)
ps -eo pid,args | grep -E "ign gazebo|gz sim|ros2 bag record|parameter_bridge|grasp_planner_node|prodmp_gazebo_executor_node|dmp_gazebo_executor_node|controller_manager|robot_state_publisher|free_target_object|ros2 launch" | grep -vE "grep|bash -c"      # atteso: nessuna riga. NON usare pgrep -af / pkill -f: combaciano con la propria shell bash -c (falsi positivi, verificato nel dry run)
# (c) Kt lasciato a 2000 da un run ucciso con SIGKILL (il finally non gira)
grep -n "translational_stiffness" /root/ros_workspaces/ros2/franka_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml

```

- **`thesis_ws/install` è richiesto, non da evitare**: `s1_run.py:48` (`INSTALL_DIR = WS_DIR / "install"`) e `:501-502` (RunError se un pacchetto non è sotto `THESIS_WS/install`), `s1_env.py:2-6`, `gate.sh:79`. Verificato nel dry run: `AMENT_PREFIX_PATH` ha i 6 pacchetti di `/root/thesis_ws/install` prima di quelli di `franka_ws/install`, che contiene copie OBSOLETE (13–15 ott) di `haptic_dmp_learning`, `grasp_monitoring`, `free_target_object` (oscurate dall'ordine corretto; `franka_cartesian_control` e `satellite_grasp_*` non ci sono). Il rischio è l'ordine sbagliato (preambolo P), non l'esistenza di `thesis_ws/install` (con `COLCON_IGNORE`).
- Residui (b): con `--pid=host` (run_with_thesis_mount.sh) `pgrep -f` nel container vede anche processi dell'host; `s1_run.ensure_clean` esclude solo i propri antenati e uccide con SIGKILL (s1_run.py:419-434) (Appendice A, punto 12).

**Riavvio pulito (derivato da s1_run.py:884-887; non c'è uno script dedicato)**

```bash
ros2 daemon stop
ps -eo pid,args | grep -E "ign gazebo|gz sim|ros2 bag record|parameter_bridge|grasp_planner_node|prodmp_gazebo_executor_node|dmp_gazebo_executor_node|controller_manager|robot_state_publisher|free_target_object|ros2 launch" | grep -vE "grep|bash -c" | awk '{print $1}' | xargs -r kill -9   # uccide per PID (pkill -f uccide la propria shell; SIGINT al launch lascia vivo "ign gazebo -s -r empty.sdf")
sleep 3; ps -eo pid,args | grep -E "ign gazebo|gz sim|ros2 bag record|parameter_bridge|grasp_planner_node|prodmp_gazebo_executor_node|dmp_gazebo_executor_node|controller_manager|robot_state_publisher|free_target_object|ros2 launch" | grep -vE "grep|bash -c"   # ricontrollo: nessuna riga
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

**Successo**: (a) quattro prefissi sotto `/root/thesis_ws/install`; (b) nessuna riga; (c) `translational_stiffness: 200.0` (valore a riposo, `KT_DEFAULT`, s1_run.py:59; copia sorgente: src/franka_gazebo_overrides/franka_gazebo_controllers.yaml:121).
**Tempo**: build non misurato nel repo (`tests/gate.sh` stampa le durate per passo).
**Errori noti**: `preflight: pacchetto X non trovato` / `prefisso ... non sotto .../install` (s1_run.py:499,502); `executor senza follow limitato: binario vecchio` (:515, binario senza stringa "then frozen" → rebuild vero, non basta `--symlink-install`); `processi residui prima del run` (codice 3).

---

## F1 — Gazebo con il robot, `ready_q6pi`, `prepos_capture`

**Scopo**: simulazione attiva con controller a impedenza, gripper, `/clock` su ROS, robot nella posa `ready_q6pi`; opzionale parcheggio a posa di cattura (READY).

**Precondizioni**: F0 completata; gripper prismatico: override applicato (`setup_franka_description_overrides.sh`); i launch usati sono le COPIE in `franka_ws/.../franka_gazebo_bringup/launch/` (i file in `src/franka_gazebo_overrides/` non sono letti in place, vedi `full_reset` step 1/6; verificato nel dry run: le copie nel bringup sono identiche (`diff` vuoto) agli override e il file in `install/` è un symlink verso di esse, quindi cambiare il YAML non richiede rebuild).

**Shell A (container franka, preambolo P)** — bringup identico a quello di `s1_run` (s1_run.py:823-824, 869-870):

```bash
FRANKA_START_POSE=ready_q6pi FRANKA_FINGERS=prismatic ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control_headless.launch.py load_gripper:=true
```

- Cosa parte (gazebo_cartesian_impedance_control.launch.py): `gz_sim` con `-s -r empty.sdf` (solo server, **nessuna finestra**; riga 129), `clock_bridge` `/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock` (149-158), `gripper_position_bridge` (165-174), `robot_state_publisher`, spawn `-topic /robot_description`, spawner di `joint_state_broadcaster` poi `cartesian_impedance_controller` (timeout 60 s), `joint_state_publisher` a 30 Hz.
- Argomenti launch: `arm_id` (fer), `load_gripper` (false), `franka_hand` (franka_hand), `headless` (solo nel launch non-headless; false → RViz).
- `FRANKA_START_POSE` ∈ {ready, ready_q6pi, capture}, default `ready`: letto da xacro al momento di generare l'URDF (src/franka_description_overrides/franka_arm.ros2_control.xacro:48-55). `ready_q6pi = [0, -π/4, 0, -3π/4, 0, π, π/4]` (giunto 6 = π). `FRANKA_FINGERS` ∈ {fixed (default), fixed_open, prismatic} (franka_hand.xacro:119-124).
- Vedere Gazebo: il launch non apre la GUI. Altra shell: `DISPLAY=:0 ign gazebo -g` (come `--gui`, s1_run.py:907-908). Con il launch non-headless si ottiene solo RViz.

**Successo**: in shell A la stringa REALE (verificata nel dry run del 23/10, bringup.log) è `[spawner_cartesian_impedance_controller]: Configured and activated cartesian_impedance_controller`, preceduta da `[controller_manager]: Loading controller 'cartesian_impedance_controller'` e `[cartesian_impedance_controller]: CartesianImpedanceController parameters loaded:` (con l'elenco dei guadagni). La stringa `Successfully loaded controller …` NON viene mai stampata. Controllo effettivo come in s1_run.py:899-902:

```bash
ros2 control list_controllers          # cartesian_impedance_controller ... active
ros2 topic info /clock --verbose       # Publisher count >= 1
```

**Tempo**: `s1_run` attende il controller fino a 70 s (`CONTROLLER_WAIT_S`, s1_run.py:77) + 5 s di assestamento (:909). Valore tipico misurato: non nel repo.
**Errori noti**: `/clock` muto → bringup di velocità (senza clock_bridge) o bridge non partito (docs/COMMAND_REFERENCE.md §7); controller non attivo entro 70 s → «controller non attivo entro 70 s» (:905); gripper fermo → mancano `load_gripper:=true` o l'override.

**`prepos_capture` (opzionale; posa di cattura + segnale READY)** — NOTA: lo script NON è in `tools/prepos_capture.py` ma in **`tools/legacy/prepos_capture.py`** (unico file con quel nome nel repo).

```bash
# Shell B (container franka, P); Gazebo già attivo
python3 tools/legacy/prepos_capture.py        # tutti i default
```

- Opzioni (tools/legacy/prepos_capture.py:293-303): `--anchor-pos X Y Z` (0.589 0.344 0.35), `--anchor-quat X Y Z W` (0.6903455270798549 0.15304591873303097 0.15304591873303086 0.6903455270798549), `--duration` (20.0 s rampa min-jerk), `--target-topic` (/target_pose), `--joy-topic` (/touch0/buttons_synced), `--world-frame` (world), `--ee-frame` (fer_hand_tcp), `--controller-name` (cartesian_impedance_controller), `--max-error` (0.100 m), `--rate` (200 Hz).
- Cosa fa: attende controller attivo, esegue `ros2 param set /cartesian_impedance_controller skip_initial_alignment true` (riga 158-160), legge TCP da TF, rampa fino all'ancora e poi tiene la posa.
- **Segnale READY**: stampa `READY: tenuta alla posa di cattura, ora si può cliccare` (riga 219) a fine rampa (~20 s sim dopo l'avvio). Poi il primo fronte sul pulsante 0 di `/touch0/buttons_synced` lo fa terminare con codice 0 (stampa `PREPOS CAPTURE COMPLETE`).
- Errori: fronte di start durante la rampa → `start edge during ramp: arm not at anchor, discard session`, uscita 2 (riga 112); errore di inseguimento > `--max-error` → `ABORT: Tracking error ...`, uscita 1.
- L'ancora di default coincide con quella di docs/COMMAND_REFERENCE.md §3.2 (`anchor_position_m`/`anchor_orientation_xyzw`); la posa di cattura non è collegata a `FRANKA_START_POSE=capture` nel codice (è un'altra posa; non verificato se vanno usate insieme (Appendice A)).

---

## F2 — Registrazione della demo col Geomagic (pipeline a pulsante singolo, 3 pressioni)

**Stato: NON eseguita nel dry run** (richiede lo stilo fisico): i passi sono ricavati dal codice; i punti incerti sono segnati «non verificato».

**Scopo**: produrre CSV della demo, joint_states e odometria del cubo, mentre il movimento dello stilo si vede in Gazebo e un cubo in moto viene spawnato.

**Nota sullo schema**: nel codice lo schema di default è **`single_button`**: UN pulsante premuto 3 volte = (1) start, (2) trigger + chiusura gripper, (3) stop; il tasto SPACE del terminale dell'orchestratore conta come una pressione dello stesso pulsante (src/haptic_dmp_learning/config/params.yaml:54-59; button_sequencer.hpp:11-18: `IDLE → STARTING → RECORDING → TRIGGERED → STOPPED`; debounce 0.30 s). Lo schema `legacy` è: pulsante 0 start, pulsante 1 stop, SPACE trigger. Non esiste uno schema a «tre pulsanti» fisici distinti. Il `live_demo_recorder_node` in più imposta `gripper_trigger` se `buttons[2] != 0` (live_demo_recorder_node.cpp:366-368).

**Precondizioni**: F1 attiva (Shell A, `/clock` presente); driver Geomagic attivo nel container `geomagic_touch`; per il robot che segue lo stilo con ancora: controller con `skip_initial_alignment=true` (lo imposta `prepos_capture`; altrimenti a mano, Appendice A punto 7).

**Driver (container `geomagic_touch`)**: pacchetto, nodo, comando e topic nativi **non sono nel repo** → non verificato. Dal codice: la pipeline si aspetta `sensor_msgs/Joy` su `/touch0/buttons` e le pose su `/master_pose_raw` (`geometry_msgs/PoseStamped`, frame `omni_base`) — `haptic_dmp_wrapper_node` legge `/touch0/pose`; per il recorder serve un remap `-r <topic_nativo>:=/master_pose_raw` sul driver (live_demo.launch.py:14-18, src/haptic_dmp_learning/DESIGN_NOTES.md:15-19). Le correzioni di frame omni_base→fer_link0 (+90° su z) sono applicate dai nodi consumatori (frame_correction.hpp), non dal driver.

**Shell C (container franka, P, TTY) — orchestratore in modalità demo con stilo reale e cubo in moto**:

```bash
ros2 run haptic_dmp_learning demo_replay_sync_orchestrator_node --ros-args \
  --params-file /root/thesis_ws/src/haptic_dmp_learning/config/params.yaml \
  -p mode:=demo -p use_csv_playback:=false \
  -p anchor_enabled:=true -p anchor_position_m:="[0.589, 0.344, 0.35]" \
  -p anchor_orientation_xyzw:="[0.6903455270798549, 0.15304591873303097, 0.15304591873303086, 0.6903455270798549]" \
  -p target_x:=0.70 -p target_y:=0.0 -p target_z:=0.77 -p target_wz:=-0.034906585
```

- Il comando è l'esempio di docs/COMMAND_REFERENCE.md §3.2 + `use_csv_playback:=false` + posa/omega del cubo scelti qui: i valori di `target_*` della demo vera NON sono nel repo → non verificato. `-0.034906585 rad/s` = −2 °/s (grasp_planner_example.yaml:113). Gli array vanno scritti con il punto decimale (`1.0`, non `1`).
- Cosa lancia l'orchestratore: attende `/clock` (timeout 10 s wall) → `ros2 launch haptic_dmp_learning demo_replay_sync.launch.py run_id:=<id> use_csv_playback:=... [anchor...]` (recorder con `use_sim_time`, ingresso pulsanti rimappato `/touch0/buttons → /touch0/buttons_synced`; + `csv_master_pose_player_node` solo se `use_csv_playback`) → attende la 1ª pressione → `ros2 launch free_target_object free_target_object.launch.py name x y z vx vy vz wx wy wz size_x/y/z mass with_collar collar_collision yaw` → attende la 1ª odometria (timeout 30 s) → dopo `sync_delay_sec` (5.0 s sim) emette il fronte di start su `/touch0/buttons_synced` → registrazione.
- Sequenza operatore: **1ª pressione** (stilo o SPACE): spawn cubo + finestra di sync (le pressioni durante STARTING sono ignorate); **2ª**: trigger + rampa di chiusura gripper (2.0 s, 0.06→0.0 m) e flag `gripper_trigger` nel CSV; **3ª**: stop → il recorder salva e addestra la DMP classica.
- `run_id` = `%Y%m%dT%H%M%S` generato all'avvio (o `-p run_id:=20261024T101500`, esempio).
- Parametri modificabili (orchestratore, demo_replay_sync_orchestrator_node.cpp:91-199; vedi anche l'Appendice B): `sync_delay_sec`, `clock_wait_timeout_sec`, `odom_wait_timeout_sec`, `target_*` (x y z, vx vy vz, wx wy wz, size_x/y/z 0.150, mass 4.412, yaw 0.0), `target_with_collar`, `target_collar_collision`, `button_scheme`, `button_index`, `debounce_s`, `use_csv_playback`, `demo_csv_path`, `anchor_*`.
- Variante con CSV al posto dello stilo (stand-in): `use_csv_playback:=true` (default); il player (`csv_master_pose_player_node.demo_csv_path`, src/haptic_dmp_learning/config/params.yaml:13) genera start/stop su buttons[0]/[1] → per l'orchestratore serve `-p button_scheme:=legacy` (nota in src/haptic_dmp_learning/config/params.yaml:57).
- **Wrapper** (`haptic_dmp_wrapper_node`): percorso alternativo/storico, solo `ros2 run` nel container `geomagic_touch` con `--params-file` obbligatorio (n_basis, alpha_x, alpha_z, beta_z senza default); pulsante 0 start, 1 stop+learn; scrive `/root/thesis_ws/dmp_weights.yaml` e `/root/thesis_ws/demo_raw.csv` (src/haptic_dmp_learning/config/params.yaml:8-9). NON fa parte della catena a pulsante singolo e non è referenziato da nessun launch.

**Vedere il movimento**: il recorder pubblica `/target_pose` durante la registrazione (anchorata se `anchor_enabled`), il controller a impedenza segue → Gazebo (GUI: `DISPLAY=:0 ign gazebo -g`, oppure RViz con il launch non-headless).

**Uscite (percorsi; i primi quattro FISSI nel codice, Appendice A punto 5)** — `ws_root = ~/thesis_ws` (HOME del container = `/root` → `/root/thesis_ws`; host `/home/lorenzo/thesis_ws`):

| File | Percorso | Dove è fissato |
|---|---|---|
| CSV demo | `/root/thesis_ws/demo_raw_<run_id>.csv` (`t,x,y,z,qw,qx,qy,qz,gripper_trigger`) | demo_replay_sync.launch.py:60-61 (sovrascrive `output_demo_csv_path`) |
| joint_states | `/root/thesis_ws/demo_raw_<run_id>_joint_states.csv` | derivato da `deriveJointStatesCsvPath` (joint_states_csv_io.cpp:66-81; topic `/joint_states`) |
| odometria cubo | `/root/thesis_ws/demo_raw_<run_id>_target_odom.csv` | `deriveTargetOdomCsvPath` (live_demo_recorder_node.cpp:23-31); `record_target_odom` default true, topic `/free_target_object/odometry` |
| pesi DMP classica | `/root/thesis_ws/dmp_weights_<run_id>.yaml` | demo_replay_sync.launch.py:62 |
| (manuale, senza orchestratore) | `/root/thesis_ws/real_demo/live_demo_raw.csv`, `real_demo/live_demo_dmp_weights.yaml` | src/haptic_dmp_learning/config/params.yaml:33-34 (stessi nomi a ogni run: sovrascrittura) |

I file `demo_raw_*.csv` e `dmp_weights_*.yaml` sono ignorati da git (.gitignore). La directory `real_demo/` è tracciata.
I pesi `dmp_weights_<run_id>.yaml` NON entrano nella catena S1: i pesi ProDMP si rifanno in F3 dal CSV.

**Successo (log)**: orchestratore: `/clock is live`, `recording stack launched; waiting for start button`, `start button: spawning target and starting the 5.00 s sync window`, `sync reached`, `synced start edge [1,0] emitted ... recorder is live.`, `stop button: republished [0,1]`; recorder: `Recording started.`, `Recording stopped. N samples collected.`, `Raw demo saved to ...`, `Joint states (N samples, ...) saved to ...`, `DMP + Quaternion DMP learned and saved to ...`. Verifica: i tre CSV + il YAML esistono; `ls /root/thesis_ws/demo_raw_*` e `ls /root/thesis_ws/dmp_weights_*.yaml`.
**Tempo**: durata della demo = durata del gesto + 5 s di sync; non misurato nel repo.
**Errori noti**: `no message received on /clock within 10 s` (bringup assente/velocity); `no message on /free_target_object/odometry within 30 s`; `joint_states ... 0 valid samples ... Joint-states CSV NOT written` (Gazebo/RSP non attivi → poi il fit produce `end_joints_rad: null` e il planner lo rifiuta); `button sequencer ... ignored: debounce` (pressioni < 0.30 s); `ignored: orchestrator not ready for start yet` (pressione prima di `waiting for start button`); `demo already STOPPED (restart the node)`.

---

## F3 — Fit ProDMP (`mp_tool fit`)

**Scopo**: da `demo_raw_<id>.csv` (+ `_joint_states.csv` accanto) produrre `weights.yaml` e `weights_demo_params.yaml` per planner ed executor.

**Precondizioni**: la demo va TRONCATA al trigger: `python3 tools/trim_demo_at_trigger.py <CSV grezzo>` scrive `<stem>_trim.csv` e `<stem>_trim_joint_states.csv` (taglia alla prima riga con `gripper_trigger != 0`, non sovrascrive). Il fit validato usa il file **`_data/demos/demo_raw_20261014T150851_trim.csv`** (le demo archiviate stanno in `/root/thesis_ws/_data/demos/`, non nella radice); il CSV grezzo dello stesso giorno NON riproduce il run del 15/10 (tau 34.966 s invece di 24.474 s, sha256 dei pesi diverso). CSV e `<stem>_joint_states.csv` devono stare nella stessa cartella (altrimenti `start_joints_rad`/`end_joints_rad` restano `null` in silenzio: demo_params.cpp:396-398). Binario `tools/dmp_offline_test/build/learn_and_test_prodmp` presente (13 ott 17:36, 655728 B); `mp_tool` lo richiede (`require_binary`, mp_tool.py:256). Verificato il 23/10: il fit sul `_trim` con i parametri sotto riproduce in modo bit-identico `weights.yaml` e `weights_demo_params.yaml` del run `runs/20261015_095418_fit_demo_20261014T150851` (sha256 `a0a20fb610facf10cdce2231a7501893178773322317c760247bc8449bdc2265`).

**Comando (container franka, P; parametri di produzione, tutti uguali ai default di `prodmp_features.yaml`)**:

```bash
python3 tools/mp_tool/mp_tool.py fit /root/thesis_ws/_data/demos/demo_raw_20261014T150851_trim.csv --algo prodmp \
  --n-basis 80 --ridge-lambda 1e-10 --window 0.05 --fix-goal --with-orientation --label fit_demo_20261014T150851
```

Per una demo nuova: stesso comando sul CSV `_trim` di quella demo (`/root/thesis_ws/demo_raw_<run_id>_trim.csv` con il `run_id` dell'orchestratore), cambiando `--label`.

- Opzioni verificate con `--help`: `--algo {dmp,prodmp}` (default prodmp), `--label`, `--n-basis`, `--ridge-lambda`, `--window`, `--fix-goal/--no-fix-goal`, `--with-orientation/--no-with-orientation`, `--satellite-center X Y Z --satellite-axis X Y Z --satellite-omega-deg-s --grasp-point {kP0,kP90,kP180,kP270} --phase-at-contact-deg` (le cinque «tutte insieme o nessuna»; senza, `satellite_at_demo: null`).
- Default letti da `src/haptic_dmp_learning/config/prodmp_features.yaml` (`num_basis: 80`, `ridge_lambda: 1.0e-10`, `position_filter.window_sec: 0.05`, `fix_goal_to_demo_endpoint: true`); `--with-orientation` è già ON di default in mp_tool.py:277. I flag sopra sono quindi ridondanti ma rendono esplicita la produzione.

**Uscite**: `runs/<YYYYmmdd_HHMMSS>_fit_<label>/` creata automaticamente sotto `REPO_ROOT/runs` (mp_tool.py:55,130-136; MAI sovrascritta, errore se esiste) con: `weights.yaml`, `weights_demo_params.yaml`, `replay.csv`, `summary.csv`, `plot.png`, `data/`, `plots/`. Non va spostata: `s1_run.py --weights-dir` accetta qualunque cartella contenente i due file (s1_run.py:1067-1072). `runs/` NON è ignorata da git (oggi 2 file tracciati).
**Successo**: stampa `[mp_tool] ProDMP fit: n_basis=80 ridge_lambda=1e-10 window=0.05 fix_goal=True with_orientation=True`, `RMSE overall ... endpoint pos error ...` e `Run directory: ...`.

**Verifica hash dei pesi**:

```bash
R=runs/20261015_095418_fit_demo_20261014T150851      # esempio: la cartella stampata dal fit ("Run directory: ...")
sha256sum $R/weights.yaml; grep weights_sha256 $R/weights_demo_params.yaml     # devono coincidere
grep -E "start_joints_rad|end_joints_rad" $R/weights_demo_params.yaml           # NESSUNO dei due deve essere null
```

(Verificato su `runs/20261015_095418_fit_demo_20261014T150851`: `a0a20fb6...c2265` in entrambi.) A runtime: il planner rifiuta se l'hash non coincide (`verifyWeightsAlignment`, selection_inputs.cpp:56) e l'executor ricalcola l'hash del file caricato e lo confronta con quello del `GraspCommand` (prodmp_gazebo_executor_node.cpp:1031-1032).
**Tempo**: non nel repo per il fit.
**Errori noti**: `demo CSV not found`; `the satellite options must be given ALL together`; `WARNING: demo_params NON scritto (timestamp ripetuti)` (exit 0, nessun demo_params); `ERROR: demo_params non scritto` (exit 3); `end_joints_rad: null` → il planner rifiuta il file (docs/COMMAND_REFERENCE.md §3.8, non verificato nel codice del planner).

---

## F4 — Lancio del cubo e replay con nuove condizioni (`tools/s1_run.py`)

**Scopo**: un run completo: bringup, spawn del cubo rotante, bag, executor ProDMP, planner, gate sul goal, report.

**Precondizioni**: F0 pulita e senza simulazioni attive (il runner fa `ensure_clean` e uccide con SIGKILL); `--weights-dir` con `weights.yaml` + `weights_demo_params.yaml` (F3); gripper prismatico applicato; DISPLAY solo con `--gui`. **Non lanciare** Gazebo a mano: il runner avvia il suo (F1 deve essere spenta).

**AVVISI PRIMA DI LANCIARE**
- **Il profilo validato NON è quello di default di `s1_run.py` né di `s1_campaign.py`.** Il dry run del 23/10 con i soli `--planner-param` del profilo, senza `--squeeze -0.05` e `--stop-cube` e con yaw 180, ha dato 0/3 successi (C0, C5, C12: dita che perdono contatto, slip 3.8–13.8 mm, gap 23.8–34.5 mm); gli stessi tre con gli argomenti della campagna hanno dato 11/11 (rerun del 24/10). Dal log dell'executor: la campagna stampa `gripper: SQUEEZE cmd=-0.0500 m` subito dopo `CLOSE done`, il dry run no.
- **`s1_campaign.py` ai default NON riproduce il profilo validato** (squeeze non passato, `stop_cube` False, `spawn_yaw` 180, nessun `--planner-param`). Dal commit 0ae4814 inoltra anche `--planner-param` (ripetibile), `--axis-tilt-deg` e `--axis-lean-deg`, che valgono per l'intera griglia (non sono assi). Per riprodurre una cella servono `--arm v2`, `--squeeze -0.05`, `--stop-cube true`, `--spawn-yaw`, `--omega-deg-s`, `--center-xyz`, tilt/lean e i 13 `--planner-param` del comando qui sotto (esempio completo in F5). Il tag di cartella non codifica `--planner-param`, tilt e lean: usare un `--outroot` diverso per ogni profilo.
- **Durante un run NON lanciare comandi `ros2` che citino il controllore o i suoi nodi** (`ros2 param get /cartesian_impedance_controller …`, `ros2 control …`, `ros2 node …`): a fine run `ensure_clean` confronta le righe di comando con `LEFTOVER_PATTERNS` (s1_run.py:94-100) e le scambia per processi residui → `S1_RUN FAIL: processi residui dopo il run: cartesian_impedance_controller`, rc=3 (osservato in dryrun_C0). Lo stesso vale per qualunque processo che citi `prodmp_gazebo_executor_node`, `grasp_planner_node`, `free_target_object`, `parameter_bridge`, ecc. Se servono `ros2 param list/get`, avviarli solo dopo `WAITING for a GraspCommand` nell'`executor.log`, interrogando `/grasp_planner_node` o `/prodmp_gazebo_executor_node`, e farli terminare PRIMA della fine del run (funzionato con attesa lunga del TRIGGER; con yaw campagna l'attesa è ~10 s: non c'è margine). Non avviare nessun nodo ROS (watcher, `ros2 daemon`) prima di `s1_run`: il runner fa `ros2 daemon stop` e `rm /dev/shm/fastrtps_*` e il processo preesistente non riesce più a raggiungere i nodi.

**Comando COMPLETO validato (C0 della campagna; container franka, shell con P; verificato il 24/10: 11/11, k2 B θ*=145.3°)**:

```bash
python3 tools/s1_run.py --weights-dir runs/20261015_095418_fit_demo_20261014T150851 --tag C0_prova \
  --arm v2 --kt 2000 --depth 0.0125 --follow-s 3.0 --hold-s 120.0 \
  --squeeze -0.05 --stop-cube --stop-cube-delay-s 0.0 --post-s 45.0 --sim-timeout 500.0 \
  --spawn-yaw-deg -124.3 --omega-deg-s -2.0 --center-xyz 0.7,0.0,0.77 \
  --axis-tilt-deg 0.0 --axis-lean-deg 0.0 \
  --planner-param s1.use_clearance=true --planner-param s1.min_clearance_mm=2.0 \
  --planner-param s1.use_margin=true --planner-param s1.min_margin_deg=28.0 \
  --planner-param s1.finalists_n=40 --planner-param s1.posthoc_cluster_deg=10.0 \
  --planner-param s1.theta_step_deg=2.0 \
  --planner-param s1.use_w=true --planner-param s1.kappa_w=7.5 \
  --planner-param s1.kappa_t=0.0 --planner-param s1.tau_j_mm=2.0 \
  --planner-param 's1.rollout_q_ref=[0.0,-0.785398163397,0.0,-2.356194490192,0.0,1.570796326795,0.785398163397]'
```

- Nessun `--gui` (la campagna l'ha usato solo in C12). `--arm v2` aggiunge da sé `s1.mode=v2` e `s1.branch_rule=rotation` (non ripeterli, RunError). **Tutti i valori `double` vanno scritti con il punto**: `28.0`, `10.0`, `0.0`, `2.0`, `7.5`; `28`, `10`, `0` fanno fallire il planner (`setting it to {integer} is not allowed`). `s1.rollout_q_ref` è passato esplicitamente dalla campagna (stesso valore del YAML riga 94). `--depth/--follow-s/--hold-s/--stop-cube-delay-s/--post-s/--sim-timeout` sono ai default ma la campagna li ha registrati negli stessi valori.
- Per un'altra cella cambiano SOLO `--spawn-yaw-deg`, `--omega-deg-s`, `--center-xyz`, `--axis-tilt-deg`, `--axis-lean-deg` (tabella sotto). **Lo yaw di spawn NON è `--spawn-yaw-deg auto`**: la campagna usa `yaw0 = wrap(θ*_selezionato − ω·45.2)` in (−180, 180], con θ* del planner per quella cella (verificato su tutte e 16 le celle; per C0, C5, C12 → −124.3, −89.6, −13.6). `auto` usa invece `θ*_ref = −163.5` fisso (`s1_run.auto_spawn_yaw_deg`, s1_run.py:159-165) e dà −73.1° per ω = −2 in tutte e tre le celle: NON coincide. Quindi per una cella nuova si ricava prima θ* (con la procedura qui sotto, o dalla tabella) e poi si fissa lo yaw. Con yaw sbagliato la presa è identica ma l'attesa pub→TRIGGER diventa lunga (108–164 s con 180° contro ~9–10 s).
**Procedura in due passi per una cella nuova** (verificata il 24/10 su C0, C5, C12: k, ramo e θ* identici alla campagna).

**(a) θ\* offline, senza Gazebo.** `_gate_out/step3_cells_gate` (sorgente `tests/golden/g25/step3_cells.cpp`) esegue la STESSA selezione S3 del profilo congelato (modalità v2, ramo `rotation`, clearance ≥ 2 mm, margine ≥ 28°, 40 finalisti). È linkato contro ROS e gira solo nel container `franka`, ma si lancia dall'host. Il CWD DEVE essere la radice del repo (pesi in `tests/golden/g25` e `fer_flat_effort.urdf` sono relativi):

```bash
docker exec franka bash -c "source /root/env_thesis.sh && cd /root/thesis_ws && ./_gate_out/step3_cells_gate --input tests/golden/g25/g25_cells_input.csv --cell C0,C5,C12 --no-csv"
```

Righe delle celle in uscita (colonne: `CAND` = k e ramo, `THETA*` [°], `d [mm]`, `w`, `J`, `MARG [deg]` = margine previsto, `CLR [mm]` = clearance prevista, `EXAM` = finalisti esaminati, `D_MG/D_CL/D_RO` = scartati per margine/clearance/rollout, `PREVISIONE`):

```
C0   | -2.00 | kSelected | k2B | 145.3  |   7.194 | 0.06776 |  27.382 | 30.61 | 23.44 |  1 | 0 | 0 | 0 | PREVISTA_RIUSCITA
C5   | -2.00 | kSelected | k3A | -180.0 |  31.722 | 0.05765 |  53.122 | 35.00 | 14.87 |  2 | 1 | 0 | 0 | PREVISTA_RIUSCITA
C12  | -2.00 | kSelected | k0A | -104.0 | 154.531 | 0.08078 | 173.401 | 28.42 | 22.47 | 25 | 16| 3 | 5 | PREVISTA_RIUSCITA
```

Se `_gate_out/step3_cells_gate` manca lo ricompila `docker exec franka bash -c "cd /root/thesis_ws && bash tests/golden/g25/build_and_run_g25.sh"` (compila e confronta con il golden).

**(b) Yaw di spawn**: `yaw0 = wrap(θ* − ω·45.2)`, ω in °/s con segno, 45.2 s = `T_total`, `wrap` in (−180, 180]. Esempio C0: θ* = 145.3, ω = −2 → 145.3 − (−2)(45.2) = 145.3 + 90.4 = 235.7 → 235.7 − 360 = **−124.3°** → `--spawn-yaw-deg -124.3`. Analogamente C5: −180.0 + 90.4 = −89.6; C12: −104.0 + 90.4 = −13.6.

Limite: verificato su 16/16 celle della campagna (selezione identica alla previsione G25, ω da −1 a −3, centri non nominali); il driver non usa la fase reale dello snapshot (usa `--snap-theta-rad -1.6727…`, `--snap-t-s 18.318` e non legge `run_meta.json`); i due tentativi di C14 hanno scelto lo stesso candidato con fasi diverse. Dal driver si confrontano `k, branch, theta_deg, d_mm, w, pred_margin_deg, pred_clearance_mm` con `theta_star`, `d_pos`, … della riga di `results.csv` / `report.json`.

**Aggiungere una cella al CSV di input.** Formato (header di `tests/golden/g25/g25_cells_input.csv`; 17 campi separati da virgola letti per POSIZIONE, `step3_cells.cpp:50-71`; meno di 17 campi → errore; righe vuote e righe che iniziano con `#` ignorate):

| Colonna | Unità | Usata dal driver | Significato |
|---|---|---|---|
| `cell` | — | sì | etichetta, filtrata da `--cell` e stampata |
| `omega_deg_s` | °/s con segno | sì | velocità angolare del satellite (→ rad/s) |
| `cx,cy,cz` | m | sì | centro di rotazione nel mondo |
| `alpha_deg`, `phi_deg` | ° | **no** (solo letti) | inclinazione e azimut dell'asse: servono a generare `ax…nz`, non a calcolare nulla nel driver |
| `ax,ay,az` | — | sì | asse di rotazione unitario (il driver lo normalizza): `a = (sin α cos φ, sin α sin φ, cos α)` |
| `qx,qy,qz,qw` | — | **no** (solo letti) | quaternione di riferimento `tilt_quat_xyzw` (ordine xyzw) |
| `nx,ny,nz` | — | sì | normale della faccia nel corpo (`face_normal_body`, normalizzata dal driver): `n = R_tilt·(1,0,0)` |

Tutte le 17 colonne sono obbligatorie come posizioni; quelle derivate (`ax…nz`) NON vanno scritte a mano ma generate da α, φ con `make_cells.process_row(nome, omega, offset_mm, alpha, phi)` (usa `tilt_axis/tilt_rotation/tilt_quat_xyzw` di `s1_run`; `offset_mm` è lo scostamento in mm dal centro nominale 0.70, 0.0, 0.77). Esempio: cella inventata α = 10°, φ = 45°, ω = −1, centro 0.75, −0.05, 0.82 (come C9), da eseguire dalla radice del repo. Non modificare il CSV in `tests/`: lavorare su una copia in `_gate_out/`.

```bash
cp tests/golden/g25/g25_cells_input.csv _gate_out/g25_cells_test.csv
python3 - <<'EOF'
import sys
sys.path[:0] = ["tools", "tests/golden/g25"]   # make_cells.py cerca tools/ con un percorso relativo che non vale in tests/
import make_cells
_, row = make_cells.process_row("X1", -1.0, (50, -50, 50), 10.0, 45.0)   # offset (+50, -50, +50) mm => centro 0.75, -0.05, 0.82
open("_gate_out/g25_cells_test.csv", "a").write(row + "\n")
print(row)
EOF
docker exec franka bash -c "source /root/env_thesis.sh && cd /root/thesis_ws && ./_gate_out/step3_cells_gate --input _gate_out/g25_cells_test.csv --cell X1,C9 --no-csv"
```

Riga prodotta: `X1,-1.0,0.75,-0.05,0.8200000000000001,10.0,45.0,0.12278780396897285,0.12278780396897282,0.984807753012208,-0.061628416716219346,0.06162841671621935,0.0,0.9961946980917455,0.992403876506104,-0.00759612349389599,-0.12278780396897285`. Uscita attesa: X1 e C9 identiche (`k2B`, θ\* 131.2, d 79.674 mm, MARG 35.00, CLR 11.24, `PREVISTA_RIUSCITA`); yaw = wrap(131.2 + 45.2) = 176.4.
- Comando di base (solo per prove, NON valida il profilo): `python3 tools/s1_run.py --weights-dir runs/20261015_095418_fit_demo_20261014T150851 --tag prova_base`.
- Tempo misurato (rerun 24/10): C0 120 s, C5 117 s, C12 117 s di wall time, RTF ≈ 0.98–0.99.

**Argomenti comuni a tutte le 16 celle della campagna (identici in tutti i `run_meta.json` del backup 21/10)**: `--weights-dir runs/20261015_095418_fit_demo_20261014T150851`, `--arm v2`, `--kt 2000`, `--depth 0.0125`, `--follow-s 3.0`, `--hold-s 120.0`, `--squeeze -0.05`, `--stop-cube` (`--stop-cube-delay-s 0.0`), `--post-s 45.0`, `--sim-timeout 500.0`, `--theta-star-ref-deg -163.5` e `--t-total-s 45.2` (default, non usati: yaw fisso), `spawn_yaw_mode=fixed`, `extra-exec-param` vuoto, e i 13 `--planner-param` del comando sopra. 16/16 `success: true`. git_head: C0 `2c6ae15`, C1–C15 `dbd87bb`.

**Parametri per cella (da `run_meta.json` + `report.json`)**

| Cella | alpha (`--axis-tilt-deg`) | phi (`--axis-lean-deg`) | omega (`--omega-deg-s`) | centro (`--center-xyz`) | `--spawn-yaw-deg` | θ* selezionato [deg] | k, ramo |
|---|---|---|---|---|---|---|---|
| C0 | 0 | 0 | −2 | 0.7,0.0,0.77 | −124.3 | 145.3 | k2 B |
| C1 | 0 | 0 | −1 | 0.7,0.0,0.77 | −169.5 | 145.3 | k2 B |
| C2 | 0 | 0 | −1.5 | 0.7,0.0,0.77 | −146.9 | 145.3 | k2 B |
| C3 | 0 | 0 | −2.5 | 0.7,0.0,0.77 | −101.7 | 145.3 | k2 B |
| C4 | 0 | 0 | −3 | 0.7,0.0,0.77 | −27.6 | −163.2 | k0 A |
| C5 | 20 | 0 | −2 | 0.7,0.0,0.77 | −89.6 | −180.0 | k3 A |
| C6 | 20 | 90 | −2 | 0.7,0.0,0.77 | −71.4 | −161.8 | k0 A |
| C7 | 20 | 180 | −2 | 0.7,0.0,0.77 | −71.6 | −162.0 | k1 A |
| C8 | 20 | 270 | −2 | 0.7,0.0,0.77 | −124.3 | 145.3 | k2 B |
| C9 | 10 | 45 | −1 | 0.75,-0.05,0.82 | 176.4 | 131.2 | k2 B |
| C10 | 20 | 135 | −1.5 | 0.65,0.05,0.82 | −76.2 | −144.0 | k0 A |
| C11 | 15 | 315 | −1.75 | 0.7,-0.05,0.72 | −153.6 | 127.3 | k2 B |
| C12 | 5 | 225 | −2 | 0.75,0.05,0.72 | −13.6 | −104.0 | k0 A |
| C13 | 20 | 300 | −2.25 | 0.65,0.0,0.82 | −83.7 | 174.6 | k3 A |
| C14 | 15 | 90 | −2.5 | 0.65,-0.05,0.72 | −63.0 | −176.0 | k1 A |
| C15 | 10 | 180 | −3 | 0.75,0.0,0.82 | −12.4 | −148.0 | k0 A |

Le celle del backup includono anche `val_C14_attempt1_FAIL_executor_stamp_future` e `val_C5_attempt1_FAIL_twist_frame` (primi tentativi falliti, non validi).

**Rerun del 24/10 (stessi argomenti, tag `val_<C>_rerun`)**: C0, C5, C12 → 11/11, stessa selezione (k, ramo, θ*, d_pos, costo identici), `SQUEEZE cmd=-0.0500 m` presente, wz dopo lo stop 0.143 / 0.0 / 0.107 deg/s (campagna 0.134 / 0.0 / 0.163), `translational_stiffness` tornato a 200.0 e nessun processo residuo. Differenze sui criteri ≤ 0.07 su gap/f_sum/slip (es. C0 f_sum 9.2177 contro 9.1996).

**Opzioni (tutte da `--help`; elenco con default e riga nell'Appendice B)** — significato fisico:

| Grandezza | Opzione | Dove arriva |
|---|---|---|
| ω (°/s con segno) | `--omega-deg-s` (−2.0) | `wz` [rad/s] del launch del cubo (BODY frame) + `satellite_rotation_angular_velocity_deg_s` di executor e planner (`omega_values`, s1_run.py:135-140; coerenza entro 0.0017 rad/s, :143-150) |
| centro (m) | `--center-xyz X,Y,Z` (0.70,0.0,0.77) | `x:= y:= z:=` dello spawn, `satellite_rotation_center` (executor e planner), `cube_geometry.center_world` e gate sul goal (:699-701, 730-735, 794-796) |
| inclinazione α (°) | `--axis-tilt-deg` (0.0, max 20) | asse `a=(sinα cosφ, sinα sinφ, cosα)`; roll/pitch/yaw estrinseci di spawn = `R_tilt·Rz(yaw)`; `satellite_rotation_axis`, `satellite_q_ref`, `cube_geometry.axis_world`, `cube_geometry.face_normal_body` scritti dal runner (:281-298, 784) |
| azimut φ (°) | `--axis-lean-deg` (0.0, in [0,360)) | `R_tilt = Rot(n, α)`, `n=(−sinφ, cosφ, 0)`; φ = β − 90 rispetto ai test G22 (:215-231) |
| yaw di spawn (°) | `--spawn-yaw-deg DEG\|auto` (**180.0**) | `yaw:=` (α=0, rad) oppure componente di `R_tilt·Rz(yaw)`; `auto` = `theta_star_ref − ω·T_total` in (−180,180] (`--theta-star-ref-deg` −163.5, `--t-total-s` 45.2; per −2 °/s ≈ −73.1°) |
| candidato forzato | `--planner-param s1.force=true` + `s1.force_k` (0..3), `s1.force_theta_deg`, `s1.force_psi_deg`, `s1.force_flip` (−1/0/1), `s1.force_ignore_constraints` | planner (grasp_planner_node.cpp:166-175); `--help` cita solo `s1.force=true` come esempio |
| modalità | `--arm {legacy,d_only,v2}` (nessuno) | espanso in `--planner-param` (:69-73) |
| GUI | `--gui` / `--no-gui` (no) | `DISPLAY=:0 exec ign gazebo -g` (:907-908) |

**Regole di coerenza (dal codice)**
1. Un solo `--center-xyz` per spawn, executor, planner e gate; il planner fallisce se il primo campione di odometria dista > 10 mm dal centro (grasp_planner_example.yaml:12-13, `checkFirstOdomSample`) e `cube_geometry.center_world` deve essere uguale a `satellite_rotation_center` (:36).
2. Lo yaw di spawn fissa solo la fase del cubo al TRIGGER, non la geometria della presa: va scelto con `wrap(θ* − ω·45.2)` (vedi sopra) per avere attesa ≈ 9–10 s; `s1_campaign` passa a `auto` da sola se ω ≠ −2 (s1_campaign.py:99-106) ma `auto` NON dà gli yaw della campagna; `s1_run` ha default 180.
3. Con `--axis-tilt-deg > 0` non passare `satellite_rotation_axis`, `satellite_q_ref`, `cube_geometry.axis_world`, `cube_geometry.face_normal_body` come `--planner-param`/`--extra-exec-param` (RunError, :761-769).
4. Attesa `wait_s_actual` (TRIGGER − pubblicazione del `GraspCommand`): finestra ammessa **[5, 5 + 360/|ω|] s** (`WAIT_MIN_S=5.0`, s1_report.py:94-95,232-236); fuori finestra il report avvisa ma non cambia il successo. `min_delay_s=5.0` è passato al planner (s1_run.py:744). L'intervallo «5–30 s» vale solo per |ω| = 14.4 °/s; per −2 °/s è [5, 185] s.
5. `--stop-cube` richiede `--follow-s > 0` e `--post-s ≥ follow_s + delay + 1` (:1164-1171). `--post-s` di default = max(45, follow_s + delay + 4).
6. Gate sul goal: `x_c` atteso = `RIM_X_C_MM − depth·1000` con `RIM_X_C_MM = 155.1` (s1_run.py:75), tolleranza `GATE_TOL_MM = 0.5` (:76); nel codice `expected_xc = RIM_X_C_MM - a.depth * 1000.0` (:971) e il run fallisce se `abs(xc - expected_xc) > GATE_TOL_MM`. Esempio: `--depth 0.0125` → 155.1 − 12.5 = **142.6 mm** ± 0.5 (accettato 142.1–143.1); `--depth 0.0` → 155.1 mm. `--depth` va coerente con ciò che il planner applica (`cube_geometry.grasp_depth_m`).
7. `|ω|` ≠ 0 e finito; `--sim-timeout` è in tempo SIMULATO.
8. `--weights-dir` e `--outdir/--tag`: se `bag/` esiste già serve `--overwrite` o altro `--tag`.

**Comandi di fondo (s1_run.py:794-825, tutti col preambolo P e `source env_thesis.sh`)**: bringup (F1); spawn `ros2 launch free_target_object free_target_object.launch.py x y z yaw wz` (+`roll pitch` se α>0); bag `ros2 bag record -o <outdir>/bag /clock /tf /tf_static /joint_states /free_target_object/odometry /target_pose /grasp_command /gripper_position_cmd`; executor `ros2 run haptic_dmp_learning prodmp_gazebo_executor_node --ros-args --params-file src/haptic_dmp_learning/config/params.yaml --params-file src/satellite_grasp_planner/config/grasp_planner_example.yaml -p ...`; planner `ros2 run satellite_grasp_planner grasp_planner_node --ros-args --params-file .../grasp_planner_example.yaml -p ...`. Liste esatte dei `-p` per il run: `run_meta.json` → `executor_params_passed`, `planner_params_passed` (s1_run.py:1194-1195). Il planner parte DOPO che l'executor stampa «gripper: OPEN» e «WAITING for a GraspCommand».

**Uscite** (`--outdir`, default `<repo>/s1_runs/<tag>`, s1_run.py:1143): `bag/` (con `metadata.yaml`), `bringup.log`, `spawn.log`, `bag.log`, `executor.log`, `planner.log`, `gui.log` (se `--gui`), `run_meta.json`, `report.json`, `report.txt`, `timeseries.csv`.
**Successo (log)**: `preflight: ...`, `YAML: translational_stiffness = <kt>`, `controller attivo (dopo N s)`, `executor pronto, 'gripper: OPEN'`, `gate goal: x_c = ... mm (atteso ...)`, `GraspCommand ACCEPTED`, `AT_END a t=...`, `<N> s simulati dopo AT_END: stop bag`, `report: .../report.txt  success=True|False`, poi `YAML: translational_stiffness ripristinato a 200.0`. Codice di uscita 0 = run completo (la presa riuscita è `report.json["success"]`).
**Tempo**: setup fino a ~70 s (controller) + 5 s + 2 s; executor pronto entro 60 s; spawn-conferma entro 30 s; da spawn a contatto ≈ 45.2 s sim per ω = −2 con yaw `auto` (attesa ≈ 15 s; con yaw 180 misurati 164 / 146 / 108 s per C0 / C5 / C12, non «≈ 138 s» come nel commento di s1_run.py:797-805; con lo yaw di campagna ≈ 9–10 s); AT_END + ≥ 45 s sim; timeout 500 s sim. Durata wall misurata: 226–294 s (dry run, yaw 180) e 117–120 s (rerun con yaw di campagna), RTF ≈ 0.98.
**Errori noti** (una riga `S1_RUN FAIL: <motivo>` su stderr; codici: 0 ok, 1 fallito, 3 processi residui → il run successivo NON è sicuro, 130 interrotto): `bringup terminato prematuramente (vedi bringup.log)`; `controller non attivo entro 70 s`; `spawn del cubo non confermato ('OK creation of entity.' assente)`; `executor non pronto entro 60 s ...`; `eccezione nel log dell'executor`; `gate goal fallito: x_c=... (grasp_depth_m non applicato?)`; `timeout in tempo simulato ... senza AT_END`; `/clock fermo da 120 s reali`; `bag non chiuso entro 15 s`; `--stop-cube: nessun topic */cmd_vel ...`; `omega non coerente`; `asse non coerente`.

---

## F5 — Report e analisi

**`tools/s1_report.py`** (rilettura di un bag già registrato; richiede rclpy/rosidl e pinocchio → container con P):

```bash
python3 tools/s1_report.py s1_runs/C0_prova/bag
```

Opzioni: `--executor-log`, `--meta`, `--criteria` (default `tools/s1_success.yaml`), `--outdir`, `--csv`, `--urdf`, `--planner-log`, `--tag`, `--min-window-s`.

Default: `executor.log`, `run_meta.json` e `planner.log` accanto a `bag/`; `--outdir` = cartella del run. Uscite: `report.json`, `report.txt` (≤ 60 righe), `timeseries.csv` (passo 0.5 s da AT_END), eventuale riga in `--csv` (colonne fisse `CSV_COLUMNS`).
**Criteri** (`tools/s1_success.yaml`; t = AT_END, tempo sim; ogni criterio NaN = non soddisfatto): `f_sum_plateau_10_20s` ∈ [8, 11] mm; `min_f_plateau_10_20s` ≥ 1.0 mm; `slip_tang_axial_3_15s` ≤ 0.5 mm; `pad_straddle_2s/5s/15s` = true; `cube_disp_pre_end_mm` < 0.3; finestra registrata ≥ `min_window_after_at_end_s` = 40 s (sennò i 3 criteri di fondo sono NaN e il run non è un successo); `gap_end_mean_last10s` ∈ [8, 11] mm; `pad_straddle_end` = true; `slip_tang_axial_3_end` ≤ 0.5 mm; `min_joint_margin_deg` ≥ 10 (criterio 11, post hoc). Solo riportati: `tcp_cube_var_close_3_8s`, `tcp_cube_var_close_3_15s`, `wz_cube_mean_4_8s`, `yaw_slope_6_12s`. La deadline sull'attesa (F4 regola 4) è solo un AVVISO.

**`tools/s1_campaign.py`** (griglia di run; ogni cella chiama `s1_run.py` con `--no-keep-alive`, `--no-gui` salvo `--gui`):

```bash
python3 tools/s1_campaign.py --weights-dir runs/20261015_095418_fit_demo_20261014T150851 --outroot s1_runs/camp_C0 \
  --arm v2 --squeeze -0.05 --stop-cube true --spawn-yaw -124.3 --omega-deg-s -2 --center-xyz 0.7,0.0,0.77 \
  --axis-tilt-deg 0.0 --axis-lean-deg 0.0 \
  --planner-param s1.use_clearance=true --planner-param s1.min_clearance_mm=2.0 \
  --planner-param s1.use_margin=true --planner-param s1.min_margin_deg=28.0 \
  --planner-param s1.finalists_n=40 --planner-param s1.posthoc_cluster_deg=10.0 \
  --planner-param s1.theta_step_deg=2.0 \
  --planner-param s1.use_w=true --planner-param s1.kappa_w=7.5 \
  --planner-param s1.kappa_t=0.0 --planner-param s1.tau_j_mm=2.0 \
  --planner-param 's1.rollout_q_ref=[0.0,-0.785398163397,0.0,-2.356194490192,0.0,1.570796326795,0.785398163397]' \
  --dry-run
```

Senza `--dry-run` lancia il run vero (una griglia di una cella = la C0 della campagna); `--skip-done` salta i tag già in `results.csv`. Una griglia di prova più grande: `--omega-deg-s -2 -3 --center-xyz 0.70,0.0,0.77 0.65,0.0,0.77 --arm legacy d_only v2`.

- Assi (liste): `depth` (0.0125), `kt` (2000), `spawn_yaw` (180; `auto` se ω varia), `follow_s` (3.0), `hold_s` (120), `squeeze` (non passato), `stop_cube` (False), `omega_deg_s` (−2), `center_xyz` (0.70,0,0.77), `arm` (nessuno). Griglia anche da `--grid yaml` (la CLI prevale; chiavi sconosciute = errore). Altre opzioni, uguali per tutta la griglia: `--sim-timeout` (500), `--gui`, `--extra-exec-param`, `--planner-param`, `--axis-tilt-deg`, `--axis-lean-deg` (questi ultimi tre passati a `s1_run` solo se indicati), `--overwrite`, `--skip-done`, `--dry-run`.
- `--dry-run` stampa `<tag> <json> yaw0=<deg>` per ogni cella senza lanciare nulla; il tag è `d<mm>_kt<>_y<yaw>_f<>_h<>_q<>[_stop]_w<ω>_c<cx>x<cy>x<cz>[_a<arm>]`.
- Uscite in `--outroot`: `results.csv` (colonne fisse di s1_report), `campaign_status.csv` (tag, rc, status, reason), `<tag>/` (`s1_run.log`, `s1_run.err`, `run_meta.json`, `report.*`, `bag/`). Nessun timeout wall; un run fallito non ferma la campagna; uscita 1 se qualcuno fallisce.

**Cella vs previsione offline**: vedi la «Procedura in due passi» e «Aggiungere una cella» in F4 (`step3_cells_gate`, senza Gazebo).

---

## F6 — Chiusura pulita

1. Run con `s1_run.py` senza `--keep-alive`: si ferma da solo dopo `--post-s` s sim da AT_END (chiude il bag, scrive il report, `finally` → `kill` dei gruppi di processi + `ensure_clean` + Kt ripristinato a 200.0; s1_run.py:1051-1064). Con `--keep-alive`: `touch s1_runs/C0_prova/STOP` (per un run con `--tag C0_prova`).
2. **Ctrl-C (SIGINT) o SIGTERM al runner**: stampa `INTERROTTO: chiusura del bag e cleanup`, chiude il bag (SIGINT al bag, attesa 15 s), segna `interrupted: true` in `run_meta.json`, `finally` come sopra, uscita **130** (`S1_RUN INTERRUPTED`); se esiste `bag/metadata.yaml` prova a scrivere il report. Un secondo Ctrl-C è ignorato (s1_run.py:1038-1050,1219-1230). `s1_campaign` NON gestisce l'interruzione: Ctrl-C ferma anche la griglia (il run corrente segue la stessa procedura).
3. Dopo ogni interruzione:

```bash
ps -eo pid,args | grep -E "ign gazebo|gz sim|ros2 bag record|parameter_bridge|grasp_planner_node|prodmp_gazebo_executor_node|dmp_gazebo_executor_node|controller_manager|robot_state_publisher|free_target_object|ros2 launch" | grep -vE "grep|bash -c"      # atteso: nessuna riga
grep -n "translational_stiffness" /root/ros_workspaces/ros2/franka_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml   # atteso 200.0
ls s1_runs/C0_prova/bag/metadata.yaml
```

Se restano processi: la pipe sopra seguita da `| awk '{print $1}' | xargs -r kill -9` li uccide per PID (mai `pkill -f`).

4. F1/F2 (shell manuali): Ctrl-C nelle shell A/B/C; l'orchestratore termina da sé i figli (`ros2 launch` del recorder e del cubo) — «Chiudi con Ctrl-C: l'orchestratore termina tutti i processi figli» (docs/COMMAND_REFERENCE.md §3.2, non verificato nel codice) → poi il controllo `ps` come sopra e, se restano, il «riavvio pulito» di F0.
5. Codice 3 di `s1_run` o `processi residui`: NON lanciare altri run finché la lista `ps` (comando di F0 (b)) non è vuota di processi reali.

---

## Appendice A — Punti non verificati

Stato dopo il dry run del 23–24/10/2026 (F0, F1, F3, F4, F5 eseguite; **F2 non eseguita**). Le voci sono lacune del repository o affermazioni non provate dal codice.

### A.1 Lacune (passi manuali non codificati)

1. **`start_joints_override_rad` incollato a mano nel runner**: costante `START_JOINTS = [-0.000009,-0.786572,0.000002,-2.378914,-0.000006,3.141609,0.785398]` (tools/s1_run.py:74), passata come `start_joints_override_rad` al planner (:739). Non è letta da `weights_demo_params.yaml` (che ha `start_joints_rad`) né da `FRANKA_START_POSE`. Copia identica in `tests/golden/g25/step3_cells.cpp:118`. Se cambiano posa iniziale/controller va aggiornata in due posti. Nel planner la chiave è commentata nel YAML (grasp_planner_example.yaml:54).
2. **Prefisso del nodo**: il runner passa ogni parametro del planner in due forme, `grasp_planner_node:<chiave>:=<v>` e `<chiave>:=<v>` (s1_run.py:752-757); `urdf_path` è dentro questo elenco (:738). Nel YAML `urdf_path` è commentato (grasp_planner_example.yaml:29) e il default del nodo è `$HOME/thesis_ws/fer_flat_effort.urdf` (grasp_planner_node.cpp:85-93). Un lancio manuale con la forma prefissata funziona solo se il nome nodo è esattamente `grasp_planner_node`; la ragione della doppia forma non è documentata («come in run_final2.py», :753).
3. **Ordine di AMENT**: sistemato dentro `s1_run.py` (`ros_cmd` + `s1_env.py`). Con l'`env_thesis.sh` attuale (`source /opt/ros/humble/setup.bash; source /root/thesis_ws/install/setup.bash`) thesis_ws è già davanti anche nelle shell manuali (verificato: output identico con e senza `eval`); l'`eval` del Preambolo P serve solo se quel file cambia. `franka_ws/install` contiene copie obsolete (13–15 ott) di `haptic_dmp_learning`, `grasp_monitoring`, `free_target_object`.
4. **Percorsi cablati**: `fer_flat_effort.urdf` nella radice (tracciato): `s1_run.py:55`, `s1_report.py:71`, `canonical_metrics.py:88`, `grasp_metrics.py:79`, `step3_cells.cpp:115` (relativo al CWD), default del planner (`grasp_planner_node.cpp:93`) e fixture dei gtest (`test_fixtures.hpp:35`); nessun comando nel repo che lo rigeneri. `/root/env_thesis.sh` (`S1_ENV_SH`, s1_run.py:43). `PLANNER_YAML`/`EXECUTOR_YAML` (:56-57) e `WS_DIR/s1_runs` (:1143, ora ignorata da git). Pesi/YAML default dei nodi demo `~/thesis_ws/...` (src/haptic_dmp_learning/config/params.yaml:8-9,33-34; `weights_yaml_path` dell'executor).
5. **Directory di output dei nodi demo**: `demo_replay_sync.launch.py:60-62` e le funzioni `*PathForRunId` dell'orchestratore (demo_replay_sync_orchestrator_node.cpp:650-668) fissano `$HOME/thesis_ws/demo_raw_<id>.csv`, `dmp_weights_<id>.yaml`, `prodmp_weights_<id>.yaml` (non configurabili; dipendono da `HOME`); joint_states e `_target_odom.csv` derivati dallo stesso stem. Per il fit si copia/rinomina a mano il CSV (la run di esempio ha etichetta `demo_20261014T150851`, quindi stem diverso da `demo_raw_*`) e i file `_joint_states.csv` devono seguire la stessa rinomina, altrimenti i giunti restano `null` senza errore.
6. **YAML dei guadagni fuori dal repo**: `/root/ros_workspaces/ros2/franka_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` (default di `CONFIG_YAML_FILE`, s1_run.py:44-47; override con `S1_CONTROLLERS_YAML`, non impostata). Verificato nel dry run: file regolare, identico (`diff`) a `src/franka_gazebo_overrides/franka_gazebo_controllers.yaml`; valori a riposo `translational_stiffness 200.0`, `rotational_stiffness 150`, `translational_damping 10`, `rotational_damping 1`, `nullspace_stiffness 0.2`, `joint1_nullspace_stiffness 100`. `s1_run` riscrive `translational_stiffness` con `re.subn` su TUTTE le righe che iniziano con quella chiave (:113-119, nessun `count`) e la riporta a **200.0** nel `finally` (:1062-1064, `KT_DEFAULT` :59): confermato 3/3 anche dopo rc=3. Un SIGKILL del runner lascerebbe Kt = `--kt` (default 2000): non provato.
7. **Skip initial alignment**: `ros2 param set /cartesian_impedance_controller skip_initial_alignment true` è fatto da `prepos_capture` (legacy) e dall'executor in grasp_command mode; per F2 senza `prepos_capture` non è codificato (richiesto da `anchor_enabled`, src/haptic_dmp_learning/config/params.yaml:38).
8. **`prepos_capture.py` non è in `tools/`** ma in `tools/legacy/`; i `tools/legacy/run_*` che lo usano dipendono dal vecchio flusso 1007.
9. **Profilo validato e `s1_campaign`**: dal commit 0ae4814 la campagna può esprimere `--planner-param`, `--axis-tilt-deg`, `--axis-lean-deg` (non sono assi: valgono per tutta la griglia, e il tag non li codifica). I default restano quelli non validati (`squeeze=None`, `stop_cube=False`, `spawn_yaw=180`). Lo yaw per cella non è codificato: formula `wrap(θ*_sel − ω·45.2)` ricavata dai `run_meta.json`, mentre `--spawn-yaw-deg auto` usa `θ*_ref = −163.5` fisso.
10. **`demo_params_path` passato all'executor ma non dichiarato** in `prodmp_gazebo_executor_node.cpp` (s1_run.py:706): confermato a runtime (assente da `ros2 param list` dell'executor, presente sul planner); il parametro è ignorato senza errore.
11. **Cubo del run vs cubo della demo**: i default del cubo nell'orchestratore (0.6, 0, 0.5) e quelli del runner (0.70, 0, 0.77) e del YAML del planner (0.75, 0, 0.35) sono tre set distinti; il runner li sovrascrive con `-p` (s1_run.py:716, 740-742) ma il YAML `/**` dell'executor (`satellite_rotation_center` 0.75,0,0.35) resta il default di lanci manuali.
12. **`ensure_clean`/`pkill` con `--pid=host`**: `pgrep -f` sul container vede i processi dell'host; il runner uccide con SIGKILL tutto ciò che combacia con 16 pattern (`free_target_object`, `spawner`, `controller_manager`, …) escludendo solo i propri antenati (s1_run.py:408-434). In più, qualunque comando esterno che citi uno dei pattern ancora in corso a fine run (es. `ros2 param get /cartesian_impedance_controller`) fa terminare il run con rc=3 (osservato in dry run, C0). Come `franka` sia stato creato (flag `--pid`) non è nel repo.
13. **Driver Geomagic**: comando di avvio, controllo di driver duplicati, topic nativi, `RMW_IMPLEMENTATION`/rete tra i due container: niente nel repo (`docker inspect` mostra solo `Cmd = bash`).
14. **Mancano nel repo**: il comando del driver, `docker_run.cfg` e il nome dell'immagine di `franka` (`run_with_thesis_mount.sh`), uno script di «riavvio pulito» manuale, l'ordine d'avvio tra `prepos_capture`, orchestratore e driver, e il comando per avviare `geomagic_touch`.
15. **Collisione nomi `runs/`**: `make_run_dir` usa il timestamp al secondo; due `fit` con stessa etichetta nello stesso secondo → errore «run dir already exists».
16. **`learn_and_test_prodmp`** va ricompilato a mano in `tools/dmp_offline_test/build/` (non fa parte di `colcon build`). Il comando di build non è documentato in un file dedicato, ma è nello script `tools/dmp_offline_test/04_basis_sweep/scripts/run_ridge_lambda_sweep_prodmp.sh:32-48` (`g++ -std=c++17 -O2 ...`, ricompila se i sorgenti sono più nuovi). Il binario (13 ott) è più vecchio dei sorgenti in `src/haptic_dmp_learning/src/core`, ma il fit F3 ha dato un risultato bit-identico al run del 15 ott.

17. **Riproducibilità dei pesi di riferimento (stato git verificato il 24/10)**: `runs/20261015_095418_fit_demo_20261014T150851/` (`weights.yaml`, `weights_demo_params.yaml`) è **presente ma IGNORATO** da git (`.gitignore:50`, regola `/runs/*`; `git ls-files` vuoto). Anche `_data/demos/demo_raw_20261014T150851_trim.csv` e `_data/demos/demo_raw_20261014T150851_trim_joint_states.csv` sono **presenti ma IGNORATI** (`.gitignore:64`, `_data/`), come il CSV grezzo e gli altri file della demo. Un checkout pulito NON ha né pesi né demo: vanno copiati dal disco del proprietario. Se i pesi mancano ma `_data/demos/` c'è, si ricostruiscono (stessi sha256, verificato il 23/10) con:

```bash
python3 tools/trim_demo_at_trigger.py /root/thesis_ws/_data/demos/demo_raw_20261014T150851.csv   # solo se i _trim non esistono (non sovrascrive)
python3 tools/mp_tool/mp_tool.py fit /root/thesis_ws/_data/demos/demo_raw_20261014T150851_trim.csv --algo prodmp \
  --n-basis 80 --ridge-lambda 1e-10 --window 0.05 --fix-goal --with-orientation --label fit_demo_20261014T150851
```

La cartella creata ha un nome con timestamp nuovo (`runs/<data>_fit_fit_demo_20261014T150851`): usarla in `--weights-dir`. Il binario `tools/dmp_offline_test/build/learn_and_test_prodmp` è richiesto (voce 16).

### A.2 Affermazioni: stato dopo il dry run

- **`thesis_ws/install` non deve esistere**: SMENTITA; è richiesto e usato (F0). Il rischio sono le copie obsolete in `franka_ws/install`.
- **Contenuto di `env_thesis.sh` / necessità dell'`eval` di `s1_env.py`**: CONFERMATO il contenuto; l'`eval` è ridondante con il file attuale.
- **Stringa `Successfully loaded controller cartesian_impedance_controller` (F1)**: SMENTITA; la stringa reale è `[spawner_cartesian_impedance_controller]: Configured and activated cartesian_impedance_controller`.
- **Copie nel bringup usate; `install` = symlink, nessun rebuild per il YAML**: CONFERMATA (`diff` vuoto, symlink).
- **Mappatura del profilo congelato (F4)**: CONFERMATA (dump `ros2 param get` su C5 e selezione k/ramo/θ* uguale alla tabella nelle 3 celle).
- **`demo_params_path` ignorato dall'executor**: CONFERMATA.
- **Tempi: «≈ 138 s» di attesa con yaw 180**: SMENTITA come valore generale (164 / 146 / 108 s secondo la cella); la finestra [5, 5 + 360/|ω|] s è rispettata.
- **`learn_and_test_prodmp` del 13 ott scrive i giunti**: CONFERMATA (start/end joints popolati, risultato identico al run del 15 ott).
- **Kt ripristinato a 200.0 dal `finally`, anche dopo rc=3**: CONFERMATA (3/3).
- **`ensure_clean` fragile con processi esterni che citano i pattern**: CONFERMATA sperimentalmente.
- **Mount host↔container e container `geomagic_touch`**: CONFERMATI da `docker inspect`; il driver resta ignoto.
- **Ctrl-C all'orchestratore termina tutti i figli** (F6): NON VERIFICATA (F2 non eseguita; nel codice solo `launchChild` con process group proprio, demo_replay_sync_orchestrator_node.cpp:788-812).
- **Schema dei pulsanti in F2; pulsante 2 del driver per `gripper_trigger`; `prepos_capture` che si ferma sul fronte di start**: NON VERIFICATI (F2 non eseguita).
- **«Il driver pubblica già `/touch0/buttons` e una PoseStamped»** (src/haptic_dmp_learning/DESIGN_NOTES.md:12-19): NON VERIFICABILE (driver non nel repo).
- **Un `*_demo_params.yaml` con `end_joints_rad: null` è rifiutato dal planner** (docs/COMMAND_REFERENCE.md §3.8): NON VERIFICATA (non provato, non letto `selection_inputs.cpp` fino a quel controllo).
- **Un SIGKILL del runner lascia Kt = `--kt`**: NON PROVATA.
- **Criteri di successo e soglie (F5)**: letti da `tools/s1_success.yaml`; i valori numerici dei commenti (plateau 9.64 mm, ecc.) sono storici e non ricontrollati.
- **Stato dei container**: `geomagic_touch` è fermo (Exited 137): F2 richiede `docker start`/`exec` (non nel repo).

## Appendice B — Elenco opzioni e parametri

### B.1 `tools/s1_run.py --help` (riga di `add_argument` in `make_parser`)

| Opzione | Default | Riga |
|---|---|---|
| `--weights-dir` | **obbligatoria** | 1077 |
| `--kt` | 2000.0 N/m | 1078 |
| `--depth` | 0.0125 m | 1079 |
| `--follow-s` | 3.0 s | 1080 |
| `--hold-s` | 120.0 s | 1081 |
| `--squeeze` | None (parametro non passato) | 1082 |
| `--stop-cube` | False | 1084 |
| `--stop-cube-delay-s` | 0.0 s sim | 1087 |
| `--post-s` | None → max(45, follow_s + delay + 4) | 1089, 1166-1167 |
| `--spawn-yaw-deg DEG\|auto` | 180.0 | 1093 |
| `--omega-deg-s` | −2.0 | 1098 |
| `--theta-star-ref-deg` | −163.5 | 1102 |
| `--t-total-s` | 45.2 s | 1104 |
| `--center-xyz X,Y,Z` | 0.70,0.0,0.77 | 1106 |
| `--axis-tilt-deg ALPHA` | 0.0 (0 ≤ α ≤ 20) | 1109 |
| `--axis-lean-deg PHI` | 0.0 ([0,360)) | 1114 |
| `--arm {d_only,legacy,v2}` | None | 1117 |
| `--gui/--no-gui` | False | 1120 |
| `--keep-alive/--no-keep-alive` | False | 1122 |
| `--outdir` | `<repo>/s1_runs/<tag>` | 1126, 1143 |
| `--tag` | `s1_<YYYYmmddTHHMMSS>` | 1127, 1142 |
| `--extra-exec-param NOME:=VALORE` (ripetibile) | [] | 1128 |
| `--planner-param CHIAVE=VALORE` (ripetibile) | [] | 1130 |
| `--sim-timeout` | 500.0 s sim | 1134 |
| `--overwrite` | False | 1136 |

Costanti del runner: `KT_DEFAULT` 200.0 (59), `SAT_CENTER` (60), `OMEGA_DEG_S_DEFAULT` −2.0 (61), `START_JOINTS` (74), `RIM_X_C_MM` 155.1 e `GATE_TOL_MM` 0.5 (75-76), timeouts (77-91), `FRANKA_START_POSE=ready_q6pi`, `FRANKA_FINGERS=prismatic` (869-870), `min_delay_s` 5.0 e `s1.posthoc_velocity_check` false al planner (744-745), parametri fissi dell'executor (703-720: `grasp_goal_source=grasp_command`, `grasp_orientation_mode=planner_goal`, `hold_target_enabled=true`, `axis_deviation_fatal_after_trigger=false`, `satellite_rotation_enabled=true`, `satellite_rotation_mode=continuous`, `satellite_phase_source=measured`, `target_odom_required=false`, `robot_base_world=[0,0,0]`).

### B.2 `tools/s1_campaign.py`, `tools/s1_report.py`, `tools/mp_tool/mp_tool.py fit`

- `s1_campaign.py`: `--weights-dir`, `--outroot` (obbligatori), `--grid`, `--depth --kt --spawn-yaw --follow-s --hold-s --squeeze --stop-cube --omega-deg-s --center-xyz --arm` (liste, `nargs="+"`; default in `DEFAULTS`, s1_campaign.py:41-43), `--sim-timeout` 500.0 (:194), `--gui`, `--extra-exec-param`, `--planner-param` (ripetibile), `--axis-tilt-deg`, `--axis-lean-deg` (ultimi tre inoltrati a `s1_run`; i primi due solo se indicati), `--overwrite`, `--skip-done`, `--dry-run`.
- `s1_report.py`: `bag_dir` (pos.), `--executor-log`, `--meta`, `--criteria` (`tools/s1_success.yaml`), `--outdir`, `--csv`, `--urdf` (`fer_flat_effort.urdf`), `--planner-log`, `--tag`, `--min-window-s` (s1_report.py:953-963).
- `mp_tool.py fit`: vedi F3 (mp_tool.py:1008-1035).

### B.3 Parametri ROS — `grasp_planner_node` (satellite_grasp_planner/src/ros/grasp_planner_node.cpp)

Obbligatori (nessun default, il nodo fallisce): `demo_params_path` (84), `min_delay_s` (97), `cube_geometry.center_world` (99), `cube_geometry.axis_world` (100), `cube_geometry.cube_side_m` (101), `cube_geometry.standoff_m` (102), `cube_geometry.collar_radius_m` (103), `cube_geometry.face_normal_body` (105), `cube_geometry.wall_thickness_m` (106), `robot_base_world` (108), `satellite_rotation_center` (109), `satellite_rotation_axis` (110), `satellite_rotation_angular_velocity_deg_s` (111), `satellite_q_ref` (113), `satellite_odom_topic` (114), `omega_consistency_tol_deg_per_s` (115).
Opzionali: `urdf_path` ($HOME/thesis_ws/fer_flat_effort.urdf, 89-93), `q_ref_consistency_tol_deg` 0.5 (119), `robot_base_world_tol_m` 1e-6 (120), `grasp_command_topic` /grasp_command (121), `cube_geometry.grasp_radius_m` (= collar_radius_m se assente, 130), `cube_geometry.grasp_depth_m` 0.0 (132), `start_joints_override_rad` (vuoto = readyPose, 137-140).
`s1.*` (righe 147-206; default da grasp_s3_selection.hpp:38-106): `length_scale_m` 0.10, `orientation_weight` 0.0, `angle_scale_rad` 1.0, `epsilon_wait_per_s` 1e-4, `w_hat_min` 0.5, `margin_min_rad` 0.3, `r_trust_m` 0.30, `delta_v_max_m_s` 3e-3, `theta_step_deg` 2.0, `posthoc_path_check` true, `posthoc_velocity_check` true (il runner: false), `posthoc_cluster_deg` 10.0, `max_posthoc_candidates` 10, `ik.pos_tol_m` 1e-4, `ik.ori_tol_rad` 1e-3, `ik.max_iter` 300, `ik.damping` 0.05, `ik.nullspace_gain` 1.0, `ik.max_step_rad` 0.2, `force` false, `force_k` 0, `force_theta_deg` 0.0, `force_psi_deg` 0.0, `force_ignore_constraints` false, `force_flip` −1, `branch_rule` "margin", `mode` "legacy", `max_start_goal_deg` 180.0, `finalists_n` 12, `rollout_q_ref` (YAML riga 94: posa ready), `use_w` true, `kappa_w` 7.5, `kappa_t` 0.0, `tau_j_mm` 2.0, `use_clearance` false, `min_clearance_mm` 2.0, `clearance_dt_s` 0.05, `gripper_open_m` 0.060, `use_margin` false, `min_margin_deg` 24.0, `margin_follow_s` 3.0, `margin_follow_dt_s` 0.01. (I default di `ik.*` e `length_scale…` sono del codice; i valori di `ik.*` qui sopra vengono dal YAML di esempio, grasp_planner_example.yaml:72-77.)

### B.4 `prodmp_gazebo_executor_node` (haptic_dmp_learning/src/ros/prodmp_gazebo_executor_node.cpp)

`weights_yaml_path` (default `$HOME/thesis_ws/...`, 45-47), `orientation_weights_yaml_path` "" (52), `target_pose_topic` /target_pose (53), `target_twist_topic` /target_twist (54), `frame_id` panda_link0 (55), `control_rate_hz` 200.0 (56), `startup_delay_sec` 1.0 (57), `target_odom_topic` (62), `target_odom_timeout_sec` 5.0 (66), `target_odom_required` true (71), `world_frame` world (76), `ee_frame` fer_hand_tcp (77), `controller_param_sync_timeout_sec` 5.0 (88), `satellite_rotation_enabled` false (94), `satellite_rotation_mode` "frozen" (96), `satellite_rotation_axis` [0,0,1] (98), `satellite_rotation_center` [0,0,0] (100), `satellite_rotation_angular_velocity_deg_s` 2.0 (115), `satellite_rotation_frozen_phase_deg` 0.0 (117), `grasp_goal_source` "parameters" (120), `gripper_open_position` 0.06 (129), `gripper_closed_position` 0.0 (130), `gripper_close_ramp_duration_sec` 2.0 (132), `gripper_squeeze_position` (= closed, 139), `post_grasp_hold_s` 120.0 (140), `post_grasp_follow_s` 3.0 (142), `post_grasp_follow` (= gripper_control, 144), `demo_csv_path` "" (241), `axis_deviation_fatal_after_trigger` true (978), `grasp_command_topic` /grasp_command (986), `grasp_command_timeout_s` 300.0 (988), `grasp_command_max_age_s` 0.0 (993), `omega_consistency_tol_deg_per_s` 0.1 (997), `grasp_command_phase_tol_deg` 1.0 (1002), `hold_target_enabled` true (1008), `hold_target_rate_hz` 10.0 (1009), `robot_base_world_tol_m` 1e-6 (1012), `contact_time_tolerance_s` 0.1, `allow_contact_time_mismatch` false, `phase_consistency_window_s` 5.0, `phase_consistency_tol_deg` 1.0, `trigger_timeout_margin_s` 30.0, `rollout_time_tol_s` 0.05 (970-975). Dichiarati senza default (obbligatori in continuous, 934-1000): `satellite_phase_source`, `satellite_odom_topic`, `satellite_q_ref`, `satellite_intercept_phase_deg`, `satellite_contact_time_s`, `satellite_model_phase0_deg`, `robot_base_world`, `grasp_orientation_mode` (documentati in src/haptic_dmp_learning/config/params.yaml:110-143).

### B.5 `demo_replay_sync_orchestrator_node` (demo_replay_sync_orchestrator_node.cpp)

`mode` (obbligatorio: demo|replay, 59), `run_id` "" (77), `clock_wait_timeout_sec` 10.0 (91), `odom_wait_timeout_sec` 30.0 (92), `sync_delay_sec` 5.0 (93), `target_name` free_target_object (94), `target_x/y/z` 0.6/0.0/0.5 (95-97), `target_vx/vy/vz` 0.0 (98-100), `target_wx/wy/wz` 0.0 (101-103), `target_size_x/y/z` 0.150 (106-108), `target_mass` 4.412 (109), `target_yaw` 0.0 (110), `target_with_collar` true (111), `target_collar_collision` true (112), `demo_csv_path` "" (114), `buttons_topic` /touch0/buttons (115), `buttons_synced_topic` /touch0/buttons_synced (116), `button_scheme` single_button (120), `button_index` 0 (132), `debounce_s` 0.30 (137), `use_csv_playback` true (144), `anchor_enabled` false (147), `anchor_position_m` / `anchor_orientation_xyzw` (148-150), `gripper_open_position` 0.06 (169), `gripper_closed_position` 0.0 (170), `gripper_close_ramp_duration_sec` 2.0 (172), `hard_force_limit_n` −1.0 (obbligatorio > 0 in replay, 175-178), `target_odom_required` false (183), `target_odom_topic` (184), `target_odom_timeout_sec` 5.0 (186), `mp_formulation` "dmp" (195).

### B.6 `live_demo_recorder_node` (src/ros/live_demo_recorder_node.cpp)

`n_basis`, `alpha_x`, `alpha_z`, `beta_z` (obbligatori, 47-50; params.yaml: 200, 4.6, 25.0, 6.25), `master_pose_topic` /master_pose_raw (62), `target_pose_topic` /target_pose (63), `buttons_topic` /touch0/buttons (64), `joint_states_topic` /joint_states (65), `output_yaml_path` (71; src/haptic_dmp_learning/config/params.yaml:33), `output_demo_csv_path` (72; src/haptic_dmp_learning/config/params.yaml:34), `output_joint_states_csv_path` (derivato, 75-77), `record_target_odom` true (81), `target_odom_topic` /free_target_object/odometry (82), `anchor_enabled` false (88), `anchor_position_m`/`anchor_orientation_xyzw` (89-91), `feature_flags_path` (122, `dmp_features.yaml`).

### B.7 Altri

`csv_master_pose_player_node` (src/haptic_dmp_learning/config/params.yaml:11-22): `demo_csv_path`, `master_pose_topic`, `buttons_topic`, `frame_id`, `publish_rate_hz` 200.0 (stand-in, intenzionale), `startup_delay_sec` 1.0. `haptic_dmp_wrapper_node` (src/haptic_dmp_learning/config/params.yaml:1-9): `n_basis`, `alpha_x`, `alpha_z`, `beta_z`, `master_pose_topic`, `output_yaml_path`, `output_demo_csv_path`. `free_target_object.launch.py` (righe 81-120): `name, world, x, y, z, roll, pitch, yaw, mass, size_x/y/z (0.150), ixx…izz, ixy…, satellite_color, show_rotation_marker, use_mesh, wx/wy/wz (BODY), vx/vy/vz (WORLD), with_collar, collar_collision, collar_segments (24), collar_radius_m (0.0725), wall_thickness_m (0.009), standoff_m (0.08)`.
