# Command Reference — thesis_ws

Riferimento operativo organizzato per **scenario d'uso**. Ogni comando è tratto
da uno script, launch file o DESIGN_NOTES del workspace; dove la sintassi esatta
non è ricavabile dal codice è marcato **`[DA VERIFICARE]`**.

## Convenzioni

- I comandi `ros2`, `colcon`, `ign` girano **dentro il container `franka`**, salvo
  quelli marcati **`(host)`**.
- Segnaposto da sostituire: `<run_id>`, `<topic>`, `<path>`, `<IMAGE>`, ecc. —
  ogni sezione dice dove trovarne il valore.
- Percorsi container: `THESIS_WS = /root/thesis_ws`,
  `FRANKA_WS = /root/ros_workspaces/ros2/franka_ws`. Sull'host `THESIS_WS` è
  montato come `~/thesis_ws` (stesso `.git`).
- Preambolo standard per qualunque comando ROS in una shell nuova:

```bash
source /opt/ros/humble/setup.bash
source /root/ros_workspaces/ros2/franka_ws/install/setup.bash   # bringup Franka
source /root/thesis_ws/install/setup.bash                        # nodi di tesi (se buildati lì)
```

---

## 1. Setup e provisioning del container

### 1.1 Entrare nel container (host)

```bash
bash ~/thesis_ws/scripts/run_with_thesis_mount.sh <IMAGE>
```

- Wrapper attorno a `docker run` che monta `~/thesis_ws` → `/root/thesis_ws`.
- `<IMAGE>` = nome di un'immagine `docker_factory`. **`[DA VERIFICARE]`** — il nome
  non è nel repo; l'elenco è in `$DOCKER_FACTORY_DIR/images/` (default
  `~/utils/docker_factory/images/`, override con `DOCKER_FACTORY_DIR=`). Serve il
  file `images/<IMAGE>/docker_run.cfg`.
- Variabili: `THESIS_WS` (default `$HOME/thesis_ws`).

### 1.2 Provisioning completo (dopo ricreazione container o macchina nuova)

```bash
bash /root/thesis_ws/scripts/full_reset_franka_container.sh
```

Unico entry point. Esegue 6 step:

| Step | Azione |
|---|---|
| 1/6 | Copia gli override (`franka_gazebo_controllers.yaml` + 3 launch `gazebo_*`) da `src/franka_gazebo_overrides/` in `$FRANKA_WS/.../franka_gazebo_bringup/{config,launch}/` |
| 2/6 | Ricrea i symlink `$FRANKA_WS/src/{franka_cartesian_control,haptic_dmp_learning}` → `thesis_ws` |
| 3/6 | Verifica che `controllers_plugin.xml` contenga `CartesianImpedanceController` (fail se no) |
| 4/6 | `colcon build --packages-select franka_cartesian_control franka_gazebo_bringup haptic_dmp_learning --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` in `$FRANKA_WS` |
| 5/6 | Verifica che il plugin sia nell'indice `install/` (fail se no) |
| 6/6 | `ros2 pkg prefix franka_cartesian_control` / `haptic_dmp_learning` |

Varianti:

```bash
# Perimetro ridotto: salta symlink + build di haptic_dmp_learning
bash /root/thesis_ws/scripts/full_reset_franka_container.sh --no-haptic

# Aiuto
bash /root/thesis_ws/scripts/full_reset_franka_container.sh --help
```

- Variabili d'ambiente override: `THESIS_WS`, `FRANKA_WS`, `OVERRIDES`
  (default `$THESIS_WS/src/franka_gazebo_overrides`), `ROS_DISTRO` (default `humble`).
- **Verifica successo**: lo script stampa `OK:` a ogni step e chiude con
  `=== FATTO. Ambiente pronto. ===`. Controllo manuale:

```bash
ros2 pkg prefix franka_cartesian_control && ros2 pkg prefix haptic_dmp_learning
grep -q CartesianImpedanceController \
  "$FRANKA_WS/install/franka_cartesian_control/share/franka_cartesian_control/controllers_plugin.xml" \
  && echo "plugin OK"
```

### 1.3 Override della descrizione robot / gripper prismatico

Da rieseguire dopo ricreazione container **o** dopo un update upstream di
`franka_ros2/franka_description`, **solo se serve il gripper**:

```bash
bash /root/thesis_ws/scripts/setup_franka_description_overrides.sh
```

- Copia `src/franka_description_overrides/{franka_hand.xacro, franka_arm.ros2_control.xacro}`
  in `franka_description` e fa `colcon build --packages-select franka_description --symlink-install`.
- Rende `fer_finger_joint1`/`joint2` giunti **prismatic** con `<mimic>` risolto in
  software da `gz_ros2_control`.
- **Promemoria dello script**: senza `load_gripper:=true` al bringup la mano non
  viene caricata affatto, indipendentemente da questo override.

### 1.4 Sequenza "da zero"

```bash
# (host) 1. avvia il container
bash ~/thesis_ws/scripts/run_with_thesis_mount.sh <IMAGE>

# (container) 2. provisioning
bash /root/thesis_ws/scripts/full_reset_franka_container.sh          # o --no-haptic

# (container) 3. solo se userai il gripper
bash /root/thesis_ws/scripts/setup_franka_description_overrides.sh

# (container) 4. build dei nodi di tesi nel workspace di sviluppo (se non già fatto dallo step 2)
source /opt/ros/humble/setup.bash
cd /root/thesis_ws
colcon build --packages-select haptic_dmp_learning franka_cartesian_control --symlink-install
source install/setup.bash
```

---

## 2. Bringup della simulazione

Tutti i launch vivono in `franka_gazebo_bringup` **dopo** lo step 1/6 del setup.
`arm_id` default `fer` per tutti.

### 2.1 Impedance controller, headless (default per demo / replay / sweep)

```bash
ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control_headless.launch.py
```

Argomenti (tutti opzionali):

| Arg | Default | Note |
|---|---|---|
| `arm_id` | `fer` | `fr3`, `fp3`, `fer` |
| `load_gripper` | `false` | `true` carica la mano (serve §1.3) |
| `franka_hand` | `franka_hand` | ee_id |

Wrapper che include `gazebo_cartesian_impedance_control.launch.py` con
`headless:=true`. **Usa questo** per orchestrator, sweep, tutto ciò che non
richiede la GUI.

### 2.2 Impedance controller, con GUI RViz

```bash
ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control.launch.py
```

Argomenti: `load_gripper` (`false`), `franka_hand` (`franka_hand`), `arm_id`
(`fer`), **`headless`** (`false`). Stesso stack del §2.1 **più** RViz2 con
`visualize_franka.rviz`. Usa quando vuoi vedere il robot.

### 2.3 Impedance + gripper

```bash
ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control_headless.launch.py load_gripper:=true
```

Richiede §1.3 già eseguito. Aggiunge il controllo dito via plugin nativo
`ignition::gazebo::systems::JointPositionController` sul topic `gripper_position_cmd`
+ `gripper_position_bridge` ROS→Ignition.

### 2.4 Velocity controller (resolved-rate DLS, "stadio 1")

```bash
ros2 launch franka_gazebo_bringup gazebo_velocity_cartesian_control.launch.py
```

Argomenti: `load_gripper`, `franka_hand`, `arm_id`, `headless` (tutti come sopra).

**Quando NON usarlo**: questo launch **non** avvia il `clock_bridge` né il
`gripper_position_bridge` e non ha wrapper headless. I flussi che dipendono dal
sim-time (`demo_replay_sync_orchestrator`, conversione sim-time di
`dmp_gazebo_executor_node` / `csv_master_pose_player_node`) **non funzionano** con
questo bringup. Usalo solo per test isolati del controller di velocità; per tutto
il resto usa l'impedance.

### 2.5 Cosa parte con il bringup impedance

`gz_sim` (`empty.sdf -r`) → spawn robot da `/robot_description` → `clock_bridge`
(gz `/clock` → ROS) → `gripper_position_bridge` (ROS `/gripper_position_cmd` → gz) →
`robot_state_publisher` (+ RViz se non headless) → event handler: allo spawn
`joint_state_broadcaster`, poi `cartesian_impedance_controller`.

### 2.6 Target rigido libero in Gazebo (per il ciclo grasp / VIM)

Da lanciare **in una simulazione Gazebo già attiva**:

```bash
ros2 launch free_target_object free_target_object.launch.py x:=0.6 y:=0.0 z:=0.5
```

Argomenti principali (default tra parentesi): `name` (`free_target_object`),
`world` (`""` → risolto da `/gazebo/worlds`), `x y z` (`0.0`),
`roll pitch yaw` (`0.0`), `mass` (`5.0`), `size_x size_y size_z` (`0.2`),
`ixx iyy izz` (`0.0` → formula box omogeneo), `vx vy vz` (`0.0`),
`wx wy wz` (`0.0`). Con qualunque componente di twist ≠ 0 la SDF aggiunge un
plugin `VelocityControl` che **tiene** il twist costante. Pubblica
`/<name>/odometry` (`nav_msgs/Odometry`, sim-time) via `parameter_bridge`.

> Nota: l'orchestratore (§3) spawna il target da sé con `size 0.045` e `mass 5.0` —
> non lanciare `free_target_object.launch.py` a mano se usi l'orchestrator.

---

## 3. Ciclo demo → replay

Due percorsi: **orchestrato** (consigliato, sincronizzato con un target in
movimento, `run_id` unico) e **manuale semplice** (`live_demo.launch.py`, nessun
target, nessuna sincronizzazione).

### 3.0 Dove finiscono i file

| File | Percorso | Prodotto da |
|---|---|---|
| CSV demo grezzo (orchestrato) | `/root/thesis_ws/demo_raw_<run_id>.csv` | `live_demo_recorder_node` via `demo_replay_sync.launch.py` |
| Pesi DMP (orchestrato) | `/root/thesis_ws/dmp_weights_<run_id>.yaml` | idem |
| CSV demo grezzo (manuale) | `/root/thesis_ws/live_demo_raw.csv` | `config/params.yaml` → `live_demo_recorder_node.output_demo_csv_path` |
| Pesi DMP (manuale) | `/root/thesis_ws/live_demo_dmp_weights.yaml` | `config/params.yaml` → `output_yaml_path` |
| Calibrazione forza | `/root/thesis_ws/calibrations/force_calibration_<run_id>.yaml` | `grasp_force_calibration_node` **mode `calibrate`** — **passo manuale**, vedi §3.5 |

Header CSV demo: `t,x,y,z,qw,qx,qy,qz,gripper_trigger` (i file a 8 colonne senza
`gripper_trigger` restano leggibili).

### 3.1 Prerequisito comune

Bringup impedance headless già attivo (§2.1), quindi `/clock` disponibile:

```bash
ros2 topic info /clock --verbose      # Publisher count deve essere > 0
```

### 3.2 Demo orchestrata (stand-in CSV del Geomagic)

```bash
ros2 run haptic_dmp_learning demo_replay_sync_orchestrator_node --ros-args -p mode:=demo
```

- `run_id` viene **generato** (formato `%Y%m%dT%H%M%S`) e stampato all'avvio;
  per forzarlo: `-p run_id:=<run_id>`.
- `use_csv_playback:=true` è il default → parte anche `csv_master_pose_player_node`
  che rilegge `config/params.yaml → csv_master_pose_player_node.demo_csv_path`
  (default `/root/thesis_ws/real_demo/reach_task_baseline.csv`) e **genera da sé**
  i fronti start/stop su `/touch0/buttons`. Non devi premere nulla per start/stop.
- L'orchestratore: attende `/clock` → spawna il target `free_target_object`
  (`0.045³ m`, `5 kg`, a `(0.6, 0, 0.5)`) → ancora la finestra di sync alla prima
  odometria del target → dopo `sync_delay_sec` (default 5.0 s sim-time) emette il
  fronte start su `/touch0/buttons_synced` → il recorder registra → a fine CSV
  emette lo stop, addestra e salva `demo_raw_<run_id>.csv` + `dmp_weights_<run_id>.yaml`.
- **Trigger gripper**: durante la fase RUNNING premi **`SPACE`** nel terminale
  dell'orchestratore → invia `0.0` su `/gripper_position_cmd` (chiusura) e marca il
  campione con `gripper_trigger=1`. Una sola volta per run.
- Chiudi con **`Ctrl-C`**: l'orchestratore termina tutti i processi figli.

Parametri utili dell'orchestratore (`-p nome:=valore`, default tra parentesi):

```
clock_wait_timeout_sec (10.0)   odom_wait_timeout_sec (30.0)   sync_delay_sec (5.0)
target_name (free_target_object)
target_x (0.6)  target_y (0.0)  target_z (0.5)
target_vx/vy/vz (0.0)   target_wx/wy/wz (0.0)
target_size_x/y/z (0.045)   target_mass (5.0)
buttons_topic (/touch0/buttons)   buttons_synced_topic (/touch0/buttons_synced)
use_csv_playback (true)
```

### 3.3 Replay orchestrato (stesso `run_id`)

```bash
ros2 run haptic_dmp_learning demo_replay_sync_orchestrator_node --ros-args \
  -p mode:=replay \
  -p run_id:=<run_id> \
  -p hard_force_limit_n:=<N>
```

- `<run_id>` = quello stampato/scelto in §3.2 (identifica
  `dmp_weights_<run_id>.yaml` e `demo_raw_<run_id>.csv` in `/root/thesis_ws/`).
- `hard_force_limit_n` (`> 0`, Newton) è **obbligatorio** in replay: limite `|F|`
  assoluto di sicurezza, inoltrato a `grasp_state_machine_node`. Nessun default.
- L'orchestratore, raggiunto lo stesso istante di sync, lancia come figli:

```
ros2 run haptic_dmp_learning dmp_gazebo_executor_node --ros-args \
  -p use_sim_time:=true -p startup_delay_sec:=0.0 \
  -p weights_yaml_path:=/root/thesis_ws/dmp_weights_<run_id>.yaml \
  -p demo_csv_path:=/root/thesis_ws/demo_raw_<run_id>.csv
ros2 run grasp_monitoring geometric_grasp_monitor --ros-args -p use_sim_time:=true
ros2 run haptic_dmp_learning grasp_force_calibration_node --ros-args \
  -p use_sim_time:=true -p mode:=verify -p run_id:=<run_id>
ros2 run haptic_dmp_learning grasp_state_machine_node --ros-args \
  -p use_sim_time:=true -p hard_force_limit_n:=<N>
```

- Lo stato aggregato del grasp esce su `/grasp_state_machine/grasp_state`
  (`std_msgs/String`: `free_space` → `contact_pending` → `contact_confirmed` →
  `limit_action`). `limit_action` è **latched**: si esce solo con

```bash
ros2 topic pub --once /grasp_state_machine/reset_limit_action std_msgs/msg/Bool "{data: true}"
```

### 3.4 Replay senza orchestratore (solo rollout DMP)

```bash
ros2 run haptic_dmp_learning dmp_gazebo_executor_node --ros-args \
  -p weights_yaml_path:=<path_yaml> \
  -p demo_csv_path:=<path_csv> \
  -p use_sim_time:=true
```

Parametri (default): `weights_yaml_path` (`$HOME/thesis_ws/dmp_weights.yaml`),
`demo_csv_path` (`""` → nessun trigger gripper), `target_pose_topic`
(`/target_pose`), `frame_id` (`panda_link0`), `control_rate_hz` (`200.0`),
`startup_delay_sec` (`1.0`). Pubblica la posa su `/target_pose` e, se il CSV ha
una riga con `gripper_trigger=1`, un `Float64` `0.0` su `/gripper_position_cmd`
al tempo corrispondente.

### 3.5 Calibrazione forza (passo manuale, prima del replay `verify`)

`mode:=calibrate` **non** è lanciato da nessuno script/orchestrator: va fatto a
mano durante un run in cui avviene un contatto reale.

```bash
ros2 run haptic_dmp_learning grasp_force_calibration_node --ros-args \
  -p use_sim_time:=true -p mode:=calibrate -p run_id:=<run_id>
```

Parametri (default): `capture_window_sec` (`0.25`), `calibration_dir`
(`$HOME/thesis_ws/calibrations`), `tolerance_ratio` (`0.3`), `tolerance_floor_n`
(`3.0`), `min_valid_force_n` (`3.0`), `geometric_confirmed_topic`
(`/geometric_grasp_monitor/geometric_grasp_confirmed`), `force_estimate_topic`
(`/cartesian_impedance_controller/contact_wrench_estimate`). Scrive
`calibrations/force_calibration_<run_id>.yaml`, che `mode:=verify` poi ricarica.

### 3.6 Demo manuale semplice (nessun target, nessuna sincronizzazione)

```bash
ros2 launch haptic_dmp_learning live_demo.launch.py
```

Argomenti: `use_csv_playback` (`true` → parte `csv_master_pose_player_node`,
`false` → solo `live_demo_recorder_node`), `params_file`
(default `share/haptic_dmp_learning/config/params.yaml`). **Non** avvia
Gazebo/controller: quelli devono già girare (§2). Output:
`live_demo_raw.csv` + `live_demo_dmp_weights.yaml` (da `params.yaml`).

Con `use_csv_playback:=true` non premi nulla (il player genera start/stop).
Con `use_csv_playback:=false` premi il pulsante fisico 0 del Geomagic per start,
1 per stop.

### 3.7 Percorso hardware reale (Geomagic Touch) — quando disponibile

Nessun launch: `haptic_dmp_wrapper_node` è pensato per `ros2 run` dentro il
container `geomagic_touch`, con `--params-file` **obbligatorio** (i parametri
`n_basis, alpha_x, alpha_z, beta_z` non hanno default e il nodo lancia se mancano):

```bash
ros2 run haptic_dmp_learning haptic_dmp_wrapper_node --ros-args \
  --params-file /root/thesis_ws/src/haptic_dmp_learning/config/params.yaml
```

Sub: `/touch0/pose` (`geometry_msgs/PoseStamped`), `/touch0/buttons`
(`sensor_msgs/Joy`). Pulsante 0 = start, 1 = stop+learn. Output default (da
`params.yaml`): `/root/thesis_ws/dmp_weights.yaml` + `demo_raw.csv`.

Per usare `live_demo.launch.py` con il driver reale invece del CSV player:
`use_csv_playback:=false` e rimappa il topic nativo del driver su
`/master_pose_raw`:

```bash
ros2 run <driver_pkg> <driver_node> --ros-args -r <driver_topic>:=/master_pose_raw
```

**`[DA VERIFICARE]`** — `<driver_pkg>`, `<driver_node>` e `<driver_topic>` non
sono nel repo; `haptic_dmp_wrapper_node` si aspetta il topic pose su `/touch0/pose`.

---

## 4. Test automatici

I test si buildano/eseguono nel workspace di sviluppo `/root/thesis_ws` (distinto
dal build in `$FRANKA_WS` fatto da `full_reset`, che non compila i target di test):

```bash
source /opt/ros/humble/setup.bash
cd /root/thesis_ws
colcon build --packages-select haptic_dmp_learning franka_cartesian_control --symlink-install
```

### 4.1 Tutte le suite di un pacchetto

```bash
colcon test --packages-select haptic_dmp_learning --event-handlers console_direct+
colcon test-result --verbose        # riepilogo dopo l'esecuzione
```

### 4.2 Una singola suite gtest

```bash
colcon test --packages-select haptic_dmp_learning --ctest-args -R test_grasp_state_machine \
  --event-handlers console_direct+
```

Oppure eseguendo direttamente il binario (nessun `colcon`, output immediato):

```bash
/root/thesis_ws/build/haptic_dmp_learning/test_grasp_state_machine
/root/thesis_ws/build/franka_cartesian_control/test_velocity_ik
```

### 4.3 Cosa copre ciascuna suite

| Suite | Pacchetto | Copertura |
|---|---|---|
| `test_dmp` | haptic_dmp_learning | DMP traslazionale: fit da demo sintetica, rollout che converge al goal, filtro media mobile |
| `test_quaternion_dmp` | haptic_dmp_learning | QuaternionDMP: fit + rollout che converge all'orientamento goal (`angularDistance` sotto soglia) |
| `test_dmp_io` | haptic_dmp_learning | Roundtrip YAML dei pesi DMP + QuaternionDMP (tau, y0, goal, centri, pesi preservati) |
| `test_demo_csv_io` | haptic_dmp_learning | `core::demo_csv_io`: roundtrip write/read CSV demo incl. `gripper_trigger`, lettura file legacy 8 colonne, error path (file assente / header-only / riga corta), `readGripperTriggerTime` (trovato / colonna assente / nessun trigger / CRLF) |
| `test_grasp_state_machine` | haptic_dmp_learning | `core::GraspStateMachine`: gate di persistenza, gate `force_verified`, trip su `|F| > limit` (`>` stretto), latch di `limit_action` contro calo forza e perdita segnale, `requestReset()` unica uscita, validazione costruttore |
| `test_cartesian_error` | franka_cartesian_control | `computePoseError` / `logMap`: errore lineare + angolare SO(3), shortest-path sull'emisfero |
| `test_velocity_ik` | franka_cartesian_control | `VelocityIkSolver`: twist proporzionale + saturazioni, inversione DLS, clamp velocità di giunto |

> Le suite linter (`copyright`, `cpplint`, `uncrustify`, `flake8`) di
> `haptic_dmp_learning` **falliscono da prima** (debito a livello di pacchetto:
> nessun file ha header di copyright, 37 file divergono da uncrustify). Non sono
> una regressione; ignorale finché non si decide di sistemarle in blocco.

---

## 5. Sweep e valutazione dei guadagni del controller

Script in `tools/gazebo_cartesian_eval/scripts/`. Ogni sweep **rilancia Gazebo da
zero per ogni punto**, quindi va lasciato girare a lungo. Tutti hard-codano
`WEIGHTS_YAML=/root/thesis_ws/real_trajA_ridge_filter.yaml` e `ROLLOUT_DURATION=70`.
**`[DA VERIFICARE]`** — `real_trajA_ridge_filter.yaml` deve esistere in
`/root/thesis_ws/` (non versionato); rigeneralo da una demo se manca.

### 5.1 Quale sweep per cosa

| Script | Cosa varia | Fisso | Output |
|---|---|---|---|
| `run_translation_gain_sweep.sh` | `translational_stiffness` ∈ {100,200,400,800,1400,2000} × `translational_damping` ∈ {10,20,30,50,70,89} | rot 20/3 | `data/translation_sweep_summary.csv` |
| `run_rotation_gain_sweep.sh` | `rotational_stiffness` ∈ {10,20,40,70,110,150} × `rotational_damping` ∈ {1,2,3,4,5,7} | trans 200/30 | `data/rotation_sweep_summary.csv` |
| `run_nullspace_stiffness_sweep.sh` | `nullspace_stiffness` ∈ {0,0.2,1,2,3,5,8,12,16,20,30,50}, 2 ripetizioni/punto | `joint1_nullspace_stiffness=100` | `results/nullspace_stiffness_sweep.csv` |
| `run_reach_task_comparison.sh` | confronto Velocity vs Impedance sulla stessa traiettoria | — | `data/` + plot 3-subplot |

Lancio (nessun argomento, tranne il comparison):

```bash
cd /root/thesis_ws/tools/gazebo_cartesian_eval/scripts

./run_translation_gain_sweep.sh
./run_rotation_gain_sweep.sh
./run_nullspace_stiffness_sweep.sh
./run_reach_task_comparison.sh [<path_weights_yaml>]   # default real_trajA_ridge_filter.yaml
```

`translation`/`nullspace` fanno `ros2 param set` a caldo o scrivono lo YAML;
`rotation`/`nullspace` **riscrivono `franka_gazebo_controllers.yaml` e rifanno
`colcon build franka_gazebo_bringup`** per ogni punto (partendo dal backup
`src/franka_gazebo_overrides/franka_gazebo_controllers.yaml`).

### 5.2 Rilanciare solo i punti falliti (righe `NA` nel CSV)

```bash
# translation / rotation: punti hard-coded nello script (MISSING=(...)), append al CSV
./run_missing_translation_points.sh
./run_missing_rotation_points.sh

# nullspace: accetta i punti come argomenti "K R" (default "5 2" "20 2" "20 3")
./run_missing_nullspace_points.sh "5 2" "20 3"
```

### 5.3 Leggere l'output

Colonne comuni dei `*_summary.csv`:
`(<gain_k>,<gain_d>,) rot_cum_deg, rot_net_deg, ratio, mean_pos_mm, max_pos_mm, mean_ang_deg, max_ang_deg`.
`ratio = rot_cum_deg / rot_net_deg` è l'indicatore di stabilità (≈1 = pulito,
≫1 = oscillazione/instabilità). `NA` = punto fallito (timeout avvio controller o
nessun `RESULT_LINE`).

Heatmap 2D del `ratio`:

```bash
cd /root/thesis_ws/tools/gazebo_cartesian_eval/scripts
python3 plot_gain_sweep_heatmap.py <csv> <col_k> <col_d>
# es.
python3 plot_gain_sweep_heatmap.py ../data/translation_sweep_summary.csv trans_stiffness trans_damping
python3 plot_gain_sweep_heatmap.py ../data/rotation_sweep_summary.csv    rot_stiffness   rot_damping
```

### 5.4 Valutazione manuale di un singolo run

```bash
cd /root/thesis_ws/tools/gazebo_cartesian_eval/scripts

# 1. registra i topic di tracking durante un rollout
./record_bag.sh <nome_run> [<controller_name>]        # default cartesian_impedance_controller

# 2. estrai il bag in CSV (target_aligned.csv + actual_pose.csv sotto data/<nome_run>/)
python3 extract_bag_to_csv.py <path_al_bag> <nome_run> [<controller_name>]

# 3a. metriche + plot interattivo
python3 evaluate_cartesian_tracking.py [<nome_run>]   # senza arg: elenca i run in data/
# 3b. versione headless (solo PNG + RESULT_LINE su stdout)
python3 evaluate_cartesian_tracking_headless.py <nome_run>
```

Diagnostica leak nullspace su un run:

```bash
python3 log_nullspace_leak.py /cartesian_impedance_controller/nullspace_leak 30.0
```

**`[DA VERIFICARE]`** — la docstring dichiara `[topic] [duration_sec]` (2 arg), ma
`run_missing_nullspace_points.sh` lo invoca con 4 arg
(`<leak_topic> <target_pose_topic> 0.5 180.0`). Controlla il parsing reale in
`log_nullspace_leak.py` prima di usarlo fuori dagli script.

---

## 6. Comandi diagnostici e di debug

### 6.1 Stato di un controller ros2_control

```bash
ros2 control list_controllers
```
- **OK**: `cartesian_impedance_controller  franka_cartesian_control/CartesianImpedanceController  active`.
- `inactive` → caricato ma non attivo: `ros2 control set_controller_state cartesian_impedance_controller active`.
- assente → non caricato: controlla il log del bringup (`Successfully loaded controller ...`) e §7.

```bash
ros2 control list_hardware_interfaces
```
- **OK (impedance)**: `fer_joint1/effort ... [available] [claimed]` per i 7 giunti.
  Con velocity controller sono `.../velocity`. Con gripper: anche
  `fer_finger_joint1/position [claimed]`.
- `[available] [unclaimed]` → nessun controller attivo rivendica quell'interfaccia.
- interfaccia `effort` assente → l'URDF non è stato reso con `gazebo_effort=true`
  (stai usando il launch di velocità?).
- `fer_finger_joint1/*` assente → override descrizione non applicato (§1.3).

```bash
ros2 control list_hardware_components
```
- **OK**: `FrankaHardwareInterface  ...  active`.

### 6.2 Ispezionare un topic

```bash
ros2 topic list
ros2 topic info <topic> --verbose        # tipo, conteggio publisher/subscriber, QoS
ros2 topic echo <topic> --once           # UN messaggio, poi esce
ros2 topic echo <topic>                  # streaming (Ctrl-C per fermare)
ros2 topic hz <topic>                    # frequenza effettiva
```
- `--once` per un check veloce del contenuto; streaming solo se ti serve vedere
  l'evoluzione nel tempo (costa CPU e sporca il terminale).
- Topic chiave: `/target_pose` (~200 Hz durante rollout),
  `/cartesian_impedance_controller/actual_pose`,
  `/cartesian_impedance_controller/target_pose_aligned`,
  `/cartesian_impedance_controller/contact_wrench_estimate` (solo se
  `enable_contact_force_estimation:=true`, com'è nel YAML),
  `/geometric_grasp_monitor/geometric_grasp_confirmed` (`Bool`),
  `/grasp_state_machine/grasp_state` (`String`).

```bash
ros2 topic echo /clock --once
ros2 topic info /clock --verbose
```
- **OK**: `sec`/`nanosec` che avanza; `Publisher count: 1`.
- `Publisher count: 0` / nessun output → `/clock` non bridgiato: hai lanciato il
  bringup di **velocità** (senza `clock_bridge`) o il bridge non è partito. Tutti i
  nodi `use_sim_time:=true` restano fermi. Vedi §7.

### 6.3 Motore fisico Ignition (bypass ROS 2)

```bash
which ign gz                              # su Fortress è "ign"; installazioni nuove: "gz"
ign topic -l                             # lista dei topic Ignition
ign topic -e -t /clock                   # echo raw del clock gz
ign topic -i -t /clock                   # info sul topic gz
ign model --list                         # modelli nel mondo
ign model -m <nome> --pose               # posa di un modello
ign model -m <nome> -j                   # giunti di un modello (free_target_object: nessuno)
```
- Comandare il gripper direttamente sul lato gz (bypassando ROS e il bridge):

```bash
ign topic -t /gripper_position_cmd -m ignition.msgs.Double -p 'data: 0.0'    # 0.0 = chiuso, 0.04 = aperto
```
Il plugin `ignition::gazebo::systems::JointPositionController` su
`fer_finger_joint1` ascolta il topic `gripper_position_cmd`
(`franka_arm.ros2_control.xacro`). **`[DA VERIFICARE]`** — con `ign topic -l`
controlla se il nome esatto è `/gripper_position_cmd` o namespaced.
- Risolvere il nome del mondo (come fa `free_target_object.launch.py`):

```bash
ign service -s /gazebo/worlds --reqtype ignition.msgs.Empty \
  --reptype ignition.msgs.StringMsg_V --timeout 5000 --req ''
```
- **Uso tipico**: se `ros2 topic echo /clock` è muto ma `ign topic -e -t /clock`
  avanza → Gazebo gira, manca solo il bridge ROS. Se anche `ign topic -e -t /clock`
  è muto → Gazebo non sta simulando (manca `-r` nel `gz_args`, o è in pausa).

### 6.4 URDF / robot_description a runtime

```bash
ros2 param get /robot_state_publisher robot_description
```
- Dump dell'URDF XML effettivamente caricato. Per un check mirato sui giunti dito:

```bash
ros2 param get /robot_state_publisher robot_description \
  | grep -oE '<joint name="fer_finger_joint[12]"[^>]*type="[a-z]+"'
```
- **OK (gripper attivo)**: `type="prismatic"`. `type="fixed"` → l'override §1.3 non
  è stato applicato o il bringup è partito senza ricaricare la descrizione.

### 6.5 Catturare e filtrare i log del bringup

```bash
stdbuf -oL -eL ros2 launch franka_gazebo_bringup \
  gazebo_cartesian_impedance_control_headless.launch.py 2>&1 | tee /tmp/bringup.log
```
Poi, in un'altra shell:

```bash
grep -nE "Successfully loaded controller|claimed|[Ee]rror|FAILED|exception" /tmp/bringup.log
grep -n "translational_stiffness\|rotational_stiffness" /tmp/bringup.log   # gain effettivamente caricati
```
Pattern usato dagli script di sweep (avvio in background + poll sul log):

```bash
ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control_headless.launch.py \
  > /tmp/launch_log.txt 2>&1 &
until grep -q "Successfully loaded controller cartesian_impedance_controller" /tmp/launch_log.txt; do sleep 1; done
```

### 6.6 Quale file di configurazione è realmente in uso (modello a container)

Il bringup **non** legge `thesis_ws/src/franka_gazebo_overrides/` direttamente:
lo step 1/6 del setup ne fa una **copia** in
`$FRANKA_WS/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/`.

```bash
# 1. la copia usata dal bringup coincide con l'override sorgente?
diff /root/ros_workspaces/ros2/franka_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml \
     /root/thesis_ws/src/franka_gazebo_overrides/franka_gazebo_controllers.yaml
#    nessun output = allineati. Differenze = la copia è stale (rilancia full_reset o ricopia a mano).

# 2. cosa punta l'install (dopo --symlink-install)
ls -l   /root/ros_workspaces/ros2/franka_ws/install/franka_gazebo_bringup/share/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml
readlink -f /root/ros_workspaces/ros2/franka_ws/install/franka_gazebo_bringup/share/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml

# 3. verità a runtime: il valore effettivamente caricato dal controller
ros2 param get /cartesian_impedance_controller translational_stiffness
ros2 param dump /cartesian_impedance_controller
```
- Il YAML nel `bringup/config/` è agganciato dall'install via symlink: **editarlo e
  rilanciare il bringup basta, senza rebuild**. Ma la modifica va persa al prossimo
  `full_reset` se non la riporti anche in `src/franka_gazebo_overrides/`.
- Per gli xacro (`franka_hand.xacro`, `franka_arm.ros2_control.xacro`) vale lo
  stesso schema: sorgente in `src/franka_description_overrides/`, copia in
  `franka_description/`, applicata da `setup_franka_description_overrides.sh`.

### 6.7 Verifica binari aggiornati dopo un edit del core

```bash
find /root/thesis_ws/src/haptic_dmp_learning/src -newer \
  /root/thesis_ws/build/haptic_dmp_learning/live_demo_recorder_node
```
- **OK**: nessun output (i binari sono più recenti dei sorgenti).
- Righe elencate → serve `colcon build --packages-select haptic_dmp_learning`
  **vero** (non basta `--symlink-install`, che aggiorna solo YAML/launch).

---

## 7. Troubleshooting rapido

| Sintomo | Prima cosa da controllare |
|---|---|
| Il gripper non si muove nonostante il comando | 1) bringup lanciato con `load_gripper:=true`? 2) `setup_franka_description_overrides.sh` eseguito? verifica `type="prismatic"` sui `fer_finger_joint` (§6.4). 3) quale YAML è caricato (§6.6). 4) prova il ramo POSITION diretto: `ign topic -l | grep gripper` e `ign topic -t /gripper_position_cmd -m ignition.msgs.Double -p 'data: 0.0'` (§6.3). |
| Nodi `use_sim_time:=true` fermi / timer che non scattano | `ros2 topic info /clock --verbose` (§6.2). Publisher count 0 → hai lanciato `gazebo_velocity_cartesian_control.launch.py` (niente `clock_bridge`): usa `gazebo_cartesian_impedance_control_headless.launch.py`. Se anche `ign topic -e -t /clock` è muto → Gazebo non simula (pausa / manca `-r`). |
| `demo_replay_sync_orchestrator` aborta subito con `FATAL ... /clock` | Come sopra: bringup impedance non attivo o `/clock` non bridgiato. Avvialo prima dell'orchestratore. |
| DMP "completamente sbagliato" (RMSE ~72 mm, ~60° orientamento) | Nodo avviato senza `--params-file` → `n_basis` è rimasto a 20 invece di 200. Passa sempre `--ros-args --params-file .../config/params.yaml`. Oppure `dmp_features.yaml` irraggiungibile → `applyFeatureConfig` lancia (fail-loud): leggi nel messaggio i path tentati. |
| `dmp_gazebo_executor_node`: rollout con `tau` incoerente / pesi stale | Stai usando un path relativo o un `dmp_weights.yaml` registrato prima della conversione sim-time. Passa `weights_yaml_path:=` **assoluto**; se i pesi sono pre-sim-time, ri-registra la demo. |
| Controller non si attiva / plugin non trovato | `ros2 control list_controllers`; grep del log `Successfully loaded controller`. Se manca il plugin: `full_reset_franka_container.sh` (step 3 e 5 verificano `controllers_plugin.xml` in `src` e in `install`). |
| Modifica ai gain YAML "ignorata" | Hai editato `src/franka_gazebo_overrides/...yaml` ma il bringup legge la **copia** in `$FRANKA_WS/.../franka_gazebo_bringup/config/`. `diff` tra i due (§6.6), ricopia o rilancia `full_reset`; conferma con `ros2 param get /cartesian_impedance_controller translational_stiffness`. |
| `geometric_grasp_monitor` non conferma mai / conferma troppo presto | Soglia `epsilon_pos` (default 0.035 m nel nodo, tarata sul cubo 0.045 m dell'orchestratore) — se cambi `target_size_*` nell'orchestratore senza aggiornare `epsilon_pos` di conseguenza, la soglia torna scollegata dalla dimensione reale del target. Verifica con `/geometric_grasp_monitor/geometric_grasp_debug` (campo `dist`). |
| Edit di `dmp_io.cpp` / costruttore nodo / `params.yaml` senza effetto | Serve `colcon build` vero (§6.7), non solo `--symlink-install`. Verifica mtime binari vs sorgenti. |
| `grasp_state_machine` resta in `limit_action` | È latched by design. Esci con `ros2 topic pub --once /grasp_state_machine/reset_limit_action std_msgs/msg/Bool "{data: true}"` (§3.3). |

---

## Appendice — file di configurazione chiave

| File | Contenuto |
|---|---|
| `src/haptic_dmp_learning/config/params.yaml` | Parametri per `haptic_dmp_wrapper_node`, `csv_master_pose_player_node`, `live_demo_recorder_node`. `n_basis: 200`, `alpha_x: 4.6`, `alpha_z: 25.0`, `beta_z: 6.25`. Path CSV del player: `/root/thesis_ws/real_demo/reach_task_baseline.csv`. |
| `src/haptic_dmp_learning/config/dmp_features.yaml` | `regression.method: ridge`, `ridge_lambda: 1.0e-6`, `second_order_canonical_system: false`, `velocity_filter.enabled: true`, `window_sec_1/2: 0.20`. |
| `src/franka_gazebo_overrides/franka_gazebo_controllers.yaml` | Sorgente di verità dei gain controller. Impedance in uso: `translational_stiffness: 200`, `translational_damping: 10`, `rotational_stiffness: 150`, `rotational_damping: 1`, `nullspace_stiffness: 0.2`, `joint1_nullspace_stiffness: 100`, `enable_nullspace_leak_diagnostics: true`, `enable_contact_force_estimation: true`. `update_rate: 1000`. |
| `src/franka_description_overrides/franka_hand.xacro` | `fer_finger_joint1/2` come `prismatic` (lower 0.0 / upper 0.04), `<mimic>` sul joint1. |
| `src/franka_description_overrides/franka_arm.ros2_control.xacro` | Blocco `<ros2_control>` gripper + plugin `JointPositionController` su topic `gripper_position_cmd`. |
