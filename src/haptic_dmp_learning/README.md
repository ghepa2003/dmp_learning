# haptic_dmp_learning

Pacchetto ROS2 (Humble) per apprendere una DMP traslazionale (posizione, 3 DoF)
da una dimostrazione registrata con il Geomagic Touch, isolando la matematica
della DMP (`core/`) dal protocollo di comunicazione ROS2 (`ros/`).

## Struttura

```
include/haptic_dmp_learning/
├── core/            # zero dipendenze ROS - solo Eigen (+ yaml-cpp per l'I/O)
│   ├── types.hpp             # struct Sample (t, position)
│   ├── demonstration_recorder.hpp
│   ├── dmp.hpp                # la DMP vera e propria
│   └── dmp_io.hpp             # salvataggio/caricamento YAML
└── ros/
    └── haptic_dmp_wrapper_node.hpp   # unico file che conosce topic/msg ROS2

src/
├── core/            # implementazioni corrispondenti
└── ros/
    ├── haptic_dmp_wrapper_node.cpp
    └── main.cpp
```

Se domani il protocollo cambia (nuovo device, nuovi nomi di topic, persino
un middleware diverso da ROS2), si tocca solo `ros/haptic_dmp_wrapper_node.*`.
`core/` resta invariato e restabile in isolamento.

## Installazione

Copia questa cartella dentro la tua workspace overlay, poi builda:

```bash
cp -r haptic_dmp_learning ~/thesis_ws/src/
cd /root/thesis_ws   # dentro il container
source /root/ros_workspaces/ros2/geomagic_touch_ws/install/setup.bash   # base del laboratorio (per i msg del device)
colcon build --packages-select haptic_dmp_learning
source install/setup.bash
```

## PRIMA DI USARLO: verifica l'indice dei due bottoni

Il nodo assume che `msg->buttons[0]` sia il bottone di start e `msg->buttons[1]`
quello di stop. **Verificalo** prima di fidarti:

```bash
ros2 topic echo /touch0/buttons
```

Premi un bottone alla volta e guarda quale indice del campo `buttons` passa da
0 a 1. Se l'ordine è invertito rispetto a quanto assunto, scambia gli indici
in `buttonsCallback()` dentro `haptic_dmp_wrapper_node.cpp`.

## Uso

```bash
ros2 run haptic_dmp_learning haptic_dmp_wrapper_node
```

Oppure con parametri custom:
```bash
ros2 run haptic_dmp_learning haptic_dmp_wrapper_node --ros-args --params-file config/params.yaml
```

- **Bottone 0**: avvia la registrazione della demo (svuota il buffer precedente)
- **Bottone 1**: ferma la registrazione, allena la DMP, salva i pesi in YAML
  (percorso di default: `~/thesis_ws/dmp_weights.yaml`, configurabile col
  parametro `output_yaml_path`)

Log attesi in console a ogni ciclo: `Recording started.` → (muovi il device) →
`Recording stopped. N samples collected.` → `DMP learned and saved to ...`

## Limitazioni note (volute, per questa prima versione)

- Solo posizione (DMP traslazionale) - niente Quaternion DMP per ora
- `y0`/`goal` presi rispettivamente da primo/ultimo campione di un'unica
  demo - nessuna media multi-demo
- Velocità/accelerazione stimate per differenze finite centrate sui dati
  grezzi del device - se il replay risulta rumoroso, il primo intervento
  da provare è un filtro passa-basso sulla posizione prima della stima
- Nessuna trasformazione di frame: la posa è presa così com'è da
  `/touch0/pose`, nel frame nativo del Geomagic Touch
