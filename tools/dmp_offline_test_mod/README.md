# dmp_offline_test

Repository focalizzato esclusivamente sul **replay offline e sulla generazione dei plot per la demo reale** (registrata con il dispositivo aptico Geomagic Touch).

Nessun ROS2 o Docker richiesto: gira in ambiente C++ standalone sull'host con **Eigen3** e **yaml-cpp**.

## Parametri e Pesi DMP

I parametri del modello (`n_basis`, `alpha_x`, `alpha_z`, `beta_z`) e le funzioni di base apprese vengono letti direttamente dal file YAML generato dalla sessione ROS2 (default: `/home/lorenzo/thesis_ws/dmp_weights.yaml`).

## Struttura della repository

- `core/`: Libreria C++ DMP (header e sorgenti: `dmp.hpp`, `dmp.cpp`, `dmp_io.hpp`, `dmp_io.cpp`, ecc.).
- `replay_saved_dmp.cpp`: Carica i pesi e i parametri da `dmp_weights.yaml` ed esegue il replay della DMP generando `replay_from_yaml.csv`.
- `plot_real_demo.py`: Script Python per generare i grafici comparativi 3D e le serie temporali per asse tra la demo reale registrata (`demo_raw.csv`) e il replay DMP (`replay_from_yaml.csv`).
- `replay_build_and_run.sh`: Script bash unico per compilare `replay_saved_dmp`, eseguire la simulazione offline e lanciare la generazione dei plot.

## Esecuzione

Per compilare, generare il replay e produrre il plot comparativo:

```bash
chmod +x replay_build_and_run.sh
./replay_build_and_run.sh [/percorso/dmp_weights.yaml] [/percorso/demo_raw.csv]
```

Default usati se non specificati:
- YAML pesi: `/home/lorenzo/thesis_ws/dmp_weights.yaml`
- Demo grezza: `/home/lorenzo/thesis_ws/demo_raw.csv`

L'esecuzione produrrà:
- `replay_from_yaml.csv` (traiettoria rigenerata dalla DMP)
- `real_demo_plot.png` (grafico comparativo 3D e per asse)
