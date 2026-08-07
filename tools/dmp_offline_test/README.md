# DMP Offline Testing Suite

Suite completa per l'analisi offline, benchmark e validazione dei modelli DMP (Dynamic Movement Primitives) e QuaternionDMP del pacchetto `haptic_dmp_learning`.

## Struttura della Suite

La suite è suddivisa per esperimenti e strumenti comuni per garantire la massima pulizia e riproducibilità:

```
tools/dmp_offline_test/
├── README.md                          # Questo file (guida generale)
├── common/                            # Moduli condivisi tra i vari esperimenti
│   ├── include/                       # Header C++ condivisi (metrics.hpp)
│   ├── src/                           # Sorgenti C++ condivisi (metrics.cpp, learn_and_test_dmp.cpp)
│   └── scripts/                       # Script Python generici di supporto (trajectory generation, plotting, aggregazione)
├── 01_clean_trajectories/             # Test base su traiettorie sintetiche pulite
├── 02_real_data/                      # Analisi e replay su registrazioni reali haptic (Traj A, B, C)
├── 03_time_sweep/                     # Sweep della durata temporale e generalizzazione temporale
├── 04_basis_sweep/                    # Sweep sulle funzioni di base (n_basis) e finestra del filtro di velocità
├── 05_vel_filt_test/                  # Evaluation & benchmark del filtro di velocità a due stadi
└── 06_goal_generalization/            # Test di generalizzazione del goal e benchmark dei guardrail
```

## Struttura di ciascun Esperimento

Ogni cartella dell'esperimento (`01_clean_trajectories` ... `06_goal_generalization`) contiene:
- `scripts/`: Contiene tutti gli script Bash, Python e codice C++ specifici per l'esperimento.
- `plots/`: Cartella di destinazione per i grafici generati.
- `configs/`: (Se applicabile) File di configurazione YAML (es. parametri Ridge / LWR e filtri).
- `README.md`: Documentazione dettagliata dello specifico strumento/esperimento, descrizione dei test e istruzioni per l'esecuzione.

## Comandi Veloci

Tutti gli script possono essere eseguiti sia dalla root del repository che all'interno della cartella dell'esperimento:

- **01 Clean Trajectories**:
  `./01_clean_trajectories/scripts/build_and_run.sh`
- **02 Real Data**:
  `./02_real_data/scripts/run_real_trajectories_sweep.sh`
- **03 Time Sweep**:
  `./03_time_sweep/scripts/build_and_run_timesweep.sh 20 60`
- **04 Basis & Filter Window Sweep**:
  `./04_basis_sweep/scripts/run_filter_window_sweep.sh`
- **05 Velocity Filter Test**:
  `python3 05_vel_filt_test/scripts/evaluate_moving_average_filter.py --demo ...`
- **06 Goal Generalization**:
  `./06_goal_generalization/scripts/run_new_goals.sh 100`
