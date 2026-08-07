# 02_real_data

Strumento di analisi e validazione delle DMP su dimostrazioni reali acquisite dal robot (`demo_raw_trajA.csv`, `demo_raw_trajB.csv`, `demo_raw_trajC.csv`).

## Descrizione del Tool
Valuta le prestazioni di apprendimento e ricostruzione delle DMP su dati reali caratterizzati da dinamiche complesse e rumore naturale del sensore/operatore. Confronta la regressione standard (Independent LWR) con la regolarizzazione Ridge.

## Struttura della Cartella
- `scripts/replay_saved_dmp.cpp`: Replay in-process a partire da pesi DMP già salvati in formato YAML.
- `scripts/run_real_trajectories_sweep.sh`: Suite completa di benchmark sulle 3 traiettorie reali (Traj A, B, C).
- `scripts/plot_real_demo.py`: Plot 3D e time series per una singola dimostrazione reale.
- `scripts/plot_real_trajectories_study.py`: Generazione dei grafici comparativi di sintesi per tutte le traiettorie reali.
- `plots/`: Destinazione per i grafici generati.

## Come Usarlo

1. **Eseguire lo sweep completo sulle 3 traiettorie reali**:
```bash
./02_real_data/scripts/run_real_trajectories_sweep.sh
```

2. **Eseguire il replay da un file di pesi YAML salvato**:
```bash
./02_real_data/scripts/replay_build_and_run.sh weights/real_trajA_ridge_filter.yaml
```
