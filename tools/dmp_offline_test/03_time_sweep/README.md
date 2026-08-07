# 03_time_sweep

Strumento per lo studio della dipendenza dalla durata temporale ($\tau$) e per il test di generalizzazione temporale della DMP.

## Descrizione del Tool
Verifica il comportamento del modello DMP variando la durata della traiettoria (es. da 30s a 90s) per analizzare:
1. Come cambiano l'errore di tracciamento e la stabilità con la scalatura temporale.
2. La capacità della DMP di generalizzare l'esecuzione mantenendo la forma dello spazio pur cambiando la velocità di riproduzione.

## Struttura della Cartella
- `scripts/generate_sweep_matrix.py`: Genera una matrice di traiettorie sintetiche varia durate e goal.
- `scripts/learn_and_test_dmp.cpp`: Esegue apprendimento e replay salvando metriche di fedeltà in CSV.
- `scripts/generalize_test_dmp.cpp`: Test di generalizzazione adattando la durata ed il goal senza ri-addestrare i pesi.
- `scripts/build_and_run_timesweep.sh`: Script principale per eseguire lo sweep di durata temporale.
- `scripts/run_generalization_sweep.sh`: Script per lo sweep di generalizzazione temporale.
- `scripts/plot_dmp_timesweep.py`: Plot della singola traiettoria nel time sweep.
- `scripts/plot_sweep_study.py`: Plot comparativo aggregato dello sweep temporale.
- `plots/`: Cartella per l'output dei grafici.

## Come Usarlo

1. **Generare la matrice delle traiettorie sintetiche (durate 30s - 90s)**:
```bash
python3 03_time_sweep/scripts/generate_sweep_matrix.py --outdir data
```

2. **Eseguire lo sweep temporale completo**:
```bash
./03_time_sweep/scripts/build_and_run_timesweep.sh 20 60
```

3. **Eseguire la prova di generalizzazione temporale**:
```bash
./03_time_sweep/scripts/run_generalization_sweep.sh 60 goalA_main 20
```
