# 01_clean_trajectories

Strumento di benchmark base per verificare l'apprendimento e la ricostruzione DMP su traiettorie sintetiche pulite e deterministiche (senza rumore).

## Descrizione del Tool
Il tool genera 4 tipologie di movimento standard:
1. `reach_semplice`: Movimento lineare punto-punto.
2. `reach_lift_pitch`: Traiettoria con sollevamento e variazione d'orientamento.
3. `rotazione_pura`: Rotazione dell'end-effector attorno a un asse senza traslazione.
4. `reach_complesso_gradino`: Traiettoria complessa a più segmenti con variazioni brusche.

Per ciascun movimento viene addestrato il modello DMP + QuaternionDMP, rieseguito in-process e calcolate le metriche di fedeltà della traiettoria (RMSE di posizione, errore angolare medio e massimo, errore di endpoint).

## Struttura della Cartella
- `scripts/test_core_offline.cpp`: Codice C++ per l'esecuzione dei test su tutte le traiettorie sintetiche.
- `scripts/plot_dmp_test.py`: Script Python per visualizzare 3D, profili di posizione ed errore di orientamento.
- `scripts/build_and_run.sh`: Script di build ed esecuzione automatica.
- `plots/`: Cartella per l'output dei grafici.

## Come Usarlo

Compilazione ed esecuzione completa dei test:
```bash
./01_clean_trajectories/scripts/build_and_run.sh
```

Generazione dei grafici per una specifica traiettoria:
```bash
python3 01_clean_trajectories/scripts/plot_dmp_test.py reach_lift_pitch
```
