# 05_vel_filt_test

Benchmark e validazione del filtro di velocità/accelerazione a due stadi (Two-Stage Moving Average Filter).

## Descrizione del Tool
Valuta la bontà della stima numerica delle derivate prima e seconda (velocità e accelerazione cartesiana; $\eta$ ed $\dot{\eta}$ per l'orientamento con log-map) rispetto al ground-truth analitico.
Dimostra come l'applicazione di un filtro a media mobile prima di ogni stadio di differenziazione abbatta le oscillazioni ad alta frequenza prodotte dalle differenze centrali su segnali rumorosi.

## Struttura della Cartella
- `scripts/evaluate_moving_average_filter.py`: Calcola l'RMSE delle derivate per diverse combinazioni di finestre di filtro rispetto al ground-truth analitico.
- `scripts/plot_filter_comparison.py`: Genera i grafici comparativi (scala ampia per il confronto del rumore grezzo vs filtrato, e zoom dettagliato per la fedeltà del segnale filtrato vs ground-truth).
- `plots/`: Destinazione per i grafici generati.

## Come Usarlo

1. **Generare la traiettoria di test e il ground-truth (tramite lo script generico in common)**:
```bash
python3 common/scripts/generate_picking_trajectory.py --output data/demo_noisy.csv --pos-noise-std 0.0005 --orient-noise-std-deg 0.1 --noise-seed 42
python3 common/scripts/generate_picking_trajectory.py --output data/demo_truth.csv
```

2. **Valutare le finestre di filtro a media mobile**:
```bash
python3 05_vel_filt_test/scripts/evaluate_moving_average_filter.py \
    --demo data/demo_noisy.csv \
    --truth data/demo_truth.csv \
    --window-sec-1 0.05 0.1 0.2 \
    --window-sec-2 0.05 0.1 0.2 \
    --output-csv 05_vel_filt_test/plots/ma_eval_results.csv
```

3. **Generare i grafici di confronto**:
```bash
python3 05_vel_filt_test/scripts/plot_filter_comparison.py \
    --demo data/demo_noisy.csv \
    --truth data/demo_truth.csv \
    --window-sec-1 0.05 --window-sec-2 0.05 \
    --out-dir 05_vel_filt_test/plots
```
