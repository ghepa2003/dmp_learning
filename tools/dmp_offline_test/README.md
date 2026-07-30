# dmp_offline_test

Test offline (no ROS2, no Docker, no device fisico) per `core::DMP` e
`core::QuaternionDMP` — punta direttamente ai sorgenti ufficiali del
pacchetto (`../../src/haptic_dmp_learning`), nessuna copia duplicata.

## Struttura

```
tools/dmp_offline_test/
├── build/                          # Eseguibili compilati
├── data/                           # File CSV generati (demo sintetiche, replay, metriche)
├── weights/                        # File YAML con i pesi DMP generati
├── plots/                          # Grafici PNG generati
├── metrics.hpp / metrics.cpp       # metriche di valutazione, LOCALI a tools
├── test_core_offline.cpp            # test multi-traiettoria sintetico
├── build_and_run.sh
├── replay_saved_dmp.cpp             # carica un dmp_weights.yaml reale e rigenera il replay
├── replay_build_and_run.sh
├── plot_dmp_test.py                # grafico demo sintetica vs replay, per traiettoria
└── plot_real_demo.py               # grafico demo REALE (Geomagic) vs replay + metriche
```

## Test multi-traiettoria sintetico

```bash
chmod +x build_and_run.sh
./build_and_run.sh
```

Prova **4 traiettorie diverse** in sequenza, ciascuna con motivazione specifica:

| Traiettoria | Cosa verifica |
|---|---|
| `reach_semplice` | Caso facile, baseline — fedeltà attesa molto alta |
| `reach_lift_pitch` | Caso misto (traslazione + gobba + rotazione) — quello usato nei test precedenti |
| `rotazione_pura` | Spostamento posizionale minimo, rotazione ampia — isola la Quaternion DMP; con lo spostamento di goal previsto innesca il guardrail A |
| `reach_complesso_gradino` | Profilo quasi a gradino — riproduce (in piccolo) il limite osservato su hardware reale con traiettorie multi-segmento |

Per ciascuna, genera:
- `data/demo_original_<nome>.csv`, `data/replay_same_goal_<nome>.csv`, `data/replay_new_goal_<nome>.csv`
- `weights/dmp_weights_<nome>.yaml` (formato combinato posizione + orientamento)
- Una riga in `data/metrics_summary.csv` per `_same_goal` (RMSE, errore max) e una per `_new_goal` (solo errore finale)
- Stampa a console eventuali avvisi del guardrail A (`isScaleReliable`)

## Grafico di una traiettoria specifica

```bash
python3 plot_dmp_test.py reach_lift_pitch
```
Senza argomento, prova `reach_lift_pitch` di default e altrimenti elenca le traiettorie trovate.

## Test con demo REALE dal Geomagic Touch

Una volta registrata una demo vera (bottone 0 → muovi il device → bottone 1), il wrapper produce `dmp_weights.yaml` e `dmp_demo_recorded.csv` dentro `~/thesis_ws/`.

```bash
chmod +x replay_build_and_run.sh
./replay_build_and_run.sh ~/thesis_ws/dmp_weights.yaml
python3 plot_real_demo.py ~/thesis_ws/dmp_demo_recorded.csv
```

`plot_real_demo.py` ora stampa anche le metriche (RMSE posizione, errore angolare) direttamente in console, oltre al grafico — stesso identico calcolo di `metrics.cpp`, solo scritto in Python per evitare di dover ricompilare per un confronto singolo.

## Nota su `metrics.hpp/.cpp`

Vivono **solo qui**, non nel pacchetto ROS2 (`src/haptic_dmp_learning`) — sono uno strumento di validazione/test, non qualcosa che il wrapper node usa a runtime. Namespace `dmp_tools::metrics`, distinto da `haptic_dmp_learning::core` per rendere esplicito che non fanno parte del codice "di produzione".
