# 04_basis_sweep

Strumento per lo studio del numero di funzioni di base ($n_{basis}$) e della dimensione della finestra del filtro di velocità.

## Descrizione del Tool
Questo esperimento analizza:
1. **Sweep su $n_{basis}$**: Valuta le prestazioni e il rischio di ill-conditioning al variare delle funzioni di base (da 5 a 500) per quantificare la capacita di approssimazione e la sovra-parametrizzazione.
2. **Sweep su Finestra di Filtro e Regressione (Ridge vs LWR)**: Analizza l'effetto delle diverse finestre di filtraggio temporale della velocità (es. `0.01s` - `0.20s`) per la traiettoria A sia con regressione Ridge che con LWR standard.

## Struttura della Cartella
- `configs/`: Cartella contenente i file YAML di configurazione delle modalità di regressione (`lwr_filter.yaml`, `ridge_filter.yaml`, ecc.).
- `scripts/run_filter_window_sweep.sh`: Script principale per eseguire lo sweep sulle finestre di filtro della Traiettoria A.
- `scripts/run_nbasis_sweep.sh`: Sweep generale su $n_{basis}$ per dati sintetici puliti e rumorosi.
- `scripts/run_real_nbasis_sweep.sh`: Sweep su $n_{basis}$ specifico per le 3 traiettorie reali.
- `plots/`: Destinazione per i grafici generati (sottocartelle `ridge/` e `noridge/`).

## Come Usarlo

1. **Eseguire lo sweep delle finestre di filtro sulla Traiettoria A**:
```bash
./04_basis_sweep/scripts/run_filter_window_sweep.sh
```

2. **Eseguire lo sweep su $n_{basis}$ per le 3 traiettorie reali**:
```bash
./04_basis_sweep/scripts/run_real_nbasis_sweep.sh
```

3. **Eseguire lo sweep su $n_{basis}$ su dati sintetici con rumore**:
```bash
./04_basis_sweep/scripts/run_nbasis_sweep.sh
```
