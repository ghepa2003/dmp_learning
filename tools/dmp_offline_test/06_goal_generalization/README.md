# 06_goal_generalization

Strumento di benchmark per la generalizzazione verso nuovi goal spaziali e test dei meccansmi di guardrail (sicurezza dell'inviluppo di movimento).

## Descrizione del Tool
Questo esperimento esegue due batterie di test principali:
1. **Goal Generalization**: Addestra il modello DMP una sola volta su ciascuna traiettoria reale (Traj A, B, C) e ne testa la capacità di raggiungere 5 nuovi goal spaziali senza ri-addestrare i pesi (sfruttando la proprietà di scaling dinamico delle DMP).
2. **Guardrail Benchmark**: Test rigoroso dei vincoli di sicurezza (guardrail di velocità, accelerazione e spazio di lavoro) per garantire l'arresto o la saturazione morbida in presenza di comandi anomali o instabilità.

## Struttura della Cartella
- `scripts/run_goal_generalization.cpp`: Esegue la generalizzazione su 5 nuovi goal per le traiettorie reali.
- `scripts/test_guardrails_rigorous.cpp`: Benchmark per la validazione delle funzioni di sicurezza (Guardrails).
- `scripts/run_new_goals.sh`: Script di avvio per il test di generalizzazione verso nuovi goal.
- `scripts/run_rigorous_guardrail_tests.sh`: Script per eseguire l'intera suite di benchmark sui guardrail.
- `scripts/plot_goal_generalization.py`: Grafici delle traiettorie per la generalizzazione dei goal.
- `scripts/plot_guardrails_rigorous.py`: Visualizzazione degli errori temporali e dell'intervento dei guardrail.
- `plots/`: Destinazione per i grafici generati.

## Come Usarlo

1. **Eseguire il test di generalizzazione su 5 nuovi goal**:
```bash
./06_goal_generalization/scripts/run_new_goals.sh 100
```

2. **Eseguire il benchmark rigoroso sui guardrail di sicurezza**:
```bash
./06_goal_generalization/scripts/run_rigorous_guardrail_tests.sh
```
