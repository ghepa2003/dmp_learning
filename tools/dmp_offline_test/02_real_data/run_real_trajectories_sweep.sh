#!/usr/bin/env bash
# Sweep e analisi comparativa sulle tre traiettorie reali (Traj A, Traj B, Traj C).
#
# Esegue l'apprendimento e il replay per ciascuna traiettoria reale sui 2 metodi
# di regressione specificati in REGRESSION_VARIANTS (es. LWR vs Ridge).
# Genera sia i grafici per-singola-prova (3D, posizione e errore angolare) sia
# i grafici comparativi aggregati sulle metriche (RMSE, errore max, errore finale).

set -euo pipefail

N_BASIS="${1:-20}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

PLOT_DIR="plots/02_real_data"
mkdir -p build data "${PLOT_DIR}" weights

# --------------------------------------------------------------------------
# Compilazione learn_and_test_dmp
# --------------------------------------------------------------------------
PKG_DIR="../../src/haptic_dmp_learning"
if [ ! -x build/learn_and_test_dmp ] || \
   [ "${PKG_DIR}/src/core/dmp.cpp" -nt build/learn_and_test_dmp ] || \
   [ "${PKG_DIR}/src/core/quaternion_dmp.cpp" -nt build/learn_and_test_dmp ] || \
   [ "${PKG_DIR}/src/core/dmp_io.cpp" -nt build/learn_and_test_dmp ] || \
   [ "03_time_sweep/learn_and_test_dmp.cpp" -nt build/learn_and_test_dmp ]; then
    echo "== Compilazione learn_and_test_dmp =="
    g++ -std=c++17 -O2 \
        -I "${PKG_DIR}/include" \
        -I 03_time_sweep \
        -I common \
        -I/usr/include/eigen3 \
        03_time_sweep/learn_and_test_dmp.cpp \
        common/metrics.cpp \
        "${PKG_DIR}/src/core/dmp.cpp" \
        "${PKG_DIR}/src/core/quaternion_dmp.cpp" \
        "${PKG_DIR}/src/core/dmp_io.cpp" \
        -o build/learn_and_test_dmp \
        -lyaml-cpp
fi

# --------------------------------------------------------------------------
# Individuazione delle 3 traiettorie reali (trajA, trajB, trajC)
# --------------------------------------------------------------------------
TRAJ_FILES=()
for name in "demo_raw_trajA.csv" "demo_raw_trajB.csv" "demo_raw_trajC.csv"; do
    if [ -f "../../${name}" ]; then
        TRAJ_FILES+=("../../${name}")
    elif [ -f "${HOME}/thesis_ws/${name}" ]; then
        TRAJ_FILES+=("${HOME}/thesis_ws/${name}")
    elif [ -f "data/${name}" ]; then
        TRAJ_FILES+=("data/${name}")
    fi
done

if [ ${#TRAJ_FILES[@]} -eq 0 ]; then
    echo "[ERRORE] Nessun file demo_raw_traj*.csv trovato!" >&2
    exit 1
fi

# --------------------------------------------------------------------------
# I 2 metodi in REGRESSION_VARIANTS (es. LWR e Ridge)
# --------------------------------------------------------------------------
REGRESSION_VARIANTS=(
    "lwr_nofilter:04_basis_sweep/test_configs/lwr_nofilter.yaml"
    "ridge_filter:04_basis_sweep/test_configs/ridge_filter.yaml"
)

SUMMARY_CSV="${PLOT_DIR}/real_trajectories_summary.csv"
rm -f "$SUMMARY_CSV"

echo "== Avvio analisi su ${#TRAJ_FILES[@]} traiettorie reali x ${#REGRESSION_VARIANTS[@]} varianti (n_basis=${N_BASIS}) =="

for demo_csv in "${TRAJ_FILES[@]}"; do
    filename="$(basename "$demo_csv")"
    # Estrae trajA, trajB, trajC dal nome del file
    if [[ "$filename" =~ (traj[A-Z0-9]+) ]]; then
        traj_id="${BASH_REMATCH[1]}"
    else
        traj_id="${filename%.csv}"
    fi

    for rv in "${REGRESSION_VARIANTS[@]}"; do
        rv_label="${rv%%:*}"
        rv_flags="${rv#*:}"
        if [ "$rv_flags" = "$rv_label" ] || [ ! -f "$rv_flags" ]; then
            rv_flags=""
        fi

        label="real_${traj_id}_${rv_label}"
        yaml_out="weights/${label}.yaml"
        replay_out="data/replay_${label}.csv"

        echo "---- Traiettoria Reale ${traj_id} / Variante ${rv_label} ----"
        build/learn_and_test_dmp "$demo_csv" "$yaml_out" "$replay_out" "$SUMMARY_CSV" "$label" \
            "$N_BASIS" - - - "${rv_flags:-}"

        # Grafici per-singola-prova (3D, posizioni nel tempo, errore angolare)
        python3 03_time_sweep/plot_dmp_timesweep.py \
            --demo "$demo_csv" \
            --replay "$replay_out" \
            --plot-dir "${PLOT_DIR}" \
            --label "$label"
    done
done

echo ""
echo "== Genero i grafici comparativi sulle metriche tra le 3 traiettorie =="
python3 02_real_data/plot_real_trajectories_study.py \
    --summary-csv "$SUMMARY_CSV" \
    --plot-dir "${PLOT_DIR}"

echo ""
echo "== Analisi completata. Riepilogo numerico in ${SUMMARY_CSV}, grafici in ${PLOT_DIR}/ =="
