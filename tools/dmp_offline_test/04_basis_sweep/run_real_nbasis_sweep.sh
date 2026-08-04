#!/usr/bin/env bash
# Sweep sul numero di basi (n_basis) per le tre traiettorie reali (Traj A, B, C)
# confrontando i due metodi di regressione in REG_VARIANTS (es. LWR vs Ridge).

set -euo pipefail

N_BASIS_LIST=(5 10 15 20 25 30 40 50 60 80 100 150 200 300 500)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

PLOT_DIR="plots/04_basis_sweep/real"
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
# Trova i file demo reali (Traj A, B, C)
# --------------------------------------------------------------------------
TRAJ_ENTRIES=()
for name in "demo_raw_trajA.csv" "demo_raw_trajB.csv" "demo_raw_trajC.csv"; do
    path=""
    if [ -f "../../${name}" ]; then
        path="../../${name}"
    elif [ -f "${HOME}/thesis_ws/${name}" ]; then
        path="${HOME}/thesis_ws/${name}"
    elif [ -f "data/${name}" ]; then
        path="data/${name}"
    fi

    if [ -n "$path" ]; then
        if [[ "$name" =~ (traj[A-Z0-9]+) ]]; then
            id="${BASH_REMATCH[1]}"
        else
            id="${name%.csv}"
        fi
        TRAJ_ENTRIES+=("${id}:${path}")
    fi
done

if [ ${#TRAJ_ENTRIES[@]} -eq 0 ]; then
    echo "[ERRORE] Nessun file demo_raw_traj*.csv trovato!" >&2
    exit 1
fi

REG_VARIANTS=(
    "lwr_nofilter:04_basis_sweep/test_configs/lwr_nofilter.yaml"
    "ridge_filter:04_basis_sweep/test_configs/ridge_filter.yaml"
)

run_single_sweep () {
    local traj_id="$1"
    local demo_csv="$2"
    local rv_label="$3"
    local rv_flags="$4"

    local summary_csv="${PLOT_DIR}/nbasis_results_${traj_id}_${rv_label}.csv"
    rm -f "$summary_csv"

    echo "" >&2
    echo "== Sweep n_basis su ${traj_id} / ${rv_label} (demo: ${demo_csv}) ==" >&2
    for n_basis in "${N_BASIS_LIST[@]}"; do
        label="nbasis_$(printf '%04d' "$n_basis")_${traj_id}_${rv_label}"
        yaml_out="weights/real_${label}.yaml"
        replay_out="data/replay_real_${label}.csv"

        build/learn_and_test_dmp "$demo_csv" "$yaml_out" "$replay_out" "$summary_csv" "$label" \
            "$n_basis" - - - "${rv_flags:-}" >&2
    done
    echo "$summary_csv"
}

ALL_PLOT_ARGS=()

for entry in "${TRAJ_ENTRIES[@]}"; do
    t_id="${entry%%:*}"
    t_path="${entry#*:}"

    TRAJ_PLOT_ARGS=()
    for rv in "${REG_VARIANTS[@]}"; do
        rv_label="${rv%%:*}"
        rv_flags="${rv#*:}"

        sum_csv=$(run_single_sweep "$t_id" "$t_path" "$rv_label" "$rv_flags")
        series_name="${t_id} (${rv_label})"
        ALL_PLOT_ARGS+=(--summary-csv "$sum_csv" --series-label "$series_name")
        TRAJ_PLOT_ARGS+=(--summary-csv "$sum_csv" --series-label "${rv_label}")
    done

    # Grafici per la singola traiettoria (LWR vs Ridge per quell'id)
    python3 04_basis_sweep/plot_nbasis_study.py "${TRAJ_PLOT_ARGS[@]}" --plot-dir "${PLOT_DIR}/${t_id}"
done

echo ""
echo "== Sweep n_basis completato su tutte le traiettorie reali. Genero i grafici comparativi generali =="
python3 04_basis_sweep/plot_nbasis_study.py "${ALL_PLOT_ARGS[@]}" --plot-dir "${PLOT_DIR}/all"

echo ""
echo "== Fatto. Grafici per singola traiettoria in ${PLOT_DIR}/traj*/, grafici generali con le 6 serie in ${PLOT_DIR}/all/ =="
