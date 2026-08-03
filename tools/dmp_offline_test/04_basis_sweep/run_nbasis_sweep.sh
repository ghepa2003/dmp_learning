#!/usr/bin/env bash
# Sweep sul numero di basi (n_basis). Due modalita':
#
#   1) Sintetica (default): genera e usa una coppia di demo sintetiche
#      (pulita + con rumore) sulla stessa forma di traiettoria.
#        ./run_nbasis_sweep.sh [duration=60] [goal_name=goalA_main]
#
#   2) Demo reale: salta la generazione sintetica, fa lo sweep solo sul
#      file reale fornito.
#        ./run_nbasis_sweep.sh --real-demo path/to/demo_raw.csv
#
# Modifica N_BASIS_LIST e i parametri di rumore qui sotto per cambiare il
# range/l'intensita' testati (validi solo in modalita' sintetica).

set -euo pipefail

REAL_DEMO=""
if [ "${1:-}" = "--real-demo" ]; then
    REAL_DEMO="${2:?Uso: $0 --real-demo <path_al_csv>}"
    shift 2
fi

DURATION="${1:-60}"
GOAL_NAME="${2:-goalA_main}"

# Range di n_basis da testare -- esteso rispetto al primo giro, per vedere
# se/dove compare comunque un limite (ill-conditioning) anche su dati puliti.
N_BASIS_LIST=(5 10 15 20 25 30 40 50 60 80 100 150 200 300 500)

# Parametri della traiettoria (stessi usati finora per goalA_main + rotazione).
DX=0.15
DY=0.0
DZ=-0.10
ROT_AXIS="0,0,1"
ROT_ANGLE_DEG=15.0

# Parametri del rumore sintetico (imita quantizzazione/jitter reale).
POS_NOISE_STD=0.0005        # 0.5 mm
ORIENT_NOISE_STD_DEG=0.1
# Piu' seed per stimare la variabilita' della serie rumorosa, non un solo
# campione di rumore -- ciascuno produce un CSV distinto (vedi nota cache).
NOISE_SEEDS=(42 43 44 45 46)

# Metodi di regressione da confrontare, oltre a pulito/rumoroso gia'
# esistente. Formato "etichetta:percorso_yaml".
#
# Il percorso "ridge" e' lo STESSO file che legge haptic_dmp_wrapper_node.cpp
# in produzione (parametro ROS2 feature_flags_path, default sotto) -- non una
# copia per i test. Per testare davvero la ridge regression qui, apri quel
# file e verifica che contenga "method: \"ridge\"" PRIMA di lanciare lo
# sweep: essendo lo stesso file, questo script non puo' forzarlo per te.
REAL_FEATURE_FLAGS_PATH="${HOME}/thesis_ws/dmp_features.yaml"
REGRESSION_VARIANTS=(
    "lwr:"
    "ridge:${REAL_FEATURE_FLAGS_PATH}"
)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

PLOT_DIR="plots/04_basis_sweep"
mkdir -p build data "${PLOT_DIR}" weights

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

# Varianti di regolarizzazione da testare (formato "label" oppure "label:path/to/flags.yaml").
DEFAULT_FEATURES_PATH="../../src/haptic_dmp_learning/config/dmp_features.yaml"
REG_VARIANTS=("dmp_features:${DEFAULT_FEATURES_PATH}")

run_sweep_variant () {
    local variant_name="$1"   # "clean" o "noisy"
    local demo_csv="$2"
    local summary_csv="${PLOT_DIR}/nbasis_sweep_results_${variant_name}.csv"
    rm -f "$summary_csv"

    echo "" >&2
    echo "== Variante: ${variant_name} (demo: ${demo_csv}) ==" >&2
    for rv in "${REG_VARIANTS[@]}"; do
        rv_label="${rv%%:*}"
        rv_flags="${rv#*:}"
        if [ "$rv_flags" = "$rv_label" ]; then
            rv_flags=""
        fi

        if [ -n "$rv_flags" ] && [ ! -f "$rv_flags" ]; then
            echo "[ATTENZIONE] '${rv_flags}' non esiste -- la variante '${rv_label}' userà i default (probabilmente NON ridge)." >&2
            echo "             Crea quel file con 'method: \"ridge\"' prima di continuare se vuoi testare ridge davvero." >&2
        fi

        for n_basis in "${N_BASIS_LIST[@]}"; do
            label="nbasis_$(printf '%04d' "$n_basis")"
            if [ -n "$rv_label" ] && [ "$rv_label" != "standard" ]; then
                label="${label}_${rv_label}"
            fi
            yaml_out="weights/${variant_name}_${label}.yaml"
            replay_out="data/replay_${variant_name}_${label}.csv"

            echo "---- ${variant_name} / ${label} ----" >&2
            build/learn_and_test_dmp "$demo_csv" "$yaml_out" "$replay_out" "$summary_csv" "$label" \
                "$n_basis" - - - "${rv_flags:-}" >&2
        done
    done
    echo "$summary_csv"
}

# --- Demo pulita ---
CLEAN_CSV="data/demo_nbasis_sweep_${DURATION}s_${GOAL_NAME}_clean.csv"
if [ -z "$REAL_DEMO" ] && [ ! -f "$CLEAN_CSV" ]; then
    echo "== Genero la demo pulita (${DURATION}s, transizione = durata, nessuna tenuta) =="
    python3 04_basis_sweep/generate_picking_trajectory.py \
        --duration "$DURATION" --transition-duration "$DURATION" \
        --dx "$DX" --dy "$DY" --dz "$DZ" \
        --rot-axis "$ROT_AXIS" --rot-angle-deg "$ROT_ANGLE_DEG" \
        --output "$CLEAN_CSV"
fi

# --- Demo con rumore sintetico: una per ciascun seed in NOISE_SEEDS.
# Il nome del file include il seed apposta -- con un nome fisso, cambiare
# NOISE_SEED senza cancellare data/ riuserebbe silenziosamente il CSV vecchio.
NOISY_CSVS=()
if [ -z "$REAL_DEMO" ]; then
    for seed in "${NOISE_SEEDS[@]}"; do
        noisy_csv="data/demo_nbasis_sweep_${DURATION}s_${GOAL_NAME}_noisy_seed${seed}.csv"
        if [ ! -f "$noisy_csv" ]; then
            echo "== Genero la demo con rumore sintetico, seed=${seed} (pos_std=${POS_NOISE_STD}m, orient_std=${ORIENT_NOISE_STD_DEG}deg) =="
            python3 04_basis_sweep/generate_picking_trajectory.py \
                --duration "$DURATION" --transition-duration "$DURATION" \
                --dx "$DX" --dy "$DY" --dz "$DZ" \
                --rot-axis "$ROT_AXIS" --rot-angle-deg "$ROT_ANGLE_DEG" \
                --pos-noise-std "$POS_NOISE_STD" --orient-noise-std-deg "$ORIENT_NOISE_STD_DEG" \
                --noise-seed "$seed" \
                --output "$noisy_csv"
        fi
        NOISY_CSVS+=("$noisy_csv")
    done
fi

if [ -n "$REAL_DEMO" ]; then
    if [ ! -f "$REAL_DEMO" ]; then
        echo "Non trovo il file demo reale: ${REAL_DEMO}"
        exit 1
    fi
    echo "== Modalita' demo reale: ${REAL_DEMO} (nessuna generazione sintetica) =="
    REAL_SUMMARY=$(run_sweep_variant "real" "$REAL_DEMO")

    echo ""
    echo "== Sweep completato. Genero i grafici (demo reale) =="
    python3 04_basis_sweep/plot_nbasis_study.py \
        --summary-csv "$REAL_SUMMARY" --series-label "reale" \
        --plot-dir "${PLOT_DIR}"

    echo ""
    echo "== Fatto. Riepilogo in ${REAL_SUMMARY}, grafici in ${PLOT_DIR}/ =="
    exit 0
fi

CLEAN_SUMMARY=$(run_sweep_variant "clean" "$CLEAN_CSV")

PLOT_ARGS=(--summary-csv "$CLEAN_SUMMARY" --series-label "pulito")
for i in "${!NOISE_SEEDS[@]}"; do
    seed="${NOISE_SEEDS[$i]}"
    noisy_summary=$(run_sweep_variant "noisy_seed${seed}" "${NOISY_CSVS[$i]}")
    PLOT_ARGS+=(--summary-csv "$noisy_summary" --series-label "rumoroso (seed ${seed})")
done

echo ""
echo "== Sweep completato. Genero i grafici (pulito vs rumoroso x ${#NOISE_SEEDS[@]} seed) =="
python3 04_basis_sweep/plot_nbasis_study.py "${PLOT_ARGS[@]}" --plot-dir "${PLOT_DIR}"

echo ""
echo "== Fatto. Riepiloghi in ${PLOT_DIR}/nbasis_sweep_results_{clean,noisy_seed*}.csv, grafici in ${PLOT_DIR}/ =="
