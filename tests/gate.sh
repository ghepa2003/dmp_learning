#!/bin/bash
# tests/gate.sh - Gate riproducibile del refactor
# Eseguito dall'HOST, orchestra i test nel container franka e sull'host.

set -eo pipefail

HOST_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${HOST_WS}"

# ==============================================================================
# Costanti di regressione attese.
# MODIFICARE SOLO con un commit esplicito quando un refactor aggiunge o toglie test.
# ==============================================================================
EXPECTED_GTEST_FRANKA=12
EXPECTED_GTEST_HAPTIC=261
EXPECTED_GTEST_PLANNER=188
EXPECTED_PYTEST=692
EXPECTED_GTEST_TOTAL=$((EXPECTED_GTEST_FRANKA + EXPECTED_GTEST_HAPTIC + EXPECTED_GTEST_PLANNER))

# Gestione opzioni da riga di comando
MODE_CLEAN=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean)
            MODE_CLEAN=1
            shift
            ;;
        *)
            echo "Opzione sconosciuta: $1" >&2
            echo "Uso: $0 [--clean]" >&2
            exit 1
            ;;
    esac
done

SHA7="$(git rev-parse --short HEAD)"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p _refactor_1021 _gate_out
LOG_FILE="_refactor_1021/gate_${SHA7}_${TIMESTAMP}.log"

# Redirige stdout e stderr sia su console che nel file di log
exec > >(tee -a "${LOG_FILE}") 2>&1

T_GATE_START="$(date +%s)"

echo "========================================================================"
echo "GATE REFACTOR: commit=${SHA7} data=${TIMESTAMP} clean=${MODE_CLEAN}"
echo "Log file: ${LOG_FILE}"
echo "========================================================================"

# Inizializzazione contatori/stato (i valori numerici finali saranno misurati a runtime)
STATUS_BUILD="FAIL"
STATUS_GTEST="FAIL"
STATUS_PYTEST="FAIL"
STATUS_G25="FAIL"
STATUS_GIT="FAIL"
GTEST_PASSED=0
PYTEST_PASSED=0

fail_gate() {
    local step="$1"
    local t_now="$(date +%s)"
    local dur="$((t_now - T_GATE_START))"
    echo ""
    echo "========================================================================"
    echo "GATE FALLITO al passo: ${step} (durata totale: ${dur}s)"
    echo "GATE FAIL, ${SHA7}, ${GTEST_PASSED}/${PYTEST_PASSED}, ${STATUS_G25}"
    echo "========================================================================"
    exit 1
}

# ----------------------------------------------------------------------
# (a) colcon build dei 6 pacchetti
# ----------------------------------------------------------------------
echo ""
echo "--- [Passo a] colcon build (6 pacchetti) ---"
T_STEP_START="$(date +%s)"

PACKAGES="franka_cartesian_control free_target_object grasp_monitoring haptic_dmp_learning satellite_grasp_msgs satellite_grasp_planner"

if [ "${MODE_CLEAN}" -eq 1 ]; then
    echo "Modalita' --clean attiva: clean-first, force-configure, console_cohesion+"
    BUILD_CMD="colcon build --packages-select ${PACKAGES} --cmake-clean-first --cmake-force-configure --event-handlers console_cohesion+"
    docker exec franka bash -c "source /root/env_thesis.sh && cd /root/thesis_ws && ${BUILD_CMD}" 2>&1 | tee _gate_out/clean_build.log || fail_gate "(a) colcon build --clean"

    python3 tests/gate_check_build_log.py _gate_out/clean_build.log || fail_gate "(a) colcon build --clean (warning o stderr non ammessi)"
else
    docker exec franka bash -c "source /root/env_thesis.sh && cd /root/thesis_ws && \
      colcon build --packages-select ${PACKAGES}" || fail_gate "(a) colcon build"
fi

STATUS_BUILD="PASS"
T_STEP_END="$(date +%s)"
echo "[Passo a] PASS (${STATUS_BUILD}) in $((T_STEP_END - T_STEP_START))s"

# ----------------------------------------------------------------------
# (b) gtest dei tre pacchetti (ctest escludendo linters ament)
# ----------------------------------------------------------------------
echo ""
echo "--- [Passo b] gtest dei tre pacchetti (attesi ${EXPECTED_GTEST_TOTAL} test, 0 falliti) ---"
T_STEP_START="$(date +%s)"

# Eseguiamo ctest con filtro label gtest (esclude ament linters) e verifichiamo il conteggio gtest da XML
docker exec \
  -e EXPECTED_GTEST_FRANKA="${EXPECTED_GTEST_FRANKA}" \
  -e EXPECTED_GTEST_HAPTIC="${EXPECTED_GTEST_HAPTIC}" \
  -e EXPECTED_GTEST_PLANNER="${EXPECTED_GTEST_PLANNER}" \
  franka bash -c 'source /root/env_thesis.sh && cd /root/thesis_ws && python3 -c "
import os, subprocess, glob, xml.etree.ElementTree as ET, sys

packages = [\"franka_cartesian_control\", \"haptic_dmp_learning\", \"satellite_grasp_planner\"]
expected = {
    \"franka_cartesian_control\": int(os.environ[\"EXPECTED_GTEST_FRANKA\"]),
    \"haptic_dmp_learning\": int(os.environ[\"EXPECTED_GTEST_HAPTIC\"]),
    \"satellite_grasp_planner\": int(os.environ[\"EXPECTED_GTEST_PLANNER\"]),
}
expected_total = sum(expected.values())

# Pulizia test_results pregressi
for p in packages:
    subprocess.run(f\"rm -rf /root/thesis_ws/build/{p}/test_results\", shell=True)

# Esecuzione ctest -L gtest
for p in packages:
    cmd = f\"cd /root/thesis_ws/build/{p} && ctest -L gtest --output-on-failure\"
    res = subprocess.run(cmd, shell=True)
    if res.returncode != 0:
        print(f\"[GTEST FAIL] ctest fallito per pacchetto {p}\", file=sys.stderr)
        sys.exit(1)

total_gtest = 0
total_fail = 0
for p in packages:
    xmls = glob.glob(f\"/root/thesis_ws/build/{p}/test_results/{p}/*.gtest.xml\")
    p_tests = 0
    p_fail = 0
    for x in xmls:
        root = ET.parse(x).getroot()
        p_tests += int(root.get(\"tests\", 0))
        p_fail += int(root.get(\"failures\", 0)) + int(root.get(\"errors\", 0))
    print(f\"  {p}: {p_tests} test (attesi: {expected[p]}), {p_fail} falliti\")
    total_gtest += p_tests
    total_fail += p_fail
    if p_tests != expected[p]:
        print(f\"[GTEST FAIL] Conteggio errato per {p}: {p_tests} != {expected[p]}\", file=sys.stderr)
        sys.exit(1)

print(f\"Totale gtest: {total_gtest} test, {total_fail} falliti\")
if total_gtest != expected_total or total_fail != 0:
    print(f\"[GTEST FAIL] Totale non corrispondente ({expected_total} atteso, 0 fail)\", file=sys.stderr)
    sys.exit(1)

os.makedirs(\"/root/thesis_ws/_gate_out\", exist_ok=True)
with open(\"/root/thesis_ws/_gate_out/gtest_total.txt\", \"w\") as f:
    f.write(str(total_gtest))
"' || fail_gate "(b) gtest"

GTEST_PASSED="$(cat _gate_out/gtest_total.txt)"
STATUS_GTEST="PASS"
T_STEP_END="$(date +%s)"
echo "[Passo b] PASS (${STATUS_GTEST}) in $((T_STEP_END - T_STEP_START))s"

# ----------------------------------------------------------------------
# (c) pytest tests (PYTHONDONTWRITEBYTECODE=1 -p no:cacheprovider)
# ----------------------------------------------------------------------
echo ""
echo "--- [Passo c] pytest tests (attesi ${EXPECTED_PYTEST} passed, 0 skipped) ---"
T_STEP_START="$(date +%s)"

mkdir -p _gate_out
docker exec franka bash -c "source /root/env_thesis.sh && cd /root/thesis_ws && \
  PYTHONDONTWRITEBYTECODE=1 pytest tests -p no:cacheprovider" 2>&1 | tee _gate_out/pytest_last.txt || fail_gate "(c) pytest"

CHECK_OUT="$(python3 tests/gate_check_pytest.py "${EXPECTED_PYTEST}" < _gate_out/pytest_last.txt)" || fail_gate "(c) gate_check_pytest"
echo "${CHECK_OUT}"

if [[ "${CHECK_OUT}" =~ ([0-9]+)\ passed ]]; then
    PYTEST_PASSED="${BASH_REMATCH[1]}"
else
    fail_gate "(c) parsing PYTEST_PASSED"
fi

STATUS_PYTEST="PASS"
T_STEP_END="$(date +%s)"
echo "[Passo c] PASS (${STATUS_PYTEST}) in $((T_STEP_END - T_STEP_START))s"

# ----------------------------------------------------------------------
# (d) build_and_run_g25.sh
# ----------------------------------------------------------------------
echo ""
echo "--- [Passo d] build_and_run_g25.sh ---"
T_STEP_START="$(date +%s)"

docker exec franka bash -c "cd /root/thesis_ws && ./tests/golden/g25/build_and_run_g25.sh" || fail_gate "(d) build_and_run_g25.sh"

STATUS_G25="PASS"
T_STEP_END="$(date +%s)"
echo "[Passo d] PASS (${STATUS_G25}) in $((T_STEP_END - T_STEP_START))s"

# ----------------------------------------------------------------------
# (e) git status --short -uno vuoto
# ----------------------------------------------------------------------
echo ""
echo "--- [Passo e] git status --short -uno ---"
T_STEP_START="$(date +%s)"

GIT_DIRTY="$(git status --short -uno)"
if [ -n "${GIT_DIRTY}" ]; then
    echo "ERRORE: Working tree tracciato sporco:"
    echo "${GIT_DIRTY}"
    fail_gate "(e) git status non vuoto"
fi

STATUS_GIT="PASS"
T_STEP_END="$(date +%s)"
echo "[Passo e] PASS (${STATUS_GIT}) - working tree tracciato pulito in $((T_STEP_END - T_STEP_START))s"

# ----------------------------------------------------------------------
# Riepilogo finale
# ----------------------------------------------------------------------
T_GATE_END="$(date +%s)"
DUR_TOTAL="$((T_GATE_END - T_GATE_START))"

echo ""
echo "========================================================================"
echo "GATE COMPLETATO CON SUCCESSO in ${DUR_TOTAL}s"
echo "GATE PASS, ${SHA7}, ${GTEST_PASSED}/${PYTEST_PASSED}, ${STATUS_G25}"
echo "========================================================================"
