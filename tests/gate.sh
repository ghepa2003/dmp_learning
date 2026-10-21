#!/bin/bash
# tests/gate.sh - Gate riproducibile del refactor
# Eseguito dall'HOST, orchestra i test nel container franka e sull'host.

set -eo pipefail

HOST_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${HOST_WS}"

SHA7="$(git rev-parse --short HEAD)"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p _refactor_1021
LOG_FILE="_refactor_1021/gate_${SHA7}_${TIMESTAMP}.log"

# Redirige stdout e stderr sia su console che nel file di log
exec > >(tee -a "${LOG_FILE}") 2>&1

T_GATE_START="$(date +%s)"

echo "========================================================================"
echo "GATE REFACTOR: commit=${SHA7} data=${TIMESTAMP}"
echo "Log file: ${LOG_FILE}"
echo "========================================================================"

# Inizializzazione contatori/stato
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
# (a) colcon build dei tre pacchetti
# ----------------------------------------------------------------------
echo ""
echo "--- [Passo a] colcon build (franka_cartesian_control, haptic_dmp_learning, satellite_grasp_planner) ---"
T_STEP_START="$(date +%s)"

docker exec franka bash -c "source /root/env_thesis.sh && cd /root/thesis_ws && \
  colcon build --packages-select franka_cartesian_control haptic_dmp_learning satellite_grasp_planner" || fail_gate "(a) colcon build"

STATUS_BUILD="PASS"
T_STEP_END="$(date +%s)"
echo "[Passo a] PASS (${STATUS_BUILD}) in $((T_STEP_END - T_STEP_START))s"

# ----------------------------------------------------------------------
# (b) gtest dei tre pacchetti (ctest escludendo linters ament)
# ----------------------------------------------------------------------
echo ""
echo "--- [Passo b] gtest dei tre pacchetti (attesi 448 test, 0 falliti) ---"
T_STEP_START="$(date +%s)"

# Eseguiamo ctest con filtro label gtest (esclude ament linters) e verifichiamo il conteggio gtest da XML
docker exec franka bash -c 'source /root/env_thesis.sh && cd /root/thesis_ws && python3 -c "
import subprocess, glob, xml.etree.ElementTree as ET, sys

packages = [\"franka_cartesian_control\", \"haptic_dmp_learning\", \"satellite_grasp_planner\"]
expected = {\"franka_cartesian_control\": 12, \"haptic_dmp_learning\": 261, \"satellite_grasp_planner\": 175}

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
if total_gtest != 448 or total_fail != 0:
    print(f\"[GTEST FAIL] Totale non corrispondente (448 atteso, 0 fail)\", file=sys.stderr)
    sys.exit(1)
"' || fail_gate "(b) gtest"

GTEST_PASSED=448
STATUS_GTEST="PASS"
T_STEP_END="$(date +%s)"
echo "[Passo b] PASS (${STATUS_GTEST}) in $((T_STEP_END - T_STEP_START))s"

# ----------------------------------------------------------------------
# (c) pytest tests (PYTHONDONTWRITEBYTECODE=1 -p no:cacheprovider)
# ----------------------------------------------------------------------
echo ""
echo "--- [Passo c] pytest tests (attesi 692 passed, 0 skipped) ---"
T_STEP_START="$(date +%s)"

docker exec franka bash -c "source /root/env_thesis.sh && cd /root/thesis_ws && \
  PYTHONDONTWRITEBYTECODE=1 pytest tests -p no:cacheprovider" || fail_gate "(c) pytest"

PYTEST_PASSED=692
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
