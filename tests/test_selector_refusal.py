"""Test puri (senza ROS, senza Gazebo) di rifiuto del selettore e colonne dei tempi di tools/campaign_runner.py / s1_run.py."""
import re
import sys
from pathlib import Path

import pytest

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "tools"))
import campaign_runner as cr  # noqa: E402
import s1_run  # noqa: E402

PLANNER_LOG = (
    "[grasp_planner_node-1] [INFO] [100.0]: grasp_planner_node: s1 status=2 evaluations=480 rollouts=12 margin_evals=0 "
    "dynamic_evals=0 posthoc_checks=0 dv=0 m/s path_margin=0.000 rad note: v2 clearance: no finalist has clearance >= 2 mm (12 finalists)\n"
    "[grasp_planner_node-1] [INFO] [100.1]: grasp_planner_node: published GraspCommand (status=2 STATUS_NO_FEASIBLE_CANDIDATE, evaluations=480), now idle.\n"
)


# ------------------------------------------------------------------ T1: s1_run

def test_exit_codes_are_distinct():
    used = {0, 1, 2, 3, s1_run.EXIT_INTERRUPTED}   # 2 = argparse
    assert s1_run.EXIT_SELECTOR_REFUSAL == cr.EXIT_SELECTOR_REFUSAL == 4
    assert s1_run.EXIT_SELECTOR_REFUSAL not in used


def test_selector_refusal_is_a_run_error_subclass():
    assert issubclass(s1_run.SelectorRefusal, s1_run.RunError)


def test_status_constants_match_msg():
    msg = WS / "src" / "satellite_grasp_msgs" / "msg" / "GraspCommand.msg"
    if not msg.is_file():
        pytest.skip(f"manca {msg}")
    consts = {m.group(1): int(m.group(2)) for m in re.finditer(r"^uint8 (STATUS_\w+) = (\d+)", msg.read_text(), re.M)}
    assert consts == {name: v for v, name in s1_run.GRASP_STATUS_NAMES.items()}
    assert consts["STATUS_SELECTED"] == s1_run.GRASP_STATUS_SELECTED


def test_planner_refusal_note():
    assert s1_run.planner_refusal_note(PLANNER_LOG) == "v2 clearance: no finalist has clearance >= 2 mm (12 finalists)"
    assert s1_run.planner_refusal_note("nessuna riga\n") == ""
    assert s1_run.planner_refusal_note("grasp_planner_node: s1 status=2 evaluations=1\n") == ""


def test_selector_refusal_meta():
    m = s1_run.selector_refusal_meta(2, 3.25, 480, PLANNER_LOG)
    assert m["outcome"] == "RIFIUTO_SELETTORE" == s1_run.OUTCOME_SELECTOR_REFUSAL
    assert m["selector_status"] == 2 and m["selector_status_name"] == "STATUS_NO_FEASIBLE_CANDIDATE"
    assert m["selection_elapsed_s"] == 3.25 and m["selector_evaluations"] == 480
    assert m["selector_note"].startswith("v2 clearance: no finalist")
    assert s1_run.selector_refusal_meta(3, 1.0, 5, "")["selector_status_name"] == "STATUS_POSTHOC_FAILED"
    assert s1_run.selector_refusal_meta(9, 1.0, 5, "")["selector_status_name"] == "STATUS_<unknown>"


# ------------------------------------------------------------------ T1: campaign_runner

@pytest.mark.parametrize("trig", [False, True])
@pytest.mark.parametrize("msg", ["", "S1_RUN RIFIUTO_SELETTORE", "executor terminato (codice 1) prima di AT_END",
                                 "gate goal fallito: x_c=1 mm != 2+-0.1 mm"])
def test_refusal_class_by_exit_code_only(trig, msg):
    """rc 4: sempre RIFIUTO_SELETTORE, mai STARTUP/FALLITA, qualunque messaggio e trigger."""
    assert cr.classify_failure(cr.EXIT_SELECTOR_REFUSAL, msg, trig)[0] == "RIFIUTO_SELETTORE"


def test_same_messages_with_rc1_are_unchanged():
    """Il rifiuto non cambia le altre classi: gli stessi messaggi con rc 1 restano STARTUP/FALLITA come prima."""
    assert cr.classify_failure(1, "gate goal fallito: x_c=1 mm != 2+-0.1 mm", False)[0] == "STARTUP"
    assert cr.classify_failure(1, "executor terminato (codice 1) prima di AT_END", True)[0] == "FALLITA"
    assert cr.classify_failure(3, "processi residui dopo il run: ros2", True)[0] == "STOP"
    assert cr.classify_failure(130, "", True)[0] == "STOP"


def test_refusal_is_a_done_state_and_not_startup():
    assert "RIFIUTO_SELETTORE" in cr.DONE_STATES            # --resume non la rilancia
    assert "STARTUP_FAIL_RIPETUTA" in cr.DONE_STATES


def test_refusal_row_fields():
    meta = s1_run.selector_refusal_meta(2, 3.25, 480, PLANNER_LOG)
    row = {c: "" for c in cr.COLUMNS}
    row.update(status=cr.STATUS_REFUSAL, motivo=cr.refusal_motivo(meta, "x"), **cr.refusal_fields(meta))
    assert set(row) == set(cr.COLUMNS)                        # nessuna colonna sconosciuta
    assert row["status"] == "RIFIUTO_SELETTORE"
    assert "STATUS_NO_FEASIBLE_CANDIDATE" in row["motivo"] and "status=2" in row["motivo"]
    assert row["selection_elapsed_s"] == "3.2500"
    assert row["selector_status"] == "STATUS_NO_FEASIBLE_CANDIDATE"
    assert row["rifiuto_nota"].startswith("v2 clearance: no finalist")


def test_refusal_row_without_meta():
    assert cr.refusal_fields(None) == {"selector_status": "", "rifiuto_nota": ""}
    assert cr.refusal_motivo(None, "fallback") == "fallback"


def test_csv_roundtrip_has_new_columns(tmp_path):
    import csv
    p = tmp_path / "c.csv"
    row = {c: "" for c in cr.COLUMNS}
    row.update(status="RIFIUTO_SELETTORE", **cr.refusal_fields(s1_run.selector_refusal_meta(2, 1.5, 3, PLANNER_LOG)))
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cr.COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerow(row)
    got = cr.read_csv(p)[0]
    assert got["status"] == "RIFIUTO_SELETTORE" and got["selection_elapsed_s"] == "1.5000"
    for c in ("success", "criterion_11_ok", "success_1_11", "min_joint_margin_meas_deg", "min_joint_margin_pred_deg",
              "wait_s", "wait_actual_s", "wait_err_s", "lost_phase_turn", "selection_elapsed_s"):
        assert c in cr.COLUMNS


# ------------------------------------------------------------------ T3: wait_err_s, lost_phase_turn

def test_lost_phase_turn_threshold_is_named_and_default():
    assert cr.LOST_PHASE_TURN_THRESHOLD_S == 1.0


@pytest.mark.parametrize("wait_s,actual,err,lost", [
    (10.0, 10.5, "0.5000", 0),
    (10.0, 11.0, "1.0000", 0),        # sul limite: strettamente maggiore
    (10.0, 11.0001, "1.0001", 1),
    (10.0, 9.0, "-1.0000", 0),
    (5.0, 5.0 + 360.0 / 2.0, "180.0000", 1),
])
def test_wait_columns(wait_s, actual, err, lost):
    assert cr.wait_columns(wait_s, actual) == {"wait_err_s": err, "lost_phase_turn": lost}


@pytest.mark.parametrize("wait_s,actual", [(None, 1.0), (1.0, None), (float("nan"), 1.0), (1.0, float("inf")), ("x", 1.0)])
def test_wait_columns_missing_or_not_finite(wait_s, actual):
    assert cr.wait_columns(wait_s, actual) == {}


def _report(**over):
    rep = {"selection": {"selection_elapsed_s": 2.5, "provisional_delay_s": 10.0, "min_joint_margin_deg": 31.0,
                         "k": 1, "wrist_flip": False, "theta_star_deg": 12.0},
           "launch": {"wait_actual_s": 12.5}, "grasp": {"min_joint_margin_deg": 7.5},
           "criteria": {}, "success": True, "criterion_11_ok": False, "success_1_11": False}
    rep.update(over)
    return rep


def test_fill_from_report_new_columns():
    row = {c: "" for c in cr.COLUMNS}
    cr.fill_from_report(row, _report(), ("1", "A", 12.0))
    assert row["selection_elapsed_s"] == "2.5000" and row["wait_s"] == "10.0000"
    assert row["wait_actual_s"] == "12.5000" and row["wait_err_s"] == "2.5000" and row["lost_phase_turn"] == 1
    assert row["min_joint_margin_meas_deg"] == "7.5000" and row["min_joint_margin_pred_deg"] == "31.0000"
    assert (row["success"], row["criterion_11_ok"], row["success_1_11"]) == (1, 0, 0)


def test_fill_from_report_old_report_without_new_fields():
    rep = _report()
    for k in ("criterion_11_ok", "success_1_11"):
        del rep[k]
    row = {c: "" for c in cr.COLUMNS}
    cr.fill_from_report(row, rep, ("1", "A", 12.0))
    assert row["criterion_11_ok"] == "" and row["success_1_11"] == "" and row["success"] == 1


# ------------------------------------------------------------------ tempi per stadio nel CSV del runner

TIMING_LOG = ("grasp_planner_node: s1 timing: total_s=1.250000 scan_ms=410.000 rollout_ms=300.500 clearance_ms=120.250 "
              "hold_end_ms=15.125 dynamic_ms=0.000 posthoc_ms=0.000 other_ms=404.125 finalists=12\n")


def test_timing_columns_follow_selection_elapsed_and_match_report_keys():
    import s1_report
    assert cr.TIMING_COLUMNS == list(s1_report.TIMING_KEYS)
    i = cr.COLUMNS.index("selection_elapsed_s")
    assert cr.COLUMNS[i + 1:i + 1 + len(cr.TIMING_COLUMNS)] == cr.TIMING_COLUMNS
    assert len(cr.COLUMNS) == len(set(cr.COLUMNS))


def test_timing_cells_empty_when_missing():
    assert cr.timing_cells({}) == {}
    assert cr.timing_cells({"scan_ms": None, "rollout_ms": 2.0}) == {"rollout_ms": "2.000"}


def test_fill_from_report_copies_timing_and_leaves_old_reports_empty():
    rep = _report()
    rep["selection"].update({"scan_ms": 410.0, "rollout_ms": 300.5, "clearance_ms": 120.25, "hold_end_ms": 15.125,
                             "dynamic_ms": 0.0, "posthoc_ms": 0.0, "other_ms": -0.012, "finalists": 12})
    row = {c: "" for c in cr.COLUMNS}
    cr.fill_from_report(row, rep, ("1", "A", 12.0))
    assert row["scan_ms"] == "410.000" and row["rollout_ms"] == "300.500" and row["dynamic_ms"] == "0.000"
    assert row["other_ms"] == "-0.012"                       # non azzerato se negativo
    old = {c: "" for c in cr.COLUMNS}
    cr.fill_from_report(old, _report(), ("1", "A", 12.0))   # report senza i campi (log vecchi)
    assert all(old[k] == "" for k in cr.TIMING_COLUMNS)


def test_refusal_row_reads_timing_from_planner_log(tmp_path):
    (tmp_path / "planner.log").write_text("grasp_planner_node: s1 status=2 evaluations=3 note: x\n" + TIMING_LOG)
    cells = cr.planner_log_timing_cells(tmp_path)
    assert cells["scan_ms"] == "410.000" and cells["hold_end_ms"] == "15.125" and cells["other_ms"] == "404.125"
    meta = s1_run.selector_refusal_meta(2, 1.25, 3, "")
    row = {c: "" for c in cr.COLUMNS}
    row.update(status=cr.STATUS_REFUSAL, **cr.refusal_fields(meta), **cells)
    assert set(row) == set(cr.COLUMNS) and row["selection_elapsed_s"] == "1.2500"
    assert cr.planner_log_timing_cells(tmp_path / "vuota") == {}   # nessun planner.log: celle vuote
    (tmp_path / "planner.log").write_text("niente\n")
    assert cr.planner_log_timing_cells(tmp_path) == {}


def test_csv_roundtrip_with_timing(tmp_path):
    import csv
    p = tmp_path / "c.csv"
    row = {c: "" for c in cr.COLUMNS}
    row.update(status="OK", **cr.timing_cells({"scan_ms": 1.5, "other_ms": 0.25}))
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cr.COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerow(row)
    got = cr.read_csv(p)[0]
    assert got["scan_ms"] == "1.500" and got["other_ms"] == "0.250" and got["rollout_ms"] == ""
