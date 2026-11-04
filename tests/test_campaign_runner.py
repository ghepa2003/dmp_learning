"""pytest: tabella di classificazione dei guasti e costruzione dei comandi di tools/campaign_runner.py (nessun Gazebo)."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import campaign_runner as cr  # noqa: E402


@pytest.mark.parametrize("rc,msg,trig,cls", [
    (0, "", False, "OK"),
    (1, "bringup terminato prematuramente (vedi bringup.log)", False, "STARTUP"),
    (1, "controller non attivo entro 70 s", False, "STARTUP"),
    (1, "executor non pronto entro 60 s dall'avvio: riga/e mancante/i", False, "STARTUP"),
    (1, "timeout in tempo simulato (501 s > 500 s) senza AT_END", False, "STARTUP"),
    (1, "timeout in tempo simulato (501 s > 500 s) senza AT_END", True, "FALLITA"),
    (1, "executor terminato (codice 1) prima di AT_END", True, "FALLITA"),
    (1, "/clock fermo durante la registrazione post AT_END", False, "FALLITA"),
    (1, "bag non chiuso entro 60 s (bag possibilmente corrotto)", True, "FALLITA"),
    (1, "report: manca qualcosa", True, "FALLITA"),
    (1, "--axis-tilt-deg 25 fuori da [0, 20]", False, "CONFIG"),
    (1, "file mancante: /x/weights.yaml", False, "CONFIG"),
    (1, "/tmp/x/bag esiste gia' (usa --overwrite o un altro --tag)", False, "CONFIG"),
    (3, "processi residui dopo il run: ros2", True, "STOP"),
    (1, "processi residui prima del run: gz", False, "STOP"),
    (130, "", True, "STOP"),
    (1, "", False, "STARTUP"),
])
def test_classify_failure(rc, msg, trig, cls):
    assert cr.classify_failure(rc, msg, trig)[0] == cls


def test_last_fail_message():
    assert cr.last_fail_message("x\nS1_RUN FAIL: primo\nS1_RUN FAIL: secondo\n") == "secondo"
    assert cr.last_fail_message("niente") == ""


def test_build_cmd_profile_and_scenario():
    sc = {"scenario_id": "d0_s001", "omega_deg_s": "-3", "alpha_deg": "10", "phi_deg": "90", "cx": "0.75", "cy": "-0.05", "cz": "0.82"}
    run = {"scenario": sc, "arm": "B", "weights_dir": "/w/demo", "spawn_yaw": "-12.500"}
    cmd = cr.build_cmd(run, "t", Path("/o/t"))
    assert cmd[2:4] == ["--weights-dir", "/w/demo"]
    for flag in ("--squeeze", "-0.05", "--stop-cube", "--post-s", "45.0", "--kt", "2000"):
        assert flag in cmd
    assert cmd[cmd.index("--arm") + 1] == "B"
    assert cmd[cmd.index("--center-xyz") + 1] == "0.75,-0.05,0.82"
    assert cmd[cmd.index("--spawn-yaw-deg") + 1] == "-12.500"
    assert "--keep-alive" not in cmd and "--gui" not in cmd


def test_unique_name(tmp_path):
    p = tmp_path / "a"
    assert cr.unique_name(p) == p
    p.mkdir()
    assert cr.unique_name(p) == Path(f"{p}_2")


# ------------------------------------------------------------ plan_runs: --arms dryrun | both

def _plan_inputs():
    base = {"omega_deg_s": "-2", "alpha_deg": "0", "phi_deg": "0", "cx": "0.7", "cy": "0", "cz": "0.77", "demo": "demos/d1"}
    manifest = [dict(base, scenario_id=f"s{i}") for i in range(4)]

    def dry(sid, need, a=True, b=True):
        d = dict(base, scenario_id=sid, run_needed=need)
        for arm, ok in (("A", a), ("B", b)):
            d.update({f"winner_{arm}_k": "1" if ok else "", f"winner_{arm}_branch": "A" if ok else "",
                      f"winner_{arm}_theta_deg": "10.0" if ok else "", f"spawn_yaw_deg_{arm}": "-5.000" if ok else ""})
        return d

    dryrun = [dry("s0", "A", b=True), dry("s1", "B", a=False), dry("s2", "both"), dry("s3", "none", a=False, b=False)]
    return manifest, dryrun, {"d1": "/w/d1"}


def _keys(runs):
    return [(r["scenario"]["scenario_id"], r["arm"], bool(r.get("skip"))) for r in runs]


def test_plan_runs_dryrun_is_unchanged():
    manifest, dryrun, dm = _plan_inputs()
    assert _keys(cr.plan_runs(manifest, dryrun, dm)) == _keys(cr.plan_runs(manifest, dryrun, dm, "dryrun")) == [
        ("s0", "A", False), ("s1", "B", False), ("s2", "A", False), ("s2", "B", False), ("s3", "", True)]


def test_plan_runs_both_ignores_run_needed():
    manifest, dryrun, dm = _plan_inputs()
    runs = cr.plan_runs(manifest, dryrun, dm, "both")
    assert _keys(runs) == [("s0", "A", False), ("s0", "B", False), ("s1", "A", True), ("s1", "B", False),
                           ("s2", "A", False), ("s2", "B", False), ("s3", "A", True), ("s3", "B", True)]
    launched = [r for r in runs if not r.get("skip")]
    assert all(r["weights_dir"] == "/w/d1" and r["spawn_yaw"] == "-5.000" for r in launched)
    assert "nessun vincitore" in next(r for r in runs if r.get("skip"))["skip"]


def test_plan_runs_both_needs_demo_map_only_when_launchable():
    manifest, dryrun, _ = _plan_inputs()
    with pytest.raises(SystemExit):
        cr.plan_runs(manifest, dryrun, {}, "both")
    only_none = [m for m in manifest if m["scenario_id"] == "s3"]
    assert all(r.get("skip") for r in cr.plan_runs(only_none, dryrun, {}, "both"))


def test_plan_runs_A_ignores_run_needed():
    manifest, dryrun, dm = _plan_inputs()
    runs = cr.plan_runs(manifest, dryrun, dm, "A")
    assert _keys(runs) == [("s0", "A", False), ("s1", "A", True), ("s2", "A", False), ("s3", "A", True)]
    launched = [r for r in runs if not r.get("skip")]
    assert all(r["weights_dir"] == "/w/d1" and r["spawn_yaw"] == "-5.000" for r in launched)
    assert all("braccio A" in r["skip"] and "nessun vincitore" in r["skip"] for r in runs if r.get("skip"))


def test_plan_runs_A_needs_demo_map_only_when_launchable():
    manifest, dryrun, _ = _plan_inputs()
    with pytest.raises(SystemExit):
        cr.plan_runs(manifest, dryrun, {}, "A")
    only_none = [m for m in manifest if m["scenario_id"] == "s3"]
    assert all(r.get("skip") for r in cr.plan_runs(only_none, dryrun, {}, "A"))


def test_status_csv_ends_with_loadavg_columns():
    assert cr.COLUMNS[-3:] == ["loadavg_1m_start", "loadavg_5m_start", "loadavg_15m_start"]


def test_plan_runs_unknown_arms():
    manifest, dryrun, dm = _plan_inputs()
    with pytest.raises(SystemExit):
        cr.plan_runs(manifest, dryrun, dm, "tutti")


# ------------------------------------------------------------ residui: demone ros2cli e STOP dopo una corsa conclusa

DAEMON = ("496272 1 /usr/bin/python3 -c from ros2cli.daemon.daemonize import main; main() "
          "--name ros2-daemon --ros-domain-id 0 --rmw-implementation rmw_fastrtps_cpp")


def _fake_ps(monkeypatch, lines):
    """ps finto: ogni riga e' 'pid ppid args'."""
    class R:
        stdout = "\n".join(lines) + "\n"
    monkeypatch.setattr(cr.subprocess, "run", lambda *a, **k: R())


def test_ros2_daemon_is_not_a_residue(monkeypatch):
    _fake_ps(monkeypatch, [DAEMON])
    assert cr.residual_processes() == []


@pytest.mark.parametrize("args", [
    "ros2 launch satellite_grasp_planner bringup.launch.py",
    "/opt/ros/gz/bin/gzserver world.sdf",
    "/opt/ros/gz/bin/gzclient",
    "/ws/install/lib/satellite_grasp_planner/grasp_planner_node",
    "/ws/install/lib/haptic_dmp_learning/prodmp_gazebo_executor_node",
    "/opt/ros/lib/ros_gz_bridge/parameter_bridge /clock",
])
def test_real_residues_are_still_caught(monkeypatch, args):
    _fake_ps(monkeypatch, [f"700 1 {args}"])
    assert cr.residual_processes() == [f"700 {args}"]


def test_daemon_next_to_real_residue_only_residue_reported(monkeypatch):
    _fake_ps(monkeypatch, [DAEMON, "700 1 /opt/ros/gz/bin/gzserver world.sdf"])
    assert cr.residual_processes() == ["700 /opt/ros/gz/bin/gzserver world.sdf"]


def _run_main(monkeypatch, tmp_path, rc, residues, report=None):
    """main() con tutto stubbato (nessun processo vero). residues: lista di risultati per le chiamate successive a residual_processes.
    Ritorna (codice di main, righe del CSV di stato, numero di corse lanciate)."""
    runs = [{"scenario": {"scenario_id": f"s{i}"}, "arm": "A", "run_needed": "A", "predicted": ("1", "A", "10.0"),
             "spawn_yaw": "-5.000", "weights_dir": "/w"} for i in range(2)]
    launched = []
    calls = iter(residues)
    monkeypatch.setattr(cr, "read_csv", lambda p: [])
    monkeypatch.setattr(cr, "read_demo_map", lambda p: {})
    monkeypatch.setattr(cr, "plan_runs", lambda *a, **k: runs)
    monkeypatch.setattr(cr, "build_cmd", lambda *a, **k: ["x"])
    monkeypatch.setattr(cr, "run_once", lambda *a, **k: (launched.append(1), (rc, "", False))[1])
    monkeypatch.setattr(cr, "trigger_seen", lambda o: True)
    monkeypatch.setattr(cr, "read_report", lambda o: report)
    monkeypatch.setattr(cr, "read_run_meta", lambda o: None)
    monkeypatch.setattr(cr, "planner_log_timing_cells", lambda o: {})
    monkeypatch.setattr(cr, "fill_from_report", lambda row, rep, pred: row.update(success=int(rep["success"])))
    monkeypatch.setattr(cr, "residual_processes", lambda: next(calls, []))
    status = tmp_path / "campaign_runs.csv"
    code = cr.main(["--manifest", "m", "--dryrun", "d", "--demo-map", "dm", "--campaign-tag", "t",
                    "--outdir-root", str(tmp_path), "--status-csv", str(status)])
    return code, cr.csv.DictReader(open(status, newline="")), len(launched)


def test_ok_run_with_residue_after_keeps_ok_and_stops(monkeypatch, tmp_path):
    code, rows, n = _run_main(monkeypatch, tmp_path, 0, [[], ["700 ros2 launch x"]], report={"success": True})
    rows = list(rows)
    assert code == 3 and n == 1 and len(rows) == 1
    assert rows[0]["status"] == "OK" and rows[0]["success"] == "1"
    assert "700 ros2 launch x" in rows[0]["motivo"] and rows[0]["motivo"].startswith("STOP:")


def test_refusal_run_with_residue_after_keeps_refusal_and_stops(monkeypatch, tmp_path):
    code, rows, n = _run_main(monkeypatch, tmp_path, 4, [[], ["700 gzserver w.sdf"]])
    rows = list(rows)
    assert code == 3 and n == 1 and len(rows) == 1
    assert rows[0]["status"] == cr.STATUS_REFUSAL
    assert "700 gzserver w.sdf" in rows[0]["motivo"] and "rifiuto del selettore" in rows[0]["motivo"]


def test_daemon_alone_before_run_does_not_stop_campaign(monkeypatch, tmp_path):
    _fake_ps(monkeypatch, [DAEMON])
    runs = [{"scenario": {"scenario_id": f"s{i}"}, "arm": "A", "run_needed": "A", "predicted": ("1", "A", "10.0"),
             "spawn_yaw": "-5.000", "weights_dir": "/w"} for i in range(2)]
    launched = []
    monkeypatch.setattr(cr, "read_csv", lambda p: [])
    monkeypatch.setattr(cr, "read_demo_map", lambda p: {})
    monkeypatch.setattr(cr, "plan_runs", lambda *a, **k: runs)
    monkeypatch.setattr(cr, "build_cmd", lambda *a, **k: ["x"])
    monkeypatch.setattr(cr, "run_once", lambda *a, **k: (launched.append(1), (0, "", False))[1])
    monkeypatch.setattr(cr, "trigger_seen", lambda o: True)
    monkeypatch.setattr(cr, "read_report", lambda o: {"success": True})
    monkeypatch.setattr(cr, "fill_from_report", lambda row, rep, pred: None)
    status = tmp_path / "campaign_runs.csv"
    code = cr.main(["--manifest", "m", "--dryrun", "d", "--demo-map", "dm", "--campaign-tag", "t",
                    "--outdir-root", str(tmp_path), "--status-csv", str(status)])
    assert code == 0 and len(launched) == 2
