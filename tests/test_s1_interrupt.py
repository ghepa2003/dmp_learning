"""Test puri della logica di interruzione (SIGINT/SIGTERM) di s1_run.py / s1_report.py, senza ROS."""
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "tools"))
import s1_report  # noqa: E402
import s1_run  # noqa: E402


def test_interrupted_meta_after_at_end():
    m = s1_run.interrupted_meta({"tag": "x"}, 130.5, 100.0)
    assert m["interrupted"] is True and m["interrupted_rel_s"] == 30.5 and m["tag"] == "x"


def test_interrupted_meta_before_at_end_or_clock():
    assert s1_run.interrupted_meta({}, 50.0, None)["interrupted_rel_s"] is None
    assert s1_run.interrupted_meta({}, None, None)["interrupted_rel_s"] is None


def test_interrupted_never_success():
    assert s1_report.apply_interrupt(True, {}) is True
    assert s1_report.apply_interrupt(True, {"interrupted": False}) is True
    assert s1_report.apply_interrupt(True, {"interrupted": True}) is False
    assert s1_report.apply_interrupt(False, {}) is False


def test_interrupt_line():
    assert s1_report.interrupt_line({}) is None
    assert "run interrotto a +30.5 s" in s1_report.interrupt_line(
        s1_run.interrupted_meta({}, 130.5, 100.0))
    assert "prima di AT_END" in s1_report.interrupt_line(s1_run.interrupted_meta({}, 5.0, None))


def test_exit_code_and_help_text():
    assert s1_run.EXIT_INTERRUPTED == 130
    h = s1_run.make_parser().format_help()
    assert "non serve alcun file STOP" in h
    assert h.count("non serve alcun file STOP") == 2   # --post-s e --keep-alive


def test_close_bag_requires_metadata(tmp_path):
    import signal
    import subprocess

    class P:   # finto Proc: il processo esce subito su SIGINT
        def __init__(self):
            self.p = subprocess.Popen(["true"])
            self.p.wait()
            self.killed = False

        def signal_group(self, sig):
            assert sig == signal.SIGINT

        def kill(self):
            self.killed = True

    try:
        s1_run.close_bag(P(), tmp_path)
        assert False, "doveva fallire senza metadata.yaml"
    except s1_run.RunError as e:
        assert "metadata.yaml" in str(e)
    (tmp_path / "metadata.yaml").write_text("x")
    s1_run.close_bag(P(), tmp_path)
