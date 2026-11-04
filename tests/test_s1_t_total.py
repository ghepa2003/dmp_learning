"""Test puri di T_total per demo (tools/s1_run.py: t_total_for_tau, read_demo_tau_s) e della formula yaw0 = wrap(theta* - omega*T_total)."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import s1_run  # noqa: E402

D1_DIR = Path(__file__).resolve().parents[1] / "runs/20261015_095418_fit_demo_20261014T150851"


def write_params(tmp_path, tau_text):
    (tmp_path / "weights_demo_params.yaml").write_text(
        "format_version: 1\nweights_file: weights.yaml\ndemo:\n"
        f"  tau_s: {tau_text}\n  t_contact_s: {tau_text}\n  contact_to_end_offset_m: [0, 0, 0]\n")
    return tmp_path


def test_d1_gives_exactly_45_2():
    assert s1_run.t_total_for_tau(24.474) == 45.2
    assert s1_run.t_total_for_tau(s1_run.TAU_D1_S) == s1_run.T_TOTAL_S == 45.2


def test_t_total_shifts_with_tau():
    assert s1_run.t_total_for_tau(30.0) == pytest.approx(45.2 + 5.526, abs=1e-12)
    assert s1_run.t_total_for_tau(20.0) == pytest.approx(45.2 - 4.474, abs=1e-12)


@pytest.mark.parametrize("tau,omega,theta_star", [(24.474, -2.0, -163.5), (30.0, -2.0, -163.5), (20.0, -4.0, 145.3)])
def test_yaw0_is_wrap_theta_minus_omega_t_total(tau, omega, theta_star):
    t_total = s1_run.t_total_for_tau(tau)
    expected = s1_run.wrap_deg_half_open(theta_star - omega * (45.2 + (tau - 24.474)))
    assert s1_run.auto_spawn_yaw_deg(omega, theta_star, t_total) == pytest.approx(expected, abs=1e-9)
    assert -180.0 < expected <= 180.0


def test_two_taus_give_different_yaw():
    y1 = s1_run.auto_spawn_yaw_deg(-2.0, -163.5, s1_run.t_total_for_tau(24.474))
    y2 = s1_run.auto_spawn_yaw_deg(-2.0, -163.5, s1_run.t_total_for_tau(30.0))
    assert y1 == pytest.approx(-73.1, abs=1e-9)
    assert y2 == pytest.approx(s1_run.wrap_deg_half_open(-163.5 + 2.0 * (45.2 + 5.526)), abs=1e-9)
    assert abs(y1 - y2) > 1.0


def test_read_demo_tau_s(tmp_path):
    assert s1_run.read_demo_tau_s(write_params(tmp_path, "27.5")) == 27.5


@pytest.mark.parametrize("bad", ["abc", "0", "-3", "nan"])
def test_read_demo_tau_s_rejects_invalid(tmp_path, bad):
    with pytest.raises(s1_run.RunError):
        s1_run.read_demo_tau_s(write_params(tmp_path, bad))


def test_read_demo_tau_s_missing_file(tmp_path):
    with pytest.raises(s1_run.RunError):
        s1_run.read_demo_tau_s(tmp_path)


@pytest.mark.skipif(not (D1_DIR / "weights_demo_params.yaml").is_file(), reason="cartella di D1 assente")
def test_d1_folder_gives_45_2():
    assert s1_run.read_demo_tau_s(D1_DIR) == 24.474
    assert s1_run.t_total_for_tau(s1_run.read_demo_tau_s(D1_DIR)) == 45.2
