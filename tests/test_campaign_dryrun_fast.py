"""Test puri della derivazione dei vincitori A/B in campaign_dryrun.py --mode fast (nessun s3_landscape)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import campaign_dryrun as dr  # noqa: E402


def row(rank, j, v1="1", v2="1", v1p="1", total="10.0", delta="0.0", div="0", err=""):
    return {"rank": str(rank), "J_s": str(j), "k": "0", "branch": "A", "theta_deg": str(rank), "v1_ok": v1, "v2_ok": v2, "v1p_ok": v1p,
            "l2_total_deg": total, "margin_delta_deg": delta, "l2_diverged": div, "error": err}


def test_a_ignores_l2_b_requires_it():
    rows = [row(0, 1.0, total="-2.0"), row(1, 2.0, total="3.0"), row(2, 3.0)]
    wa, wb = dr.derive_winners(rows)
    assert wa["rank"] == "0" and wb["rank"] == "1"


def test_static_constraints_exclude_for_both():
    rows = [row(0, 1.0, v1="0"), row(1, 2.0, v2="0"), row(2, 3.0, v1p=""), row(3, 4.0, err="boom"), row(4, 5.0)]
    wa, wb = dr.derive_winners(rows)
    assert wa["rank"] == "4" and wb["rank"] == "4"


def test_diverged_and_nan_fail_l2():
    rows = [row(0, 1.0, div="1"), row(1, 2.0, total=""), row(2, 3.0, total="nan"), row(3, 4.0, total="0.0", delta="0.0")]
    wa, wb = dr.derive_winners(rows)
    assert wa["rank"] == "0" and wb["rank"] == "3"     # total >= delta: 0.0 >= 0.0 supera


def test_tie_goes_to_lower_rank_and_none_when_empty():
    rows = [row(5, 1.0), row(2, 1.0)]
    assert dr.derive_winners(rows)[0]["rank"] == "2"
    assert dr.derive_winners([]) == (None, None)
    assert dr.derive_winners([row(0, 1.0, v1="0")]) == (None, None)


def test_spawn_yaw_uses_given_t_total():
    r = {"theta_deg": "145.3"}
    assert dr.spawn_yaw(r, -2.0, 45.2) == "-124.300"
    assert dr.spawn_yaw(None, -2.0, 45.2) == ""
