"""pytest: coerenza di campaign_A.yaml / campaign_B.yaml con grasp_planner_example.yaml.

- i blocchi cube_geometry.* presenti nei file di campagna sono numericamente uguali (tolleranza 1e-12) a quelli attivi dell'esempio;
  eccezione dichiarata: cube_geometry.grasp_depth_m (valore di campagna 0.0125 misurato in results/val_C0; nell'esempio e' commentato,
  default del nodo 0.0) - qui si controlla solo che A e B coincidano;
- A e B differiscono SOLO per s1.use_dynamic_follow (A false, B true).
"""
import math
from pathlib import Path

import pytest

yaml = pytest.importorskip("yaml")

CONFIG = Path(__file__).resolve().parents[1] / "src/satellite_grasp_planner/config"
TOL = 1e-12
DEPTH_KEY = "cube_geometry.grasp_depth_m"
REQUIRED = ["cube_geometry.cube_side_m", "cube_geometry.standoff_m", "cube_geometry.collar_radius_m",
            "cube_geometry.grasp_radius_m", "cube_geometry.wall_thickness_m"]


def params(name):
    doc = yaml.safe_load((CONFIG / name).read_text())
    return doc["grasp_planner_node"]["ros__parameters"]


def cube_block(p):
    return {k: v for k, v in p.items() if k.startswith("cube_geometry.")}


@pytest.fixture(scope="module")
def cfg():
    return {n: params(n) for n in ("campaign_A.yaml", "campaign_B.yaml", "grasp_planner_example.yaml")}


@pytest.mark.parametrize("name", ["campaign_A.yaml", "campaign_B.yaml"])
def test_cube_geometry_equals_example(cfg, name):
    camp, ex = cube_block(cfg[name]), cube_block(cfg["grasp_planner_example.yaml"])
    for k in REQUIRED:
        assert k in camp, f"{name}: manca {k}"
    for k, v in camp.items():
        if k == DEPTH_KEY:
            continue
        assert k in ex, f"{name}: {k} non e' attivo in grasp_planner_example.yaml"
        if isinstance(v, (list, tuple)):
            assert len(v) == len(ex[k]) and all(abs(float(x) - float(y)) <= TOL for x, y in zip(v, ex[k])), (name, k, v, ex[k])
        else:
            assert math.isclose(float(v), float(ex[k]), rel_tol=0.0, abs_tol=TOL), (name, k, v, ex[k])


def test_cube_geometry_same_in_A_and_B(cfg):
    a, b = cube_block(cfg["campaign_A.yaml"]), cube_block(cfg["campaign_B.yaml"])
    assert a.keys() == b.keys()
    for k in a:
        assert a[k] == b[k], k


def test_A_and_B_differ_only_in_use_dynamic_follow(cfg):
    a, b = cfg["campaign_A.yaml"], cfg["campaign_B.yaml"]
    assert a.keys() == b.keys()
    diff = {k for k in a if a[k] != b[k]}
    assert diff == {"s1.use_dynamic_follow"}, diff
    assert a["s1.use_dynamic_follow"] is False and b["s1.use_dynamic_follow"] is True
