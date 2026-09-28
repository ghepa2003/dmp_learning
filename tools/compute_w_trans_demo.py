#!/usr/bin/env python3
"""
compute_w_trans_demo.py

Calculates the translational manipulability index w_trans on the final joint configuration
of a recorded demonstration (from a joint_states CSV file produced during live teaching),
and exports the result to a YAML file for downstream normalization in Phase 2 optimization.

Pinocchio kinematics and w_trans computation are copied directly from tools/eval_10_runs.py:
  - Model: fer_flat_effort.urdf
  - Frame: fer_hand_tcp (LOCAL_WORLD_ALIGNED)
  - Index: w_trans = prod(svd(J_v)) = sqrt(det(J_v * J_v^T))

Usage:
  python3 compute_w_trans_demo.py <path_to_joint_states.csv> [optional_urdf_path]
"""

import sys
import os
import csv
import numpy as np
import pinocchio as pin

JOINT_NAMES = [f"fer_joint{i}" for i in range(1, 8)]
DEFAULT_FRAME_NAME = "fer_hand_tcp"


def find_urdf_path(cli_urdf=None):
    candidates = []
    if cli_urdf:
        candidates.append(cli_urdf)
    if "URDF_PATH" in os.environ:
        candidates.append(os.environ["URDF_PATH"])
    candidates.extend([
        "/root/thesis_ws/fer_flat_effort.urdf",
        "/home/lorenzo/thesis_ws/fer_flat_effort.urdf",
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "fer_flat_effort.urdf")
    ])
    for p in candidates:
        if os.path.exists(p):
            return os.path.abspath(p)
    raise FileNotFoundError(f"Could not find fer_flat_effort.urdf. Checked: {candidates}")


def build_q_index(model, joint_names):
    """Resolves each joint name to its Pinocchio q-vector index (model.idx_qs[joint_id]),
    mirroring RobotModel::update's q_index resolution in robot_model.cpp and eval_10_runs.py."""
    return [model.idx_qs[model.getJointId(name)] for name in joint_names]


def pad_q(model, q_index, q):
    """Zero-pads a 7-element arm q into Pinocchio's full model.nq configuration vector."""
    q_full = np.zeros(model.nq)
    for i, idx in enumerate(q_index):
        q_full[idx] = q[i]
    return q_full


def compute_w_trans(model, data, frame_id, q_index, q):
    """Computes translational manipulability index w_trans = prod(svd(J_v)) = sqrt(det(J_v * J_v^T)).
    Copied directly from tools/eval_10_runs.py."""
    q_full = pad_q(model, q_index, q)
    pin.computeJointJacobians(model, data, q_full)
    pin.framesForwardKinematics(model, data, q_full)
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_p = J[:3, :]
    sv = np.linalg.svd(J_p, compute_uv=False)
    return float(np.prod(sv))


def main():
    if len(sys.argv) < 2:
        print("Usage: compute_w_trans_demo.py <path_to_joint_states.csv> [optional_urdf_path]", file=sys.stderr)
        sys.exit(1)

    csv_path = sys.argv[1]
    cli_urdf = sys.argv[2] if len(sys.argv) > 2 else None

    if not os.path.exists(csv_path):
        print(f"Error: CSV file not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    urdf_path = find_urdf_path(cli_urdf)

    # 1. Parse CSV
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            print(f"Error: CSV file is empty or missing header: {csv_path}", file=sys.stderr)
            sys.exit(1)

        # Validate required joint columns
        missing_cols = [jn for jn in JOINT_NAMES if jn not in reader.fieldnames]
        if missing_cols:
            print(f"Error: CSV file missing expected columns {missing_cols}. Found: {reader.fieldnames}", file=sys.stderr)
            sys.exit(1)

        for r in reader:
            if not any(r.values()):
                continue
            rows.append(r)

    if not rows:
        print(f"Error: No data rows found in {csv_path}", file=sys.stderr)
        sys.exit(1)

    # 2. Extract final joint configuration (last row)
    final_row = rows[-1]
    try:
        t_final = float(final_row.get("t", 0.0))
        q_final = np.array([float(final_row[jn]) for jn in JOINT_NAMES])
    except ValueError as e:
        print(f"Error parsing numeric joint values from last row: {e}", file=sys.stderr)
        sys.exit(1)

    # 3. Load Pinocchio model
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()

    if not model.existFrame(DEFAULT_FRAME_NAME):
        print(f"Error: Frame '{DEFAULT_FRAME_NAME}' not found in URDF model {urdf_path}", file=sys.stderr)
        sys.exit(1)

    frame_id = model.getFrameId(DEFAULT_FRAME_NAME)
    for jn in JOINT_NAMES:
        if not model.existJointName(jn):
            print(f"Error: Joint '{jn}' not found in URDF model {urdf_path}", file=sys.stderr)
            sys.exit(1)

    q_index = build_q_index(model, JOINT_NAMES)

    # 4. Compute w_trans
    w_trans_demo = compute_w_trans(model, data, frame_id, q_index, q_final)

    # 5. Output to stdout
    print(f"Loaded demo joint states from : {csv_path}")
    print(f"Samples read                  : {len(rows)}")
    print(f"Final timestamp (t)           : {t_final:.4f} s")
    print(f"Final joint configuration q   : {[round(float(x), 4) for x in q_final]}")
    print(f"w_trans_demo                  : {w_trans_demo:.6f}")

    # 6. Save YAML sidecar
    yaml_path = os.path.splitext(csv_path)[0] + "_w_trans.yaml"
    with open(yaml_path, "w") as yf:
        yf.write("# Translational manipulability w_trans on demo final posture\n")
        yf.write(f"w_trans_demo: {w_trans_demo:.8f}\n")
        yf.write(f"t_final: {t_final:.6f}\n")
        yf.write("q_final:\n")
        for jn, val in zip(JOINT_NAMES, q_final):
            yf.write(f"  {jn}: {val:.8f}\n")

    print(f"Saved w_trans_demo to         : {yaml_path}")


if __name__ == "__main__":
    main()
