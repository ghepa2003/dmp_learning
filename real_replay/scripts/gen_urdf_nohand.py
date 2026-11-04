#!/usr/bin/env python3
"""Generate real_replay/urdf/fer_nohand.urdf: the FER arm WITHOUT hand/fingers/TCP, from xacro.

Run inside the container with the franka_ws environment sourced (franka_description must be
discoverable through ament). Same xacro arguments as the Gazebo launch
(src/franka_gazebo_overrides/gazebo_cartesian_impedance_control.launch.py, lines 66-75), with
hand=false. The control frame of the real-robot executable is then the flange link `fer_link8`.

  source /opt/ros/humble/setup.bash
  source /root/ros_workspaces/ros2/franka_ws/install/setup.bash
  python3 /root/thesis_ws/real_replay/scripts/gen_urdf_nohand.py

The output is validated: arm links 0..8 present, 7 revolute arm joints, no link/joint related to
hand, fingers or TCP. The script exits non-zero if any check fails.
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET


def build_mappings(arm_id: str, ee_id: str, sim_tags: bool) -> dict:
    # Launch file lines 66-75, with 'hand' forced to 'false'.
    return {
        'arm_id': arm_id,
        'hand': 'false',
        'ros2_control': 'true' if sim_tags else 'false',
        'gazebo': 'true' if sim_tags else 'false',
        'ee_id': ee_id,
        'gazebo_effort': 'true' if sim_tags else 'false',
    }


def validate(urdf_text: str, arm_id: str) -> None:
    root = ET.fromstring(urdf_text)
    links = {l.get('name') for l in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}

    problems = []
    for i in range(0, 9):
        if f'{arm_id}_link{i}' not in links:
            problems.append(f'missing link {arm_id}_link{i}')
    for i in range(1, 8):
        j = joints.get(f'{arm_id}_joint{i}')
        if j is None:
            problems.append(f'missing joint {arm_id}_joint{i}')
        elif j.get('type') != 'revolute':
            problems.append(f'{arm_id}_joint{i} is {j.get("type")}, expected revolute')

    forbidden = ('hand', 'finger', 'tcp')
    for name in sorted(links | set(joints)):
        if name and any(tok in name for tok in forbidden):
            problems.append(f'unexpected hand-related element: {name}')

    moving = [n for n, j in joints.items() if j.get('type') not in ('fixed', None)]
    if len(moving) != 7:
        problems.append(f'expected 7 non-fixed joints, found {len(moving)}: {sorted(moving)}')

    total_mass = 0.0
    for l in root.findall('link'):
        m = l.find('inertial/mass')
        if m is not None:
            total_mass += float(m.get('value'))

    print(f'links: {len(links)}, joints: {len(joints)}, non-fixed joints: {len(moving)}')
    print(f'total mass of links with inertia: {total_mass:.4f} kg')
    for i in range(1, 8):
        j = joints.get(f'{arm_id}_joint{i}')
        lim = j.find('limit') if j is not None else None
        if lim is not None:
            print(f'  {arm_id}_joint{i}: lower {lim.get("lower")}  upper {lim.get("upper")}  '
                  f'effort {lim.get("effort")}  velocity {lim.get("velocity")}')

    if problems:
        print('VALIDATION FAILED:', file=sys.stderr)
        for p in problems:
            print('  - ' + p, file=sys.stderr)
        sys.exit(2)
    print('validation OK')


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(here, '..', 'urdf', 'fer_nohand.urdf'),
                    help='output URDF path (default: real_replay/urdf/fer_nohand.urdf)')
    ap.add_argument('--arm-id', default='fer')
    ap.add_argument('--ee-id', default='franka_hand', help="same value the Gazebo launch passes as 'ee_id'")
    ap.add_argument('--no-sim-tags', action='store_true',
                    help='ros2_control/gazebo/gazebo_effort=false: no <ros2_control>/<gazebo> tags '
                         '(Pinocchio ignores them anyway; use this if the sim packages are not sourced)')
    args = ap.parse_args()

    try:
        import xacro
        from ament_index_python.packages import get_package_share_directory
    except ImportError as exc:
        sys.exit(f'cannot import xacro / ament_index_python ({exc}). Source ROS and franka_ws first.')

    try:
        share = get_package_share_directory('franka_description')
    except Exception as exc:  # PackageNotFoundError
        sys.exit(f'franka_description not found ({exc}). Source /root/ros_workspaces/ros2/franka_ws/install/setup.bash')

    xacro_file = os.path.join(share, 'robots', args.arm_id, args.arm_id + '.urdf.xacro')
    if not os.path.isfile(xacro_file):
        sys.exit(f'xacro file not found: {xacro_file}')

    mappings = build_mappings(args.arm_id, args.ee_id, sim_tags=not args.no_sim_tags)
    print(f'xacro: {xacro_file}')
    print(f'mappings: {mappings}')
    urdf_text = xacro.process_file(xacro_file, mappings=mappings).toxml()

    validate(urdf_text, args.arm_id)

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w', encoding='utf-8') as f:
        f.write(urdf_text)
    print(f'written: {out}')


if __name__ == '__main__':
    main()
