#!/usr/bin/env python3
"""Plot and summarise a real_replay CSV log.

  python3 plot_log.py LOG.csv [--config config/real_replay.yaml] [--out FILE.png] [--show]

Plots: tracking error (translation, rotation), commanded torques against the absolute clamp,
period and success rate, callback duration, joint velocities. The clamp lines and the callback
budget are drawn only if --config is given (needs PyYAML).
"""

import argparse
import os
import sys

import numpy as np


def load(path):
    data = np.genfromtxt(path, delimiter=',', names=True)
    if data.size == 0:
        sys.exit(f'{path}: empty log')
    if data.shape == ():
        data = data.reshape(1)
    return data


def cols(d, base, n=7):
    return np.stack([d[f'{base}{i}'] for i in range(1, n + 1)], axis=1)


def vec3(d, base):
    return np.stack([d[f'{base}_x'], d[f'{base}_y'], d[f'{base}_z']], axis=1)


def read_limits(config_path):
    if not config_path:
        return None, None
    try:
        import yaml
    except ImportError:
        print('PyYAML not available: clamp lines and budget not drawn', file=sys.stderr)
        return None, None
    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    saf = cfg['safety']
    clamp = np.array(saf['effort_limits_nm'], dtype=float) * float(saf['torque_limit_fraction'])
    return clamp, float(saf['callback_budget_us'])


def summarise(d):
    t = d['t']
    e_pos = np.linalg.norm(vec3(d, 'e_pos'), axis=1)
    e_rot = np.linalg.norm(vec3(d, 'e_rot'), axis=1)
    tau = cols(d, 'tau_cmd')
    dq = cols(d, 'dq')
    period = d['period']
    cb = d['cb_us']
    print(f'rows: {len(t)}   trajectory time: {t[-1]:.3f} s')
    print(f'max tracking error:      {e_pos.max() * 1e3:.3f} mm   {e_rot.max():.5f} rad ({np.degrees(e_rot.max()):.3f} deg)')
    print(f'max |tau_cmd|:           {np.abs(tau).max():.3f} Nm   per joint: '
          + ' '.join(f'{v:.2f}' for v in np.abs(tau).max(axis=0)))
    print(f'max |dq|:                {np.abs(dq).max():.4f} rad/s   per joint: '
          + ' '.join(f'{v:.3f}' for v in np.abs(dq).max(axis=0)))
    print(f'min success_rate:        {d["success_rate"].min():.4f}')
    late = int(np.sum(period[1:] > 1.5e-3)) if len(period) > 1 else 0
    print(f'cycles with period > 1.5 ms: {late}   (max period {period.max() * 1e3:.3f} ms)')
    print(f'callback duration [us]:  mean {cb.mean():.1f}  p99 {np.percentile(cb, 99):.1f}  max {cb.max():.1f}')
    flags = d['flags'].astype(int)
    print(f'cycles rate-limited: {int(np.count_nonzero(flags & 1))}   clamped: {int(np.count_nonzero(flags & 2))}   '
          f'in controlled stop: {int(np.count_nonzero(flags & 4))}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv')
    ap.add_argument('--config', help='real_replay.yaml: draws torque clamp and callback budget')
    ap.add_argument('--out', help='PNG path (default: <csv>.png)')
    ap.add_argument('--show', action='store_true', help='open an interactive window instead of only saving')
    args = ap.parse_args()

    import matplotlib
    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    d = load(args.csv)
    summarise(d)

    t = d['t']
    clamp, budget = read_limits(args.config)
    e_pos = np.linalg.norm(vec3(d, 'e_pos'), axis=1) * 1e3
    e_rot = np.linalg.norm(vec3(d, 'e_rot'), axis=1)
    tau = cols(d, 'tau_cmd')
    tau_meas = cols(d, 'tau_J')
    dq = cols(d, 'dq')

    fig, ax = plt.subplots(3, 2, figsize=(15, 11), sharex=True)
    ax = ax.ravel()

    ax[0].plot(t, e_pos)
    ax[0].set_ylabel('translation error [mm]')
    ax[0].set_title('tracking error')

    ax[1].plot(t, e_rot, color='tab:orange')
    ax[1].set_ylabel('rotation error [rad]')
    ax[1].set_title('tracking error')

    for j in range(7):
        line, = ax[2].plot(t, tau[:, j], label=f'tau_cmd{j + 1}')
        ax[2].plot(t, tau_meas[:, j], color=line.get_color(), alpha=0.25, lw=0.8)
        if clamp is not None:
            ax[2].axhline(clamp[j], color=line.get_color(), ls=':', lw=0.8)
            ax[2].axhline(-clamp[j], color=line.get_color(), ls=':', lw=0.8)
    ax[2].set_ylabel('torque [Nm]')
    ax[2].set_title('commanded (solid) / measured (faint) torque' + (', dotted = clamp' if clamp is not None else ''))
    ax[2].legend(ncol=7, fontsize=7, loc='upper right')

    ax[3].plot(t[1:], d['period'][1:] * 1e3, label='period [ms]')
    ax[3].axhline(1.5, color='r', ls='--', lw=0.8, label='1.5 ms')
    ax[3].set_ylabel('period [ms]')
    twin = ax[3].twinx()
    twin.plot(t, d['success_rate'], color='tab:green', lw=0.8, label='success_rate')
    twin.set_ylabel('success rate')
    twin.set_ylim(min(0.0, float(d['success_rate'].min()) - 0.05), 1.05)
    ax[3].set_title('libfranka period and command success rate')
    ax[3].legend(loc='upper left', fontsize=8)

    ax[4].plot(t, d['cb_us'])
    if budget is not None:
        ax[4].axhline(budget, color='r', ls='--', lw=0.8, label=f'budget {budget:.0f} us')
        ax[4].legend(fontsize=8)
    ax[4].set_ylabel('callback [us]')
    ax[4].set_xlabel('trajectory time [s]')
    ax[4].set_title('callback duration')

    for j in range(7):
        ax[5].plot(t, dq[:, j], label=f'dq{j + 1}')
    ax[5].set_ylabel('dq [rad/s]')
    ax[5].set_xlabel('trajectory time [s]')
    ax[5].set_title('joint velocities')
    ax[5].legend(ncol=7, fontsize=7, loc='upper right')

    stop_idx = np.where(d['flags'].astype(int) & 4)[0]
    if len(stop_idx):
        for a in ax:
            a.axvline(t[stop_idx[0]], color='k', ls='-.', lw=0.8)

    for a in ax:
        a.grid(True, alpha=0.3)
    fig.suptitle(os.path.basename(args.csv))
    fig.tight_layout()

    out = args.out or (os.path.splitext(args.csv)[0] + '.png')
    fig.savefig(out, dpi=110)
    print(f'plot written: {out}')
    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
