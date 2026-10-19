#!/usr/bin/env python3
"""
tools/s1_env.py - ordine dei path ROS per i run S1.

Se un pacchetto esiste in due workspace (franka_ws/install e thesis_ws/install) vince il primo di
AMENT_PREFIX_PATH, senza errori. Qui i prefissi sotto <root> (= THESIS_WS/install) passano davanti,
a parita' di ordine relativo; il resto resta invariato.

Funzioni pure (test: tests/test_s1_env.py). Da shell, DOPO il source dell'ambiente:

    eval "$(python3 tools/s1_env.py --root /root/thesis_ws/install)"

stampa gli `export` di AMENT_PREFIX_PATH e LD_LIBRARY_PATH riordinati.
"""
from __future__ import annotations

import argparse
import os
import shlex


def _under(entry: str, root: str) -> bool:
    root = root.rstrip("/")
    e = entry.rstrip("/")
    return e == root or e.startswith(root + "/")


def reorder_path(path: str, root: str) -> str:
    """Voci di `path` (separate da ':') sotto `root` per prime, poi le altre; ordine relativo conservato,
    voci vuote e duplicati eliminati."""
    seen, front, back = set(), [], []
    for e in path.split(":"):
        if not e or e in seen:
            continue
        seen.add(e)
        (front if _under(e, root) else back).append(e)
    return ":".join(front + back)


def ld_library_path(ld_path: str, ament_path_reordered: str, root: str) -> str:
    """LD_LIBRARY_PATH: <prefisso>/lib di ogni prefisso AMENT sotto `root` (nell'ordine dell'AMENT),
    poi le voci di `ld_path` sotto `root`, poi le altre."""
    libs = [e.rstrip("/") + "/lib" for e in ament_path_reordered.split(":") if e and _under(e, root)]
    return reorder_path(":".join(libs + [ld_path]), root)


def exports(env: dict, root: str) -> str:
    ament = reorder_path(env.get("AMENT_PREFIX_PATH", ""), root)
    ld = ld_library_path(env.get("LD_LIBRARY_PATH", ""), ament, root)
    return (f"export AMENT_PREFIX_PATH={shlex.quote(ament)}\n"
            f"export LD_LIBRARY_PATH={shlex.quote(ld)}\n")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--root", required=True, help="THESIS_WS/install")
    a = ap.parse_args(argv)
    print(exports(dict(os.environ), a.root), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
