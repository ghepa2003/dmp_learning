#!/usr/bin/env python3
"""
tests/gate_check_build_log.py

Verifica l'output di colcon build in modalita' --clean (_gate_out/clean_build.log).
Requisiti:
  a) file presente, non vuoto, contiene 'Summary: 6 packages finished';
  b) rimozione dei soli avvisi CMake noti di terze parti (FindBoost python310 da eigenpy);
  c) assenza di qualsiasi altra riga contenente 'warning' (case-insensitive);
  d) assenza di qualsiasi blocco '--- stderr: <pkg>' con contenuto non vuoto;
  e) su successo stampa riepilogo con conteggio avvisi rimossi.
"""

import os
import re
import sys

def main() -> int:
    if len(sys.argv) < 2:
        print("Uso: python3 tests/gate_check_build_log.py <file_log>", file=sys.stderr)
        return 1

    log_path = sys.argv[1]
    if not os.path.exists(log_path):
        print(f"ERRORE: File di log '{log_path}' non trovato.", file=sys.stderr)
        return 1

    try:
        with open(log_path, "r", encoding="utf-8", errors="replace") as f:
            text = f.read()
    except Exception as e:
        print(f"ERRORE: Impossibile leggere '{log_path}': {e}", file=sys.stderr)
        return 1

    if not text or not text.strip():
        print("ERRORE: Il file di log e' vuoto.", file=sys.stderr)
        return 1

    # (a) Controllo presenza della riga Summary
    summary_found = False
    for line in text.splitlines():
        if "Summary: 6 packages finished" in line:
            summary_found = True
            break

    if not summary_found:
        print("ERRORE: Riga 'Summary: 6 packages finished' non trovata nel log.", file=sys.stderr)
        return 1

    # (b) Rimozione dei soli paragrafi CMake noti e di terze parti
    pattern = re.compile(
        r"CMake Warning at [^\n]*FindBoost\.cmake:2201 \(message\):\n"
        r"  No header defined for python310; skipping header check \(note: header-only\n"
        r"  libraries have no designated component\)\n"
        r"Call Stack \(most recent call first\):\n"
        r"((?:  [^\n]+\n)+)",
        re.MULTILINE
    )

    removed_count = 0

    def repl(m: re.Match) -> str:
        nonlocal removed_count
        callstack = m.group(1)
        if "eigenpy" in callstack:
            removed_count += 1
            return ""
        return m.group(0)

    cleaned = pattern.sub(repl, text)

    # (c) Controllo assenza di righe contenenti 'warning' (case-insensitive)
    warning_lines = [line for line in cleaned.splitlines() if "warning" in line.lower()]
    if warning_lines:
        print("ERRORE: Rilevati warning nel log di build:", file=sys.stderr)
        for w in warning_lines:
            print(f"  {w}", file=sys.stderr)
        return 1

    # (d) Controllo blocchi stderr non vuoti
    in_stderr = False
    stderr_pkg = ""
    stderr_lines = []
    non_empty_stderr_blocks = []

    for line in cleaned.splitlines():
        if line.startswith("--- stderr: "):
            in_stderr = True
            stderr_pkg = line[len("--- stderr: "):].strip()
            stderr_lines = []
        elif in_stderr:
            if line.strip() == "---" or line.startswith("--- "):
                content = "\n".join(stderr_lines).strip()
                if content:
                    non_empty_stderr_blocks.append((stderr_pkg, content))
                in_stderr = False
            else:
                stderr_lines.append(line)

    if in_stderr:
        content = "\n".join(stderr_lines).strip()
        if content:
            non_empty_stderr_blocks.append((stderr_pkg, content))

    if non_empty_stderr_blocks:
        print("ERRORE: Rilevati blocchi stderr non vuoti nel log:", file=sys.stderr)
        for pkg, content in non_empty_stderr_blocks:
            print(f"--- stderr: {pkg}", file=sys.stderr)
            print(content, file=sys.stderr)
            print("---", file=sys.stderr)
        return 1

    # (e) Messaggio di successo
    print(
        f"BUILD LOG OK: {removed_count} avvisi CMake noti ignorati "
        f"(FindBoost python310, da eigenpy/pinocchio, terze parti); 0 warning del compilatore"
    )
    return 0

if __name__ == "__main__":
    sys.exit(main())
