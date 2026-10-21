#!/usr/bin/env python3
"""
tests/gate_check_pytest.py - Verifica stringente dell'output di pytest per gate.sh.
Uso: python3 tests/gate_check_pytest.py <N_atteso>
Legge l'output di pytest da sys.stdin.
Accetta SOLO se il riepilogo finale contiene esattamente '<N_atteso> passed'
e nessun altro contatore (failed, error/errors, skipped, xfailed, xpassed, deselected, warning/warnings, rerun).
"""

import sys
import re


def main():
    if len(sys.argv) != 2:
        sys.stderr.write(f"Uso: {sys.argv[0]} <N_atteso>\n")
        sys.exit(1)

    try:
        expected_passed = int(sys.argv[1])
    except ValueError:
        sys.stderr.write(f"ERRORE: Argomento non valido per N_atteso: '{sys.argv[1]}'\n")
        sys.exit(1)

    content = sys.stdin.read()
    lines = [line.strip() for line in content.splitlines() if line.strip()]
    if not lines:
        sys.stderr.write("ERRORE: Input vuoto, nessun output da pytest.\n")
        sys.exit(1)

    # Cerca la riga di riepilogo finale risalendo dal basso
    summary_line = None
    summary_pattern = re.compile(r'(?:=+\s*)?(.*?)\s+in\s+\d+(?:\.\d+)?s.*')
    for line in reversed(lines):
        m = summary_pattern.match(line)
        if m:
            summary_line = line
            break

    if not summary_line:
        sys.stderr.write("ERRORE: Riga di riepilogo finale di pytest non trovata.\n")
        sys.exit(1)

    # Verifica presenza di contatori non ammessi
    forbidden_pattern = re.compile(
        r'\b(failed|errors?|skipped|xfailed|xpassed|deselected|warnings?|reruns?)\b',
        re.IGNORECASE
    )
    forbidden_matches = forbidden_pattern.findall(summary_line)
    if forbidden_matches:
        distinct = sorted(set(m.lower() for m in forbidden_matches))
        sys.stderr.write(
            f"ERRORE: Rilevati contatori non ammessi nel riepilogo: {distinct}\n"
            f"  Riga riepilogo: '{summary_line}'\n"
        )
        sys.exit(1)

    # Verifica presenza contatore passed
    passed_match = re.search(r'\b(\d+)\s+passed\b', summary_line)
    if not passed_match:
        sys.stderr.write(
            f"ERRORE: Nessun contatore 'passed' trovato nel riepilogo.\n"
            f"  Riga riepilogo: '{summary_line}'\n"
        )
        sys.exit(1)

    actual_passed = int(passed_match.group(1))
    if actual_passed != expected_passed:
        sys.stderr.write(
            f"ERRORE: Test passati ({actual_passed}) diversi da quelli attesi ({expected_passed}).\n"
            f"  Riga riepilogo: '{summary_line}'\n"
        )
        sys.exit(1)

    # Verifica che il blocco di riepilogo contenga esclusivamente '<N> passed'
    m = summary_pattern.match(summary_line)
    summary_body = m.group(1).strip()
    if summary_body != f"{expected_passed} passed":
        sys.stderr.write(
            f"ERRORE: Il riepilogo contiene elementi aggiuntivi imprevisti: '{summary_body}'\n"
            f"  Riga riepilogo: '{summary_line}'\n"
        )
        sys.exit(1)

    print(f"PYTEST OK: {actual_passed} passed")
    sys.exit(0)


if __name__ == "__main__":
    main()
