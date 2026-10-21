#!/usr/bin/env python3
"""
compare_g25.py - Confronta due CSV di output di step3_cells riga per riga e
campo per campo come stringhe, ignorando la sola colonna selection_time_s.
Esce con 0 se identici, 1 altrimenti stampando la prima differenza.
"""

import sys
import csv
from pathlib import Path


def compare_csv(file1_path: str, file2_path: str) -> int:
    p1 = Path(file1_path)
    p2 = Path(file2_path)

    if not p1.exists():
        print(f"ERRORE: File non trovato: {p1}", file=sys.stderr)
        return 1
    if not p2.exists():
        print(f"ERRORE: File non trovato: {p2}", file=sys.stderr)
        return 1

    with open(p1, newline="", encoding="utf-8") as f1, open(p2, newline="", encoding="utf-8") as f2:
        reader1 = list(csv.reader(f1))
        reader2 = list(csv.reader(f2))

    if not reader1 or not reader2:
        print("ERRORE: Uno o entrambi i CSV sono vuoti.", file=sys.stderr)
        return 1

    header1 = reader1[0]
    header2 = reader2[0]

    if header1 != header2:
        print(f"DIFFERENZA HEADER:\n  File 1: {header1}\n  File 2: {header2}", file=sys.stderr)
        return 1

    ignore_col = "selection_time_s"
    if ignore_col in header1:
        ignore_idx = header1.index(ignore_col)
    else:
        ignore_idx = -1

    cell_idx = header1.index("cell") if "cell" in header1 else 0

    if len(reader1) != len(reader2):
        print(f"DIFFERENZA NUMERO RIGHE: {len(reader1)} vs {len(reader2)}", file=sys.stderr)
        return 1

    for row_num, (row1, row2) in enumerate(zip(reader1[1:], reader2[1:]), start=2):
        cell_name = row1[cell_idx] if len(row1) > cell_idx else f"riga_{row_num}"
        if len(row1) != len(row2):
            print(f"DIFFERENZA NUMERO CAMPI riga {row_num} (cella {cell_name}): {len(row1)} vs {len(row2)}", file=sys.stderr)
            return 1
        for col_idx, (val1, val2) in enumerate(zip(row1, row2)):
            if col_idx == ignore_idx:
                continue
            if val1 != val2:
                col_name = header1[col_idx] if col_idx < len(header1) else f"col_{col_idx}"
                print(f"DIFFERENZA: cella={cell_name}, colonna={col_name}, file1='{val1}', file2='{val2}'", file=sys.stderr)
                return 1

    return 0


def main():
    if len(sys.argv) != 3:
        print(f"Uso: {sys.argv[0]} <file1.csv> <file2.csv>", file=sys.stderr)
        sys.exit(1)

    rc = compare_csv(sys.argv[1], sys.argv[2])
    sys.exit(rc)


if __name__ == "__main__":
    main()
