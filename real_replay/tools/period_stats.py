#!/usr/bin/env python3
"""Statistics of a numeric column of real_replay CSV logs. Python 3 standard library only.

usage: period_stats.py [--col NAME] [--include-first] FILE.csv [FILE2.csv ...]

--col period (default): the control-cycle period reported by libfranka, in seconds in the CSV, shown in ms. The first
    row is skipped by default, because libfranka reports 0 on the very first call (same as real_replay's own statistics
    in main.cpp); --include-first keeps it. Prints: cycles, mean, min, max, p99, p99.9 [ms], count and fraction of
    cycles > 1.5 ms and > 4 ms, and whether max <= 4 ms (verdict).
--col NAME (any other column, e.g. cb_us): prints only n, mean, min, max, p99, p99.9 in the unit of the column
    (cb_us is in microseconds), over ALL rows, with no threshold and no verdict.

Percentiles use the nearest-rank method, as main.cpp does: value at index ceil(p*n) - 1 of the sorted data.
Exit code: 0 if no file failed the verdict (always 0 for columns other than "period"), 1 if a "period" file has
max > 4 ms, 2 on read errors.
"""
import csv
import math
import sys

LIMIT_WARN_MS = 1.5
LIMIT_MAX_MS = 4.0


def percentile(sorted_vals, p):
    """Nearest-rank percentile (p in (0, 1]) of an ascending list."""
    k = max(0, min(len(sorted_vals) - 1, int(math.ceil(p * len(sorted_vals))) - 1))
    return sorted_vals[k]


def read_column(path, col_name, scale, skip_first):
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError("empty file")
        if col_name not in header:
            raise ValueError('no column named "%s" (columns: %s ...)' % (col_name, ", ".join(header[:8])))
        col = header.index(col_name)
        vals = []
        for lineno, row in enumerate(reader, start=2):
            if len(row) <= col:
                raise ValueError("line %d: too few fields" % lineno)
            try:
                v = float(row[col])
            except ValueError:
                raise ValueError("line %d: not a number: %r" % (lineno, row[col]))
            vals.append(v * scale)
    if skip_first and vals:
        vals = vals[1:]
    return vals


def report_period(path, vals):
    n = len(vals)
    print("== %s   [column: period, shown in ms]" % path)
    if n == 0:
        print("   no cycles")
        return False
    s = sorted(vals)
    mean = sum(vals) / n
    over_a = sum(1 for v in vals if v > LIMIT_WARN_MS)
    over_b = sum(1 for v in vals if v > LIMIT_MAX_MS)
    mx = s[-1]
    print("   cycles          : %d" % n)
    print("   mean            : %.4f ms" % mean)
    print("   min             : %.4f ms" % s[0])
    print("   max             : %.4f ms" % mx)
    print("   p99             : %.4f ms" % percentile(s, 0.99))
    print("   p99.9           : %.4f ms" % percentile(s, 0.999))
    print("   > %.1f ms        : %d cycles (%.4f %%)" % (LIMIT_WARN_MS, over_a, 100.0 * over_a / n))
    print("   > %.1f ms        : %d cycles (%.4f %%)" % (LIMIT_MAX_MS, over_b, 100.0 * over_b / n))
    ok = mx <= LIMIT_MAX_MS
    print("   max <= %.1f ms   : %s" % (LIMIT_MAX_MS, "YES" if ok else "NO"))
    return ok


def report_generic(path, col_name, vals):
    n = len(vals)
    print("== %s   [column: %s, in the unit of the column]" % (path, col_name))
    if n == 0:
        print("   no rows")
        return
    s = sorted(vals)
    print("   n               : %d" % n)
    print("   mean            : %.6g" % (sum(vals) / n))
    print("   min             : %.6g" % s[0])
    print("   max             : %.6g" % s[-1])
    print("   p99             : %.6g" % percentile(s, 0.99))
    print("   p99.9           : %.6g" % percentile(s, 0.999))


def main(argv):
    include_first = False
    col_name = "period"
    files = []
    i = 1
    while i < len(argv):
        a = argv[i]
        if a == "--include-first":
            include_first = True
        elif a == "--col":
            if i + 1 >= len(argv):
                sys.stderr.write("--col needs a column name\n")
                return 2
            i += 1
            col_name = argv[i]
        elif a in ("-h", "--help"):
            print(__doc__)
            return 0
        elif a.startswith("--"):
            sys.stderr.write("unknown option %s\n" % a)
            return 2
        else:
            files.append(a)
        i += 1
    if not files:
        sys.stderr.write(__doc__)
        return 2
    is_period = col_name == "period"
    all_ok = True
    errors = False
    for path in files:
        try:
            if is_period:
                vals = read_column(path, "period", 1e3, skip_first=not include_first)
            else:
                vals = read_column(path, col_name, 1.0, skip_first=False)
        except (OSError, ValueError) as e:
            sys.stderr.write("error: %s: %s\n" % (path, e))
            errors = True
            continue
        if is_period:
            all_ok = report_period(path, vals) and all_ok
        else:
            report_generic(path, col_name, vals)
    if errors:
        return 2
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
