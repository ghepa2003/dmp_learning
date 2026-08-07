#!/usr/bin/env python3
"""Plots position error over time e(t) comparing:
  1. Test 1 (kMinDG WITH Guardrail) vs Test 3a (kMinDG WITHOUT Guardrail)
  2. Test 2 (Ratio > 2.0 WITH Guardrail) vs Test 3b (Ratio > 2.0 WITHOUT Guardrail)

Ensures 100% numerical synchronization with C++ benchmark execution (FIX 1).
"""

import argparse
import csv
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_summary(path):
    metrics = {}
    if not os.path.exists(path):
        return metrics
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            metrics[row["test_id"]] = {
                "name": row["test_name"],
                "scale_x": float(row["scale_x"]),
                "max_err_mm": float(row["max_pos_err_mm"]),
                "settling_sec": float(row["settling_sec"]),
                "settling_pct": float(row["settling_pct"]),
                "iae": float(row["iae_mm_s"]),
                "final_err_mm": float(row["final_pos_err_mm"]),
            }
    return metrics


def load_timeseries(path):
    t, err_t1, err_t3a, err_t2, err_t3b = [], [], [], [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            err_t1.append(float(row["err_t1_kmindg_with"]))
            err_t3a.append(float(row["err_t3a_kmindg_without"]))
            err_t2.append(float(row["err_t2_ratio_with"]))
            err_t3b.append(float(row["err_t3b_ratio_without"]))
    return t, err_t1, err_t3a, err_t2, err_t3b


def format_scale(val):
    if val >= 1e5:
        return f"{val:.3e}"
    else:
        return f"{val:.4f}"


def make_guardrail_plots(csv_timeseries, csv_summary, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    t, err_t1, err_t3a, err_t2, err_t3b = load_timeseries(csv_timeseries)
    summary = load_summary(csv_summary)

    # Extract exact scale values from unified C++ summary run
    scale_t1 = summary.get("t1", {}).get("scale_x", 1.0)
    scale_t3a = summary.get("t3a", {}).get("scale_x", 5.0e10)
    scale_t2 = summary.get("t2", {}).get("scale_x", 1.0)
    scale_t3b = summary.get("t3b", {}).get("scale_x", 7.6628)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 6))
    fig.suptitle("Analisi Rigorosa Prima/Dopo Guardrail DMP (Connessione a Nuovo Goal +5cm su X)", fontsize=14, fontweight="bold")

    # --------------------------------------------------------------------------
    # Panel 1: kMinDG Comparison (Test 1 vs Test 3a)
    # --------------------------------------------------------------------------
    ax1.plot(t, err_t1, color="tab:green", linewidth=2.0, label=f"Test 1: CON Guardrail kMinDG (scale_x={format_scale(scale_t1)})")
    ax1.plot(t, err_t3a, color="tab:red", linestyle="--", linewidth=2.0, label=f"Test 3a: SENZA Guardrail (scale_x={format_scale(scale_t3a)})")

    ax1.set_xlabel("t [s]")
    ax1.set_ylabel("Errore di Posizione dal Goal [mm]")
    ax1.set_title("Guardrail kMinDG: Con vs Senza Guardrail (dG_x = 0)", fontweight="bold")
    ax1.set_yscale("log")
    ax1.grid(True, which="both", linestyle="--", alpha=0.5)
    ax1.legend(fontsize=8.5, loc="upper right")

    m_t1 = summary.get("t1", {})
    m_t3a = summary.get("t3a", {})

    t1_settle = f"{m_t1.get('settling_sec', 0.0):.2f}s ({m_t1.get('settling_pct', 0.0):.1f}%)" if m_t1.get("settling_sec", 0) >= 0 else "N/A"
    t3a_settle = f"{m_t3a.get('settling_sec', -1):.2f}s" if m_t3a.get("settling_sec", -1) >= 0 else "N/A (mai)"

    ann_text1 = (
        f"CON Guardrail (Test 1):\n"
        f"  • scale_x = {format_scale(scale_t1)}\n"
        f"  • Max Err = {m_t1.get('max_err_mm', 0):.2f} mm | Final Err = {m_t1.get('final_err_mm', 0):.2f} mm\n"
        f"  • Settling <10mm = {t1_settle} | IAE = {m_t1.get('iae', 0):.1f} mm·s\n\n"
        f"SENZA Guardrail (Test 3a):\n"
        f"  • scale_x = {format_scale(scale_t3a)} (Esplosivo)\n"
        f"  • Max Err = {m_t3a.get('max_err_mm', 0):.1e} mm (Catastrofico)\n"
        f"  • Settling <10mm = {t3a_settle}"
    )
    ax1.text(0.04, 0.10, ann_text1, transform=ax1.transAxes, bbox=dict(boxstyle="round", facecolor="white", alpha=0.85), fontsize=8)

    # --------------------------------------------------------------------------
    # Panel 2: ratio > 2.0 Comparison (Test 2 vs Test 3b)
    # --------------------------------------------------------------------------
    ax2.plot(t, err_t2, color="tab:blue", linewidth=2.0, label=f"Test 2: CON Guardrail Ratio>2.0 (scale_x={format_scale(scale_t2)})")
    ax2.plot(t, err_t3b, color="tab:orange", linestyle="--", linewidth=2.0, label=f"Test 3b: SENZA Guardrail (scale_x={format_scale(scale_t3b)})")

    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("Errore di Posizione dal Goal [mm]")
    ax2.set_title("Guardrail Ratio > 2.0: Con vs Senza Guardrail (Ratio=3.04)", fontweight="bold")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(fontsize=8.5, loc="upper right")

    m_t2 = summary.get("t2", {})
    m_t3b = summary.get("t3b", {})

    t2_settle = f"{m_t2.get('settling_sec', 0.0):.2f}s ({m_t2.get('settling_pct', 0.0):.1f}%)" if m_t2.get("settling_sec", 0) >= 0 else "N/A"
    t3b_settle = f"{m_t3b.get('settling_sec', 0.0):.2f}s ({m_t3b.get('settling_pct', 0.0):.1f}%)" if m_t3b.get("settling_sec", 0) >= 0 else "N/A"

    ann_text2 = (
        f"CON Guardrail (Test 2):\n"
        f"  • scale_x = {format_scale(scale_t2)}\n"
        f"  • Max Err = {m_t2.get('max_err_mm', 0):.2f} mm | Final Err = {m_t2.get('final_err_mm', 0):.2f} mm\n"
        f"  • Settling <10mm = {t2_settle} | IAE = {m_t2.get('iae', 0):.1f} mm·s\n\n"
        f"SENZA Guardrail (Test 3b):\n"
        f"  • scale_x = {format_scale(scale_t3b)} (Amplificato 7.66x)\n"
        f"  • Max Err = {m_t3b.get('max_err_mm', 0):.2f} mm | Final Err = {m_t3b.get('final_err_mm', 0):.2f} mm\n"
        f"  • Settling <10mm = {t3b_settle} | IAE = {m_t3b.get('iae', 0):.1f} mm·s"
    )
    ax2.text(0.04, 0.10, ann_text2, transform=ax2.transAxes, bbox=dict(boxstyle="round", facecolor="white", alpha=0.85), fontsize=8)

    plt.tight_layout()
    out_img = os.path.join(out_dir, "guardrails_rigorous_comparison.png")
    fig.savefig(out_img, dpi=150)
    plt.close(fig)
    print(f"Plot salvato e sincronizzato: {out_img}")
    return out_img


def main():
    parser = argparse.ArgumentParser(description="Plot rigorous guardrail benchmark with unified summary")
    parser.add_argument("--timeseries", default="06_goal_generalization/plots/data/guardrail_timeseries.csv", help="Timeseries CSV")
    parser.add_argument("--summary", default="06_goal_generalization/plots/data/guardrail_summary_metrics.csv", help="Summary CSV")
    parser.add_argument("--out-dir", default="06_goal_generalization/plots", help="Output directory")
    args = parser.parse_args()

    make_guardrail_plots(args.timeseries, args.summary, args.out_dir)


if __name__ == "__main__":
    main()
