"""Write LaTeX table snippets from the result files."""
import json, sys
import pandas as pd
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg.style import METHOD_STYLE

LBL = {k: v["label"] for k, v in METHOD_STYLE.items()}


def f(x, d=1):
    return "--" if x != x else f"{x:.{d}f}"


def flat(x):
    if x < 0.01:
        return "$<$0.01"
    if x < 1:
        return f"{x:.2f}"
    if x < 10:
        return f"{x:.1f}"
    return f"{x:.0f}"


def table_c7():
    S = json.load(open("results/compare_summary.json"))
    lines = []
    for r in S["overall"]:
        lines.append(f"{LBL[r['method']]} & {f(r['collision'])} & {f(r['margin_violation'])} & {f(r['risk_violation'])} & "
                     f"{f(r['min_gap_median'],2)} & {f(r['deadlock'])} & {f(r['delay_ratio'],2)} & "
                     f"{flat(r['lat_mean'])} & {flat(r['lat_p99'])} & {f(r['miss'])} \\\\")
    open("tex/tab_c7_overall.tex", "w").write("\n".join(lines) + "\n")
    lines = []
    for grp in ("ID", "OOD"):
        for r in S[grp]:
            lines.append(f"{'In-distribution' if grp=='ID' else 'Out-of-distribution'} & {LBL[r['method']]} & {f(r['collision'])} & "
                         f"{f(r['margin_violation'])} & {f(r['risk_violation'])} & {f(r['deadlock'])} & {f(r['delay_ratio'],2)} & {f(r['learned_share']) if 'McQueenGuard' in r['method'] else '--'} \\\\")
        lines.append("\\midrule")
    open("tex/tab_c7_idood.tex", "w").write("\n".join(lines[:-1]) + "\n")


def table_c6():
    E = json.load(open("results/embedded_profile.json"))
    L = E["verifier_latency"]
    rows = []
    for k, v in L.items():
        rows.append(f"Learned verifier, {k} & {v['mean_us']:.1f}\\,$\\mu$s & {v['p99_us']:.1f}\\,$\\mu$s & -- \\\\")
    p = E["learned_decision_path"]
    rows.append(f"Complete learned decision path (features, MLP, gate, admission, action) & {p['mean_us']:.1f}\\,$\\mu$s & {p['p99_us']:.1f}\\,$\\mu$s & -- \\\\")
    q = E["prism_latency_ms"]
    rows.append(f"Exact verification, PRISM 4.8.1 (warm server, sparse engine) & {q['mean']:.0f}\\,ms & {q['p99']:.0f}\\,ms & -- \\\\")
    open("tex/tab_c6_latency.tex", "w").write("\n".join(rows) + "\n")


if __name__ == "__main__":
    import os
    os.makedirs("tex", exist_ok=True)
    for fn in sys.argv[1:] or ["c7", "c6"]:
        {"c7": table_c7, "c6": table_c6}[fn]()
