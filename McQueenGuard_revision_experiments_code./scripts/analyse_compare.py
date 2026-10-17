"""R1-C7: comparison of McQueenGuard with VerifAI, MPC, rule-based and end-to-end RL (final paired set)."""
import sys, json
import numpy as np
import pandas as pd
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg.config import CATEGORIES, DT_DEC
from mqg.results import load_episodes, summarise, mcnemar, wilson, ID_CATS, OOD_CATS
from mqg import style
import matplotlib.pyplot as plt

ORDER = ["McQueenGuard", "McQueenGuard (exact only)", "VerifAI runtime assurance", "MPC (Zhang et al. 2022)",
         "Rule-based (IDM + AEB)", "End-to-end RL (PPO)"]
GROUPS = [("Nominal", ID_CATS), ("High speed", ["High speed (50-60)"]), ("Novel trajectory", ["Novel trajectory"]),
          ("High noise", ["High noise"]), ("Unseen weather", ["Unseen weather"])]


def main():
    style.setup()
    df, lat, aud, _ = load_episodes()
    overall = summarise(df, lat, ORDER)
    ID = summarise(df, lat, ORDER, ID_CATS)
    OOD = summarise(df, lat, ORDER, OOD_CATS)
    tests = {m: mcnemar(df, "McQueenGuard", m) for m in ORDER[1:]}
    tests_col = {m: mcnemar(df, "McQueenGuard", m, "collided") for m in ORDER[1:]}
    bycat = []
    for c in CATEGORIES:
        for r in summarise(df, lat, ORDER, [c.name]):
            r["category"] = c.name
            bycat.append(r)
    json.dump(dict(overall=overall, ID=ID, OOD=OOD, mcnemar_mv=tests, mcnemar_col=tests_col, by_category=bycat),
              open("results/c7_summary.json", "w"), indent=1, default=float)
    for r in overall:
        print(f"{r['method'][:30]:30s} n {r['episodes']} col {r['collision']:4.1f} mv {r['margin_violation']:4.1f} risk {r['risk_violation']:4.1f} "
              f"dead {r['deadlock']:4.1f} delay {r['delay_ratio']:.2f} lat {r['lat_mean']:7.2f}/{r['lat_p99']:7.1f} miss {r['miss']:4.1f}")
    print("McNemar mv", tests)
    print("McNemar col", tests_col)

    w = 0.8 / len(ORDER)
    # Fig A: safety violations per scenario group
    fig, ax = plt.subplots(figsize=(6.8, 3.0))
    for i, m in enumerate(ORDER):
        vals, lo, hi = [], [], []
        for g, cats in GROUPS:
            d = df[(df.method == m) & df.category.isin(cats)]
            k, n = int(d.mv.sum()), len(d)
            p = 100 * k / max(1, n)
            ci = wilson(k, n)
            vals.append(p); lo.append(p - 100 * ci[0]); hi.append(100 * ci[1] - p)
        x = np.arange(len(GROUPS)) + (i - (len(ORDER) - 1) / 2) * w
        st = style.METHOD_STYLE[m]
        ax.bar(x, vals, w * 0.92, color=st["color"], label=st["label"], yerr=[lo, hi],
               error_kw=dict(lw=0.6, capsize=1.5, ecolor="#555555"))
    ax.set_xticks(np.arange(len(GROUPS)))
    ax.set_xticklabels([g for g, _ in GROUPS])
    ax.set_ylabel("Safety violations (% of episodes)")
    ax.set_xlabel("Scenario category")
    style.top_legend(ax, ncol=3)
    fig.savefig("figures/fig_c7_safety_by_group.pdf"); fig.savefig("figures/fig_c7_safety_by_group.png")
    plt.close(fig)

    # Fig B: verified risk of executed commands
    fig, ax = plt.subplots(figsize=(6.8, 2.9))
    for i, m in enumerate(ORDER):
        vals = []
        for g, cats in GROUPS:
            d = df[(df.method == m) & df.category.isin(cats)]
            vals.append(100 * d.n_aud_viol.sum() / max(1, d.n_aud.sum()))
        x = np.arange(len(GROUPS)) + (i - (len(ORDER) - 1) / 2) * w
        st = style.METHOD_STYLE[m]
        ax.bar(x, vals, w * 0.92, color=st["color"], label=st["label"])
    ax.set_xticks(np.arange(len(GROUPS)))
    ax.set_xticklabels([g for g, _ in GROUPS])
    ax.set_ylabel(r"Commands with $P_{unsafe}\geq\theta$ (%)")
    ax.set_xlabel("Scenario category")
    style.top_legend(ax, ncol=3)
    fig.savefig("figures/fig_c7_risk.pdf"); fig.savefig("figures/fig_c7_risk.png")
    plt.close(fig)

    # Fig C: decision latency
    fig, ax = plt.subplots(figsize=(6.8, 2.9))
    x = np.arange(len(ORDER))
    means = [np.mean(lat[m]) * 1000 for m in ORDER]
    p99 = [np.percentile(lat[m], 99) * 1000 for m in ORDER]
    ax.bar(x - 0.18, means, 0.36, color=style.BLUE, label="Mean decision latency")
    ax.bar(x + 0.18, p99, 0.36, color=style.RED, label="99th-percentile latency")
    ax.axhline(DT_DEC * 1000, color="#333333", ls="--", lw=1.0, label="Decision period (100 ms)")
    ax.set_yscale("log")
    ax.set_ylabel("Time (ms)")
    ax.set_xticks(x)
    ax.set_xticklabels([style.METHOD_STYLE[m]["label"].replace(" (", "\n(") for m in ORDER], fontsize=7.2)

    def fmt(v):
        return f"{v:.2g}" if v < 10 else f"{v:.0f}"
    for xi, (a, b) in enumerate(zip(means, p99)):
        ax.text(xi - 0.18, a * 1.15, fmt(a), ha="center", va="bottom", fontsize=6.3)
        ax.text(xi + 0.18, b * 1.15, fmt(b), ha="center", va="bottom", fontsize=6.3)
    ax.set_ylim(1e-3, 2e4)
    style.top_legend(ax, ncol=3)
    fig.savefig("figures/fig_c7_latency.pdf"); fig.savefig("figures/fig_c7_latency.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
