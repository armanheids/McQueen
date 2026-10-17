"""Figures/tables for Comment 6: embedded profile, closed loop under embedded timing, noise-shift study."""
import sys, glob, json, pickle, math
import numpy as np
import pandas as pd
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg import style
from mqg.config import THETA, CATEGORIES
import matplotlib.pyplot as plt

style.setup()


def shift_stream_fig():
    S = json.load(open("results/shift_stream.json"))
    sev = sorted(float(k) for k in S)
    keys = [f"{s:.2f}" for s in sev]
    fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.6))
    ax = axes[0]
    ax.grid(True, axis="both")
    ax.plot(sev, [100 * S[k]["coverage_static"] for k in keys], "o-", color=style.BLUE, label="Static bound coverage")
    ax.plot(sev, [100 * S[k]["familiar_rate"] for k in keys], "D-", color=style.GREEN, label="Familiar (gate passed)")
    ax.axhline(95, color="#333333", ls="--", lw=1.0, label="Nominal 1-α = 95 %")
    ax.set_xlabel("Shift severity s")
    ax.set_ylabel("Rate (%)")
    ax.set_ylim(min(60, ax.get_ylim()[0]), 101)
    ax.legend(loc="lower left", fontsize=6.6)
    ax.set_title("(a) Coverage and familiarity", fontsize=8.5)
    ax = axes[1]
    ax.grid(True, axis="both")
    pols = [("static_nogate", "Static bound, no gate", style.RED, "s-"),
            ("static_gate", "Static bound + gate", style.BLUE, "o-"),
            ("adaptive_gate", "+ online recalibration", style.PURPLE, "^-")]
    for p, lab, col, mk in pols:
        ax.plot(sev, [100 * S[k][p]["unsafe_admission_rate"] for k in keys], mk, color=col, label=lab)
    ax.axhline(100 * 0.05, color="#333333", ls="--", lw=1.0, label="α = 5 %")
    ax.set_xlabel("Shift severity s")
    ax.set_ylabel("Unsafe admissions (% of admitted)")
    ax.set_ylim(0, 9.5)
    ax.legend(loc="upper left", fontsize=6.0, ncol=2)
    ax.set_title("(b) Unsafe-admission rate", fontsize=8.5)
    ax = axes[2]
    ax.grid(True, axis="both")
    for p, lab, col, mk in pols:
        ax.plot(sev, [100 * S[k][p]["admission_rate"] for k in keys], mk, color=col, label=lab)
    ax.set_xlabel("Shift severity s")
    ax.set_ylabel("Learned decisions admitted (%)")
    ax.set_title("(c) Admission rate", fontsize=8.5)
    ax.set_ylim(55, 100)
    ax.legend(loc="lower left", fontsize=6.0)
    for ax in axes:
        ax.set_xticks(sev)
    fig.tight_layout(w_pad=1.2)
    fig.savefig("figures/fig_c6_shift_stream.pdf"); fig.savefig("figures/fig_c6_shift_stream.png")
    plt.close(fig)


def shift_closed():
    rows = []
    for f in sorted(glob.glob("results/shift_closed_*_w*.pkl")):
        for r in pickle.load(open(f, "rb")):
            rows.append(dict(method=r.method, sev=r.severity, col=r.collided, mv=r.margin_violation, to=r.timeout,
                             spd=r.mean_speed * 3.6, learned=r.n_learned, exact=r.n_exact, fb=r.n_fallback_ctrl,
                             lat=r.lat_mean, n_aud=len(r.audits), n_viol=int(np.sum(np.array(r.audits) >= THETA)) if r.audits else 0))
    if not rows:
        return None
    df = pd.DataFrame(rows)
    out = []
    for (m, s), d in df.groupby(["method", "sev"]):
        out.append(dict(method=m, severity=s, episodes=len(d), collision=100 * d.col.mean(), margin_violation=100 * d.mv.mean(),
                        deadlock=100 * d.to.mean(), mean_speed=d.spd.mean(),
                        learned_share=100 * d.learned.sum() / max(1, (d.learned + d.exact + d.fb).sum()),
                        risk_violation=100 * d.n_viol.sum() / max(1, d.n_aud.sum())))
    T = pd.DataFrame(out)
    T.to_csv("results/shift_closed_summary.csv", index=False)
    print(T.to_string())
    order = ["McQueenGuard (static bound)", "McQueenGuard (online recalibration)", "VerifAI runtime assurance",
             "MPC (Zhang et al. 2022)", "Rule-based (IDM + AEB)", "End-to-end RL (PPO)"]
    sevs = sorted(T.severity.unique())
    fig, ax = plt.subplots(figsize=(6.8, 2.8))
    w = 0.8 / len(order)
    for i, m in enumerate(order):
        vals = [float(T[(T.method == m) & (T.severity == s)].margin_violation.iloc[0]) if len(T[(T.method == m) & (T.severity == s)]) else 0 for s in sevs]
        x = np.arange(len(sevs)) + (i - (len(order) - 1) / 2) * w
        st = style.METHOD_STYLE[m]
        ax.bar(x, vals, w * 0.92, color=st["color"], label=st["label"])
    ax.set_xticks(np.arange(len(sevs)))
    ax.set_xticklabels([f"s = {s:.2f}" for s in sevs])
    ax.set_xlabel("Sensor-noise shift severity")
    ax.set_ylabel("Safety violations (% of episodes)")
    style.top_legend(ax, ncol=3)
    fig.savefig("figures/fig_c6_shift_closed.pdf"); fig.savefig("figures/fig_c6_shift_closed.png")
    plt.close(fig)
    return T


def embedded_timing():
    """Desktop vs embedded-timing closed-loop comparison (same scenario instances)."""
    def load(tag):
        rows = []
        for f in sorted(glob.glob(f"results/{tag}_w*.pkl")):
            for r in pickle.load(open(f, "rb")):
                rows.append(dict(method=r.method, category=r.category, seed=r.seed, mv=r.margin_violation, col=r.collided,
                                 miss=r.miss_rate, n_dec=r.n_dec, lat=r.lat_mean, p99=r.lat_p99, to=r.timeout,
                                 spd=r.mean_speed * 3.6))
        return pd.DataFrame(rows)
    E = load("embedded")
    if len(E) == 0:
        return None
    from mqg.results import load_episodes
    F, _, _, _ = load_episodes()
    Dk = pd.DataFrame(dict(method=F.method, category=F.category, seed=F.seed, mv=F.mv, col=F.collided, miss=F.miss,
                           n_dec=F.n_dec, lat=F.lat_mean, to=F.timeout, spd=F.mean_speed))
    E = E[E.method != "McQueenGuard w/o admission (ablation)"]
    keys = set(zip(E.category, E.seed))
    Dk = Dk[[k in keys for k in zip(Dk.category, Dk.seed)]]
    out = []
    for m in E.method.unique():
        for lab, df in (("desktop", Dk), ("embedded", E)):
            d = df[df.method == m]
            if len(d) == 0:
                continue
            out.append(dict(method=m, platform=lab, episodes=len(d), margin_violation=100 * d.mv.mean(),
                            collision=100 * d.col.mean(), deadline_miss=100 * (d.miss * d.n_dec).sum() / max(1, d.n_dec.sum()),
                            lat_mean=float((d.lat * d.n_dec).sum() / max(1, d.n_dec.sum())), deadlock=100 * d.to.mean(),
                            mean_speed=d.spd.mean()))
    T = pd.DataFrame(out)
    T.to_csv("results/embedded_timing_summary.csv", index=False)
    print(T.to_string())
    order = [m for m in ["McQueenGuard", "McQueenGuard (exact only)", "VerifAI runtime assurance",
                         "MPC (Zhang et al. 2022)", "Rule-based (IDM + AEB)", "End-to-end RL (PPO)"] if m in set(T.method)]
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.7))
    for ax, col, ylab in ((axes[0], "deadline_miss", "Deadline misses (% of decisions)"),
                          (axes[1], "margin_violation", "Safety violations (% of episodes)")):
        x = np.arange(len(order))
        dv = [float(T[(T.method == m) & (T.platform == "desktop")][col].iloc[0]) for m in order]
        ev = [float(T[(T.method == m) & (T.platform == "embedded")][col].iloc[0]) for m in order]
        ax.bar(x - 0.18, dv, 0.36, color=style.BLUE, label="Testbed timing")
        ax.bar(x + 0.18, ev, 0.36, color=style.RED, label="Embedded timing (×2.5)")
        ax.set_xticks(x)
        short = {"McQueenGuard": "MQG", "McQueenGuard (exact only)": "MQG\nexact", "VerifAI runtime assurance": "VerifAI\nRA",
                 "MPC (Zhang et al. 2022)": "MPC", "Rule-based (IDM + AEB)": "Rule-\nbased", "End-to-end RL (PPO)": "PPO"}
        ax.set_xticklabels([short[m] for m in order], fontsize=7)
        ax.set_ylabel(ylab)
        style.top_legend(ax, ncol=2)
    fig.tight_layout(w_pad=1.5)
    fig.savefig("figures/fig_c6_embedded_timing.pdf"); fig.savefig("figures/fig_c6_embedded_timing.png")
    plt.close(fig)
    return T


if __name__ == "__main__":
    what = sys.argv[1:] or ["stream", "closed", "embedded"]
    if "stream" in what:
        shift_stream_fig()
    if "closed" in what:
        shift_closed()
    if "embedded" in what:
        embedded_timing()
