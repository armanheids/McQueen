"""Reviewer 2, Comments 6 and 7: ablation, large-scale decision/safety preservation, OOD reliability.

Outputs results/r2_summary.json and figures/fig_r2_*.{pdf,png}.
"""
import sys, json, glob, math
import numpy as np
import pandas as pd
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg import style
from mqg.config import THETA, ALPHA_CONF, EPS_MAX, CATEGORIES
from mqg.results import load_episodes, summarise, mcnemar, wilson, ID_CATS, OOD_CATS
from mqg.verifier import LearnedVerifier, admissible_residual, conformal_eps
from mqg.controllers import lexicographic_select
import matplotlib.pyplot as plt

style.setup()
IDEAL = "McQueenGuard, zero-latency exact fallback"
ABL = ["McQueenGuard", "McQueenGuard - conformal margin", "McQueenGuard - familiarity gate",
       "McQueenGuard - exact fallback", "McQueenGuard w/o admission (ablation)", "McQueenGuard - Simplex shield",
       "Learned verifier only - Simplex shield", "McQueenGuard (exact only)", IDEAL]
SHORT = {"McQueenGuard": "Full\nMcQueenGuard", "McQueenGuard - conformal margin": "– conformal\nmargin ε",
         "McQueenGuard - familiarity gate": "– familiarity\ngate H", "McQueenGuard - exact fallback": "– exact\nfallback",
         "McQueenGuard w/o admission (ablation)": "Learned\nonly", "McQueenGuard - Simplex shield": "– Simplex\nshield",
         "Learned verifier only - Simplex shield": "Learned only\n– Simplex", "McQueenGuard (exact only)": "Exact\nonly", IDEAL: "Zero-latency\nfallback*"}
LONG = {"McQueenGuard": "Full McQueenGuard", "McQueenGuard - conformal margin": "-- conformal margin ($\\varepsilon=0$)",
        "McQueenGuard - familiarity gate": "-- familiarity gate $\\mathcal{H}$",
        "McQueenGuard - exact fallback": "-- exact fallback (conservative control instead)",
        "McQueenGuard w/o admission (ablation)": "Learned verifier only (no admission, no fallback)",
        "McQueenGuard - Simplex shield": "-- Simplex shield",
        "Learned verifier only - Simplex shield": "Learned verifier only, -- Simplex shield",
        "McQueenGuard (exact only)": "Exact verification only (no learned verifier)"}


def rows_frame(rows, ver, eps_u):
    recs = []
    m2_all = []
    for r in rows:
        P, U, ph, uh = r["P"], r["U"], r["ph"], r["uh"]
        eps = ver.eps
        aE, okE = lexicographic_select(P, U, 0.0)
        res = float(admissible_residual(P[None], ph[None])[0])
        pol = {}
        for name, e_, gate in (("learned", 0.0, None), ("eps", eps, False), ("gate", 0.0, True), ("full", eps, True)):
            a, ok = lexicographic_select(ph, uh, e_)
            if gate is None:
                adm = True
            else:
                adm = ok and (r["fam"] or not gate) and (not r["sm"]) and e_ <= EPS_MAX
            pol[name] = (adm, a)
        SE = [i for i in range(5) if P[i] < THETA]
        SL = [i for i in range(5) if ph[i] + eps < THETA]
        if len(SE) >= 2:
            us = sorted([U[i] for i in SE], reverse=True)
            T = us[0] - us[1]
        else:
            T = math.inf
        err_u = max(abs(U[i] - uh[i]) for i in SE) if SE else 0.0
        z = ((r["x"] - ver.mu) / ver.sd)[~ver.const] - ver.gate_mean
        m2 = float(z @ ver.gate_prec @ z)
        rec = dict(cat=r["cat"], seed=r["seed"], ood=r["cat"] in OOD_CATS, mode=r["mode"], fam=r["fam"], sm=r["sm"],
                   aE=aE, resid=res, cov=res <= eps, risky=float(P.max()) >= 1e-3, T=T, m=T - 2 * eps_u,
                   err_u=err_u, same_set=(SE == SL), m2=m2, tracked=r["tracked"], d_hat=r["d_hat"],
                   regret_full=np.nan)
        for name, (adm, a) in pol.items():
            rec[f"adm_{name}"] = adm
            rec[f"act_{name}"] = a
            rec[f"fa_{name}"] = adm and P[a] >= THETA
            rec[f"agree_{name}"] = a == aE
        adm, a = pol["full"]
        if adm and okE:
            rec["regret_full"] = float(U[aE] - U[a])
        recs.append(rec)
    return pd.DataFrame(recs)


def main():
    df, lat, aud, rows = load_episodes()
    counts = df.groupby("method").size()
    print(counts)
    ver = LearnedVerifier.load("models/verifier")
    # ------------------------------------------------ utility error bound for Theorem 1 (calibration set)
    D = [np.load(f) for f in sorted(glob.glob("data/d0_*.npz"))]
    X = np.concatenate([d["X"] for d in D]).astype(np.float64)
    P = np.concatenate([d["P"] for d in D])
    Uex = np.concatenate([d["U"] for d in D])
    ep = np.concatenate([d["ep"] for d in D])
    sp = np.load("data/split_ids.npz")
    cal = np.isin(ep, sp["cal"])
    _, Uh = ver.predict(X[cal])
    eps_u = conformal_eps(np.abs(Uex[cal] - Uh).max(1), ALPHA_CONF)
    dd = (Uh[:, :, None] - Uh[:, None, :]) - (Uex[cal][:, :, None] - Uex[cal][:, None, :])
    delta_pair = conformal_eps(np.abs(dd).reshape(len(dd), -1).max(1), ALPHA_CONF)
    R = rows_frame(rows, ver, eps_u)
    R["m_pair"] = R["T"] - delta_pair
    R.to_pickle("results/r2_decisions.pkl")
    consistency = float(np.mean(R.adm_full == (R["mode"] == "learned")))
    print("decision rows", len(R), "full-policy reconstruction consistency", consistency, "eps_u", eps_u)

    out = dict(eps=float(ver.eps), eps_u=float(eps_u), delta_pair=float(delta_pair), n_rows=int(len(R)), reconstruction=consistency,
               episodes_per_method=int(counts.max()), methods=list(map(str, counts.index)))
    # ------------------------------------------------ (1) closed-loop ablation
    abl = summarise(df, lat, ABL)
    abl_id = summarise(df, lat, ABL, ID_CATS)
    abl_ood = summarise(df, lat, ABL, OOD_CATS)
    out["ablation"] = abl
    out["ablation_id"] = abl_id
    out["ablation_ood"] = abl_ood
    out["ablation_mcnemar_mv"] = {m: mcnemar(df, "McQueenGuard", m) for m in ABL[1:]}
    out["ablation_mcnemar_col"] = {m: mcnemar(df, "McQueenGuard", m, "collided") for m in ABL[1:]}
    for r in abl:
        print(f"{r['method'][:42]:42s} col {r['collision']:4.1f} mv {r['margin_violation']:4.1f} dead {r['deadlock']:4.1f} "
              f"risk {r['risk_violation']:4.1f} delay {r['delay_ratio']:.2f} lat {r.get('lat_mean',0):7.2f} miss {r['miss']:4.1f} "
              f"L {r['learned_share']:4.1f} E {r['exact_share']:4.1f} C {r['cons_share']:4.1f}")
    print(out["ablation_mcnemar_mv"])

    # ------------------------------------------------ (2) decision-level preservation (all categories)
    def pol_stats(d, name):
        n = len(d)
        adm = d[f"adm_{name}"]
        na = int(adm.sum())
        fa = int(d[f"fa_{name}"].sum())
        ag = int((adm & d[f"agree_{name}"]).sum())
        return dict(n=n, admitted=na, admission=100 * na / max(1, n), false_adm=fa,
                    false_adm_per_dec=100 * fa / max(1, n), false_adm_per_adm=100 * fa / max(1, na),
                    fa_ci=[100 * x for x in wilson(fa, n)], agree_adm=100 * ag / max(1, na))
    out["policies"] = {}
    for grp, sel in (("all", R), ("ID", R[~R.ood]), ("OOD", R[R.ood]), ("risky", R[R.risky]),
                     ("risky_OOD", R[R.risky & R.ood])):
        out["policies"][grp] = {p: pol_stats(sel, p) for p in ("learned", "eps", "gate", "full")}
    # Theorem 1 at scale (admitted decisions of the full policy)
    A = R[R.adm_full]
    th = {}
    for lab, sel in (("same_set_m_pos", A[A.same_set & (A.m > 0) & np.isfinite(A["T"])]), ("same_set_m_nonpos", A[A.same_set & (A.m <= 0)]),
                     ("different_set", A[~A.same_set])):
        bh = sel.err_u <= eps_u
        th[lab] = dict(n=int(len(sel)), agree=100 * float(sel.agree_full.mean()) if len(sel) else math.nan,
                       bound_holds=100 * float(bh.mean()) if len(sel) else math.nan,
                       agree_given_bound=100 * float(sel[bh].agree_full.mean()) if bh.sum() else math.nan,
                       n_bound=int(bh.sum()),
                       regret_mean=float(sel.regret_full[~sel.agree_full].mean()) if (~sel.agree_full).sum() else 0.0,
                       unsafe=int(sel.fa_full.sum()))
    th["finite_T"] = int(np.isfinite(A["T"]).sum())
    for lab, sel in (("pair_pos", A[A.same_set & (A.m_pair > 0) & np.isfinite(A["T"])]), ("pair_nonpos", A[A.same_set & (A.m_pair <= 0)])):
        th[lab] = dict(n=int(len(sel)), agree=100 * float(sel.agree_full.mean()) if len(sel) else math.nan,
                       regret_mean=float(sel.regret_full[~sel.agree_full].mean()) if (~sel.agree_full).sum() else 0.0)
    orc = A[A.same_set & np.isfinite(A["T"]) & (2 * A.err_u < A["T"])]
    th["oracle_pos"] = dict(n=int(len(orc)), agree=100 * float(orc.agree_full.mean()) if len(orc) else math.nan)
    orn = A[A.same_set & np.isfinite(A["T"]) & (2 * A.err_u >= A["T"])]
    th["oracle_nonpos"] = dict(n=int(len(orn)), agree=100 * float(orn.agree_full.mean()) if len(orn) else math.nan)
    fin = A[A.same_set & np.isfinite(A["T"])]
    th["T_median"] = float(fin["T"].median())
    th["T_p90"] = float(fin["T"].quantile(0.9))
    th["single_admissible"] = int((A.same_set & ~np.isfinite(A["T"])).sum())
    th["single_admissible_agree"] = 100 * float(A[A.same_set & ~np.isfinite(A["T"])].agree_full.mean()) if th["single_admissible"] else math.nan
    out["theorem1"] = th
    out["admitted_total"] = int(len(A))
    out["admitted_agree"] = 100 * float(A.agree_full.mean())
    out["admitted_unsafe"] = int(A.fa_full.sum())
    dis = A[~A.agree_full]
    # disagreements: conservative (learned action decelerates more than exact) vs other
    accs = np.array([1.5, 0.0, -2.0, -4.0, -9.0])
    out["disagree_conservative"] = int((accs[dis.act_full.values] < accs[dis.aE.values]).sum())
    out["disagree_total"] = int(len(dis))
    out["disagree_safe"] = int((~dis.fa_full).sum())
    out["disagree_regret_median"] = float(dis.regret_full.median()) if len(dis) else 0.0
    print(json.dumps(out["theorem1"], indent=0))

    # ------------------------------------------------ (3) OOD reliability per category (R2-C7)
    per = []
    full_ep = df[df.method == "McQueenGuard"]
    for c in CATEGORIES:
        d = R[R.cat == c.name]
        a_ = d[d.adm_full]
        e_ = full_ep[full_ep.category == c.name]
        nd = (e_.n_learned + e_.n_exact + e_.n_fb).sum()
        per.append(dict(category=c.name, ood=not c.in_distribution, n=int(len(d)), n_risky=int(d.risky.sum()),
                        admission=100 * float(d.adm_full.mean()), fallback=100 - 100 * float(d.adm_full.mean()),
                        familiar=100 * float(d.fam.mean()),
                        cov_all=100 * float(d["cov"].mean()), cov_risky=100 * float(d[d.risky]["cov"].mean()) if d.risky.sum() else math.nan,
                        cov_adm=100 * float(a_["cov"].mean()) if len(a_) else math.nan,
                        cov_unfam=100 * float(d[~d.fam]["cov"].mean()) if (~d.fam).sum() else math.nan,
                        fa_dec=100 * float(d.fa_full.mean()), fa_adm=100 * float(a_.fa_full.mean()) if len(a_) else 0.0,
                        n_fa=int(d.fa_full.sum()),
                        fa_dec_nogate=100 * float(d.fa_eps.mean()), fa_dec_noeps=100 * float(d.fa_gate.mean()),
                        fa_dec_learned=100 * float(d.fa_learned.mean()),
                        ep_fallback=100 * float((e_.n_exact + e_.n_fb).sum() / max(1, nd)),
                        ep_collision=100 * float(e_.collided.mean()), ep_mv=100 * float(e_.mv.mean()),
                        ep_dead=100 * float(e_.timeout.mean()), episodes=int(len(e_))))
    out["per_category"] = per
    for p in per:
        print(f"{p['category'][:20]:20s} n {p['n']:5d} adm {p['admission']:5.1f} fam {p['familiar']:5.1f} cov {p['cov_all']:5.1f} "
              f"covR {p['cov_risky']:5.1f} covA {p['cov_adm']:5.1f} faD {p['fa_dec']:4.2f} faA {p['fa_adm']:4.2f} "
              f"| noGate {p['fa_dec_nogate']:4.2f} noEps {p['fa_dec_noeps']:4.2f} learned {p['fa_dec_learned']:4.2f} "
              f"| epFB {p['ep_fallback']:5.1f} col {p['ep_collision']:4.1f} mv {p['ep_mv']:4.1f}")
    # gate as an OOD detector for bound failure
    from sklearn.metrics import roc_auc_score
    y = (~R["cov"]).astype(int)
    out["gate_auc_all"] = float(roc_auc_score(y, R.m2)) if 0 < y.sum() < len(y) else math.nan
    ro = R[R.ood]
    yo = (~ro["cov"]).astype(int)
    out["gate_auc_ood"] = float(roc_auc_score(yo, ro.m2)) if 0 < yo.sum() < len(yo) else math.nan
    out["cov_fam"] = 100 * float(R[R.fam]["cov"].mean())
    out["cov_unfam"] = 100 * float(R[~R.fam]["cov"].mean())
    out["cov_fam_ood"] = 100 * float(ro[ro.fam]["cov"].mean()) if ro.fam.sum() else math.nan
    out["cov_unfam_ood"] = 100 * float(ro[~ro.fam]["cov"].mean()) if (~ro.fam).sum() else math.nan
    json.dump(out, open("results/r2_summary.json", "w"), indent=1, default=float)

    # ================================================ figures
    names = [m for m in ABL if m in set(df.method)]
    # Fig A: ablation (safety violations, deadlocks) ID vs OOD
    fig, axes = plt.subplots(2, 1, figsize=(7.2, 4.9), sharex=True)
    x = np.arange(len(names))
    for ax, col, ylab in ((axes[0], "mv", "Safety violations\n(% of episodes)"), (axes[1], "timeout", "Deadlocks\n(% of episodes)")):
        for k, (lab, cats, color) in enumerate((("In-distribution", ID_CATS, style.BLUE), ("Out-of-distribution", OOD_CATS, style.RED))):
            vals, lo, hi = [], [], []
            for m in names:
                d = df[(df.method == m) & df.category.isin(cats)]
                kk, n = int(d[col].sum()), len(d)
                p = 100 * kk / max(1, n)
                ci = wilson(kk, n)
                vals.append(p); lo.append(p - 100 * ci[0]); hi.append(100 * ci[1] - p)
            ax.bar(x + (k - 0.5) * 0.38, vals, 0.36, color=color, label=lab, yerr=[lo, hi],
                   error_kw=dict(lw=0.5, capsize=1.2, ecolor="#555555"))
        ax.set_xticks(x)
        ax.set_xticklabels([SHORT[m] for m in names], fontsize=7)
        ax.set_ylabel(ylab)
    style.top_legend(axes[0], ncol=2)
    fig.tight_layout(h_pad=0.6)
    fig.savefig("figures/fig_r2_ablation.pdf"); fig.savefig("figures/fig_r2_ablation.png")
    plt.close(fig)

    # Fig B: (a) false-admission rate per decision by admission policy, ID vs OOD; (b) Theorem 1 agreement vs m(x)
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.8))
    ax = axes[0]
    pols = [("learned", "No admission"), ("eps", "ε only"), ("gate", "Gate only"), ("full", "ε + gate (full)")]
    xx = np.arange(len(pols))
    for k, (grp, color) in enumerate((("ID", style.BLUE), ("OOD", style.RED))):
        vals = [out["policies"][grp][p]["false_adm_per_dec"] for p, _ in pols]
        cis = [out["policies"][grp][p]["fa_ci"] for p, _ in pols]
        ax.bar(xx + (k - 0.5) * 0.38, vals, 0.36, color=color, label=("In-distribution" if grp == "ID" else "Out-of-distribution"),
               yerr=[[v - c[0] for v, c in zip(vals, cis)], [c[1] - v for v, c in zip(vals, cis)]],
               error_kw=dict(lw=0.5, capsize=1.2, ecolor="#555555"))
    ax.axhline(100 * ALPHA_CONF, color="#333333", ls="--", lw=1.0, label="α = 5 %")
    ax.set_xticks(xx)
    ax.set_xticklabels([l for _, l in pols], fontsize=7)
    ax.set_ylabel("False admissions (% of decisions)")
    ax.set_title("(a) False admissions by admission policy", fontsize=8.3)
    ax.legend(loc="upper right", fontsize=6.6)
    ax = axes[1]
    B = A[A.same_set & np.isfinite(A["T"])]
    edges = [0, 0.0025, 0.005, 0.01, 0.02, 0.04, 1.0]
    labs, ag, ns = [], [], []
    for lo_, hi_ in zip(edges[:-1], edges[1:]):
        sl = B[(B["T"] > lo_) & (B["T"] <= hi_)] if lo_ > 0 else B[B["T"] <= hi_]
        if len(sl) >= 20:
            labs.append((f"≤ {hi_:g}" if lo_ == 0 else f"{lo_:g}–\n{hi_:g}") if hi_ < 1 else f"> {lo_:g}")
            ag.append(100 * sl.agree_full.mean()); ns.append(len(sl))
    ax.bar(np.arange(len(labs)), ag, 0.62, color=style.BLUE)
    for i, (a_, n_) in enumerate(zip(ag, ns)):
        ax.text(i, a_ + 1.2, f"n={n_:,}", ha="center", va="bottom", fontsize=5.8)
    ax.set_xticks(np.arange(len(labs)))
    ax.set_xticklabels(labs, fontsize=6.4)
    ax.set_ylim(0, 112)
    ax.set_xlabel(r"Utility gap $\mathcal{T}(k)$ between best and second-best admissible action")
    ax.set_ylabel("Agreement with exact action (%)")
    ax.set_title(r"(b) Decision agreement vs. tolerable error ($2\varepsilon_U$=" + f"{2*eps_u:.2f})", fontsize=8.3)
    fig.tight_layout(w_pad=1.4)
    fig.savefig("figures/fig_r2_preservation.pdf"); fig.savefig("figures/fig_r2_preservation.png")
    plt.close(fig)

    # Fig C: per-category coverage and false admissions
    P_ = pd.DataFrame(per)
    fig, axes = plt.subplots(2, 1, figsize=(7.2, 5.0), sharex=True)
    x = np.arange(len(P_))
    short = {"Clear (10-30)": "Clear\n10–30", "Clear (35-50)": "Clear\n35–50", "Fog (10-30)": "Fog\n10–30",
             "Fog (35-50)": "Fog\n35–50", "Rain (10-30)": "Rain\n10–30", "Rain (35-50)": "Rain\n35–50",
             "High speed (50-60)": "High speed\n50–60", "Novel trajectory": "Novel\ntrajectory", "High noise": "High\nnoise",
             "Unseen weather": "Unseen\nweather"}
    ax = axes[0]
    ax.axvspan(5.5, 9.6, color="#F2F2F2", zorder=0)
    ax.bar(x - 0.27, P_.cov_all, 0.26, color=style.BLUE, label="All decisions")
    ax.bar(x, P_.cov_risky, 0.26, color=style.YELLOW, label="Risk-relevant decisions")
    ax.bar(x + 0.27, P_.cov_adm.fillna(0), 0.26, color=style.GREEN, label="Admitted decisions")
    ax.axhline(95, color="#333333", ls="--", lw=1.0, label="1 − α = 95 %")
    ax.set_ylim(60, 101)
    ax.set_ylabel("Coverage of ε (%)")
    ax.set_title("(a) Coverage of the calibrated margin (shaded: out of distribution)", fontsize=8.3, pad=16)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=4, fontsize=6.6, borderaxespad=0.1)
    ax = axes[1]
    ax.axvspan(5.5, 9.6, color="#F2F2F2", zorder=0)
    ax.bar(x - 0.27, P_.fa_dec, 0.26, color=style.BLUE, label="McQueenGuard (margin + familiarity check)")
    ax.bar(x, P_.fa_dec_nogate, 0.26, color=style.ORANGE, label="Without familiarity check")
    ax.bar(x + 0.27, P_.fa_dec_learned, 0.26, color=style.RED, label="Without admission rule")
    ax.axhline(100 * ALPHA_CONF, color="#333333", ls="--", lw=1.0, label="α = 5 %")
    ax.set_ylabel("False admissions\n(% of decisions)")
    ax.set_title("(b) False admissions per category", fontsize=8.3)
    ax.legend(loc="upper left", fontsize=6.4, ncol=2)
    ax.set_xticks(x)
    ax.set_xticklabels([short[c] for c in P_.category], fontsize=7)
    fig.tight_layout(h_pad=1.0)
    fig.savefig("figures/fig_r2_ood.pdf"); fig.savefig("figures/fig_r2_ood.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
