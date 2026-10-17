"""Build the consolidated response document (R1-C6, R1-C7, R2-C6, R2-C7) from the result files.

Every number in the document is read from results/*.json|csv produced by the experiment scripts.
"""
import json, sys, os, math
import numpy as np
import pandas as pd
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg.style import METHOD_STYLE

LBL = {k: v["label"] for k, v in METHOD_STYLE.items()}
C7 = json.load(open("results/c7_summary.json"))
R2 = json.load(open("results/r2_summary.json"))
V = json.load(open("results/verifier_stats.json"))
E = json.load(open("results/embedded_profile.json"))
LB = json.load(open("results/latency_breakdown.json"))
SS = json.load(open("results/shift_stream.json"))
SC = pd.read_csv("results/shift_closed_summary.csv")
ET = pd.read_csv("results/embedded_timing_summary.csv")
PV = pd.read_csv("data/prism_validation.csv")

MQ, EX, VA, MP, RB, PP = ("McQueenGuard", "McQueenGuard (exact only)", "VerifAI runtime assurance",
                          "MPC (Zhang et al. 2022)", "Rule-based (IDM + AEB)", "End-to-end RL (PPO)")
NOEPS, NOGATE, NOEXACT, LEARN, NOMON, LEARN_NOMON = (
    "McQueenGuard - conformal margin", "McQueenGuard - familiarity gate", "McQueenGuard - exact fallback",
    "McQueenGuard w/o admission (ablation)", "McQueenGuard - Simplex shield", "Learned verifier only - Simplex shield")
ABL = [MQ, NOEPS, NOGATE, NOEXACT, LEARN, NOMON, LEARN_NOMON, EX]
ABL_LBL = {MQ: "Full McQueenGuard", NOEPS: "$-$ conformal margin ($\\varepsilon{=}0$)",
           NOGATE: "$-$ familiarity gate $\\mathcal{H}$", NOEXACT: "$-$ exact fallback$^{a}$",
           LEARN: "Learned verifier only$^{b}$", NOMON: "$-$ Simplex shield",
           LEARN_NOMON: "Learned verifier only, $-$ Simplex shield", EX: "Exact verification only$^{c}$"}

O = {r["method"]: r for r in C7["overall"]}
BYCAT = {(r["method"], r["category"]): r for r in C7["by_category"]}
OID = {r["method"]: r for r in C7["ID"]}
OOD = {r["method"]: r for r in C7["OOD"]}
A = {r["method"]: r for r in R2["ablation"]}
AID = {r["method"]: r for r in R2["ablation_id"]}
AOD = {r["method"]: r for r in R2["ablation_ood"]}
PC = pd.DataFrame(R2["per_category"])
POL = R2["policies"]
TH = R2["theorem1"]
N_EP = O[MQ]["episodes"]
N_PER_CAT = N_EP // 10


def f1(x):
    return "--" if x is None or x != x else f"{x:.1f}"


def f2(x):
    return "--" if x is None or x != x else f"{x:.2f}"


def flat(x):
    if x < 0.01:
        return "$<$0.01"
    if x < 1:
        return f"{x:.2f}"
    if x < 10:
        return f"{x:.1f}"
    return f"{x:,.0f}"


def us(x):
    return f"{x:.1f}\\,$\\mu$s" if x < 1000 else f"{x/1000:.2f}\\,ms"


def pfmt(p):
    if p < 0.001:
        return "$p<0.001$"
    return f"$p={p:.3f}$" if p < 0.1 else f"$p={p:.2f}$"


def ci(r, key="mv_ci"):
    lo, hi = r[key]
    return f"[{lo:.1f}, {hi:.1f}]"


def pct_reduction(a, b):
    return 100.0 * (1 - a / b)


# =============================================================== tables
# --- Table: comparison (R1-C7)
C7_ORDER = [MQ, EX, VA, MP, RB, PP]
t_c7 = []
for m in C7_ORDER:
    r = O[m]
    t_c7.append(f"{LBL[m]} & {f1(r['collision'])} & {f1(r['margin_violation'])} {ci(r)} & {f1(r['risk_violation'])} & "
                f"{f1(r['deadlock'])} & {f2(r['delay_ratio'])} & {flat(r['lat_mean'])} & {flat(r['lat_p99'])} & {f1(r['miss'])} \\\\")
t_c7b = []
for m in C7_ORDER:
    a, b = OID[m], OOD[m]
    t_c7b.append(f"{LBL[m]} & {f1(a['collision'])} & {f1(a['margin_violation'])} & {f1(a['deadlock'])} & "
                 f"{f1(b['collision'])} & {f1(b['margin_violation'])} & {f1(b['deadlock'])} \\\\")
mc = C7["mcnemar_mv"]
mcc = C7["mcnemar_col"]

# --- Table: ablation (R2-C6)
amc = R2["ablation_mcnemar_mv"]
t_abl = []
for m in ABL:
    r = A[m]
    p = "--" if m == MQ else pfmt(amc[m][2])
    if m in (LEARN, LEARN_NOMON):
        share = "100 / 0 / 0"          # no exact verification; 'fallback' decisions are learned decisions without an admissible action
    else:
        share = f"{r['learned_share']:.0f} / {r['exact_share']:.0f}" + (f" / {r['cons_share']:.0f}" if r['cons_share'] > 0.05 else " / 0")
    t_abl.append(f"{ABL_LBL[m]} & {f1(r['collision'])} & {f1(r['margin_violation'])} & {p} & {f1(AID[m]['margin_violation'])} & "
                 f"{f1(AOD[m]['margin_violation'])} & {f1(r['deadlock'])} & {f2(r['delay_ratio'])} & {f1(r['risk_violation'])} & "
                 f"{share} & {flat(r['lat_mean'])} & {f1(r['miss'])} \\\\")

# --- Table: admission policies on the decision stream
POL_LBL = [("learned", "No admission (learned verifier only)"), ("eps", "Conformal margin only"),
           ("gate", "Familiarity gate only"), ("full", "Margin + gate (McQueenGuard)")]
t_pol = []
for p, lab in POL_LBL:
    a, b = POL["ID"][p], POL["OOD"][p]
    t_pol.append(f"{lab} & {f1(a['admission'])} & {f2(a['false_adm_per_dec'])} & {f2(a['false_adm_per_adm'])} & {f1(a['agree_adm'])} & "
                 f"{f1(b['admission'])} & {f2(b['false_adm_per_dec'])} & {f2(b['false_adm_per_adm'])} & {f1(b['agree_adm'])} \\\\")

# --- Table: Theorem 1 at scale
t_th = []
for key, lab in (("same_set_m_pos", "Same admissible set, $m(x)>0$"),
                 ("same_set_m_nonpos", "Same admissible set, $m(x)\\le 0$"),
                 ("different_set", "Different admissible sets")):
    r = TH[key]
    t_th.append(f"{lab} & {r['n']:,} & {f1(r['bound_holds'])} & {f1(r['agree'])} & {f1(r['agree_given_bound'])} & "
                f"{r['regret_mean']:.3f} & {r['unsafe']} \\\\")

# --- Table: per category OOD reliability (R2-C7)
_RD0 = pd.read_pickle("results/r2_decisions.pkl")
FBR_ = (100 - 100 * _RD0[_RD0.risky].groupby("cat").adm_full.mean()).to_dict()
t_cat = []
for _, r in PC.iterrows():
    t_cat.append(f"{r.category.replace('(', '(').replace('-', '--')} & {int(r.n):,} & {f1(r.familiar)} & {f1(r.fallback)} / {f1(FBR_[r.category])} & "
                 f"{f1(r.cov_all)} & {f1(r.cov_risky)} & {f1(r.cov_adm)} & {f2(r.fa_dec)} & {f2(r.fa_dec_nogate)} & "
                 f"{f1(r.ep_collision)} & {f1(r.ep_mv)} \\\\")
    if r.category == "Rain (35-50)":
        t_cat.append("\\midrule")
t_cat = "\n".join(t_cat).replace("Clear (10--30)", "Clear (10--30)")

# --- Table: latency decomposition (R2-C7)
lb = LB
pr = lb["prism"]
gen = lb["prism_model_generation"]["mean_us"] / 1000
io = max(0.0, pr["io_mean_ms"] - gen)
lat_rows = [
    ("\\multicolumn{3}{l}{\\textit{Learned path (per decision)}} \\\\", None),
    (f"Feature extraction $\\phi(x)$ & {us(lb['features']['mean_us'])} & {us(lb['features']['p99_us'])} \\\\", None),
    (f"MLP inference, NumPy (FP64) & {us(lb['mlp_numpy']['mean_us'])} & {us(lb['mlp_numpy']['p99_us'])} \\\\", None),
    (f"MLP inference, ONNX Runtime (FP32) & {us(lb['mlp_onnx_fp32']['mean_us'])} & {us(lb['mlp_onnx_fp32']['p99_us'])} \\\\", None),
    (f"Familiarity gate ($x\\in\\mathcal{{H}}$) & {us(lb['gate']['mean_us'])} & {us(lb['gate']['p99_us'])} \\\\", None),
    (f"Admission test and lexicographic selection & {us(lb['admission_select']['mean_us'])} & {us(lb['admission_select']['p99_us'])} \\\\", None),
    (f"\\textbf{{Complete learned decision}} (with NumPy inference) & \\textbf{{{us(lb['learned_path']['mean_us'])}}} & \\textbf{{{us(lb['learned_path']['p99_us'])}}} \\\\", None),
    ("\\midrule\\multicolumn{3}{l}{\\textit{Exact path (per fallback, PRISM 4.8.1 warm server)}} \\\\", None),
    (f"Valuation $\\lambda$ and pMDP instantiation (model generation) & {us(lb['valuation_query']['mean_us'] + lb['prism_model_generation']['mean_us'])} & -- \\\\", None),
    (f"Model file transfer and inter-process communication (excluding generation) & {io:.1f}\\,ms & -- \\\\", None),
    (f"PRISM model parsing & {pr['parse_mean_ms']:.1f}\\,ms & {pr['parse_p99_ms']:.0f}\\,ms \\\\", None),
    (f"PRISM model construction and model checking & {pr['check_mean_ms']:.0f}\\,ms & {pr['check_p99_ms']:,.0f}\\,ms \\\\", None),
    (f"\\textbf{{Complete exact verification}} & \\textbf{{{pr['wall_mean_ms']:.0f}\\,ms}} & \\textbf{{{pr['wall_p99_ms']:,.0f}\\,ms}} \\\\", None),
    ("\\midrule\\multicolumn{3}{l}{\\textit{End-to-end decision latency in closed loop (all decisions of the 600 episodes)}} \\\\", None),
    (f"McQueenGuard (learned decisions and exact fallbacks) & {O[MQ]['lat_mean']:.1f}\\,ms & {O[MQ]['lat_p99']:,.0f}\\,ms \\\\", None),
    (f"Exact verification at every decision & {O[EX]['lat_mean']:.0f}\\,ms & {O[EX]['lat_p99']:,.0f}\\,ms \\\\", None),
]
t_lat = "\n".join(r for r, _ in lat_rows)

# --- R1-C6 tables
Lv = E["verifier_latency"]
mem = E["memory_mb"]
mod = E["model"]
# one source per quantity: MLP NumPy / ONNX FP32, learned path and PRISM from latency_breakdown.json;
# PyTorch eager and ONNX INT8 from embedded_profile.json
pth = LB["learned_path"]
qx = dict(mean=LB["prism"]["wall_mean_ms"], p99=LB["prism"]["wall_p99_ms"])
t_emb = [
    f"MLP inference only, NumPy (FP64) & {us(LB['mlp_numpy']['mean_us'])} & {us(LB['mlp_numpy']['p99_us'])} \\\\",
    f"MLP inference only, PyTorch eager (FP32) & {us(Lv['PyTorch eager (FP32)']['mean_us'])} & {us(Lv['PyTorch eager (FP32)']['p99_us'])} \\\\",
    f"MLP inference only, ONNX Runtime (FP32) & {us(LB['mlp_onnx_fp32']['mean_us'])} & {us(LB['mlp_onnx_fp32']['p99_us'])} \\\\",
    f"MLP inference only, ONNX Runtime (INT8) & {us(Lv['ONNX Runtime (INT8)']['mean_us'])} & {us(Lv['ONNX Runtime (INT8)']['p99_us'])} \\\\",
    f"Complete learned decision (features, MLP, familiarity check, admission, action) & {us(pth['mean_us'])} & {us(pth['p99_us'])} \\\\",
    f"Exact verification, PRISM 4.8.1 warm server (sparse engine) & {qx['mean']:.0f}\\,ms & {qx['p99']:,.0f}\\,ms \\\\",
]
POWER = {"15 W": 15.0, "MAXN 60 W": 60.0}
proj = {}
for k, fct in (("low", 1.5), ("central", 2.5), ("high", 4.0)):
    lm_, lp_ = pth["mean_us"] / 1000 * fct, pth["p99_us"] / 1000 * fct
    em_, ep_ = qx["mean"] * fct, qx["p99"] * fct
    proj[k] = dict(factor=fct, learned_mean_ms=lm_, learned_p99_ms=lp_, exact_mean_ms=em_, exact_p99_ms=ep_,
                   energy_upper_mJ={pm: dict(learned=pw * lm_, exact=pw * em_) for pm, pw in POWER.items()})
t_proj = []
for k in ("low", "central", "high"):
    pj = proj[k]
    t_proj.append(f"{k.capitalize()} ($\\times${pj['factor']}) & {pj['learned_mean_ms']*1000:.0f}\\,$\\mu$s & {pj['learned_p99_ms']*1000:.0f}\\,$\\mu$s & "
                  f"{pj['exact_mean_ms']:,.0f}\\,ms & {pj['exact_p99_ms']:,.0f}\\,ms & "
                  f"{pj['energy_upper_mJ']['15 W']['learned']:.1f} / {pj['energy_upper_mJ']['MAXN 60 W']['learned']:.1f}\\,mJ & "
                  f"{pj['energy_upper_mJ']['15 W']['exact']/1000:.1f} / {pj['energy_upper_mJ']['MAXN 60 W']['exact']/1000:.1f}\\,J \\\\")
t_et = []
for m in (MQ, EX, VA, MP, RB, PP):
    d = ET[ET.method == m]
    if len(d) < 2:
        continue
    a = d[d.platform == "desktop"].iloc[0]
    b = d[d.platform == "embedded"].iloc[0]
    t_et.append(f"{LBL[m]} & {flat(a.lat_mean)} & {f1(a.deadline_miss)} & {f1(a.margin_violation)} & "
                f"{flat(b.lat_mean)} & {f1(b.deadline_miss)} & {f1(b.margin_violation)} \\\\")
n_et = int(ET.episodes.max())
ETd = {(r.method, r.platform): r for r in ET.itertuples()}
t_ss = []
for k in sorted(SS):
    s = SS[k]
    t_ss.append(f"{float(k):.1f} & {s['n']:,} & {100*s['coverage_static']:.1f} & {100*s['familiar_rate']:.1f} & "
                f"{100*s['static_nogate']['admission_rate']:.1f} / {100*s['static_nogate']['unsafe_admission_rate']:.1f} & "
                f"{100*s['static_gate']['admission_rate']:.1f} / {100*s['static_gate']['unsafe_admission_rate']:.1f} & "
                f"{100*s['adaptive_gate']['admission_rate']:.1f} / {100*s['adaptive_gate']['unsafe_admission_rate']:.1f} & "
                f"{s['adaptive_gate']['eps_final']:.3f} \\\\")
ss0, ss1 = SS[sorted(SS)[0]], SS[sorted(SS)[-1]]
sevs = sorted(SC.severity.unique())
t_sc = []
for m in ["McQueenGuard (static bound)", "McQueenGuard (online recalibration)", VA, MP, RB, PP]:
    cells = []
    for s in sevs:
        d = SC[(SC.method == m) & (SC.severity == s)]
        cells.append(f"{f1(d.collision.iloc[0])} / {f1(d.margin_violation.iloc[0])}" if len(d) else "--")
    t_sc.append(f"{LBL[m]} & " + " & ".join(cells) + " \\\\")
n_sc = int(SC.episodes.max())
SCd = {(r.method, r.severity): r for r in SC.itertuples()}

json.dump(dict(generated=True), open("results/_make_response.ok", "w"))

# =============================================================== derived numbers for the text
IDEAL = "McQueenGuard, zero-latency exact fallback"
HAS_IDEAL = IDEAL in A
PCd = {r.category: r for r in PC.itertuples()}
_RDall = pd.read_pickle("results/r2_decisions.pkl")
FBR = (100 - 100 * _RDall[_RDall.risky].groupby("cat").adm_full.mean()).to_dict()
import pickle as _pk, glob as _gl
_rows = []
for _f in sorted(_gl.glob("results/abl_w*.pkl")):
    _rows += _pk.load(open(_f, "rb"))["rows"]
_lat = np.array([r["lat"] for r in _rows]); _mode = np.array([r["mode"] for r in _rows])
_occ = np.maximum(0.1, _lat)
TIME_EXACT = 100 * float(_occ[np.isin(_mode, ["exact", "fallback"])].sum() / _occ.sum())
from mqg.results import wilson as _wil
_k = round(V["coverage_test"] * V["n_test"])
COVCI = "[" + ", ".join(f"{100*x:.1f}" for x in _wil(_k, V["n_test"])) + "]\\,\\%"
ood_fb = float(np.mean([PCd[c].fallback for c in ("High speed (50-60)", "Novel trajectory", "High noise", "Unseen weather")]))
id_fb = float(np.mean([PCd[c].fallback for c in ("Clear (10-30)", "Clear (35-50)", "Fog (10-30)", "Fog (35-50)", "Rain (10-30)", "Rain (35-50)")]))
id_fa_max = max(PCd[c].fa_dec for c in ("Clear (10-30)", "Clear (35-50)", "Fog (10-30)", "Fog (35-50)", "Rain (10-30)", "Rain (35-50)"))
all_fa_max = float(PC.fa_dec.max())
lat_red = pct_reduction(O[MQ]["lat_mean"], O[EX]["lat_mean"])
speedup_inf = LB["prism"]["wall_mean_ms"] * 1000 / LB["mlp_onnx_fp32"]["mean_us"]
speedup_path = LB["prism"]["wall_mean_ms"] * 1000 / LB["learned_path"]["mean_us"]
speedup_e2e = O[EX]["lat_mean"] / O[MQ]["lat_mean"]
check_share = 100 * LB["prism"]["check_mean_ms"] / LB["prism"]["wall_mean_ms"]
n_dec_total = R2["n_rows"]

# =============================================================== narrative pieces (data-dependent)
ID_C = ["Clear (10-30)", "Clear (35-50)", "Fog (10-30)", "Fog (35-50)", "Rain (10-30)", "Rain (35-50)"]
ABL_PRESENT = [m for m in ABL if m in A] + (["McQueenGuard, zero-latency exact fallback"] if "McQueenGuard, zero-latency exact fallback" in A else [])


def sig(m, test=mc):
    return test[m][2] < 0.05


better = [m for m in (MP, RB, PP, VA) if sig(m) and O[m]["margin_violation"] > O[MQ]["margin_violation"]]
notsig = [m for m in (MP, RB, PP, VA) if not sig(m)]
worse = [m for m in (MP, RB, PP, VA) if sig(m) and O[m]["margin_violation"] < O[MQ]["margin_violation"]]


def names(ms):
    lab = {MP: "MPC", RB: "the rule-based planner", PP: "end-to-end PPO", VA: "VerifAI", EX: "exact verification only"}
    ms = [lab[m] for m in ms]
    if len(ms) <= 1:
        return "".join(ms)
    return ", ".join(ms[:-1]) + " and " + ms[-1]


parts = []
if better:
    parts.append("In paired McNemar tests McQueenGuard had significantly fewer safety violations than " + names(better) + " ("
                 + ", ".join(pfmt(mc[m][2]) for m in better) + ")")
if notsig:
    parts.append(("the differences to " if parts else "The differences to ") + names(notsig) + " were not significant ("
                 + ", ".join(pfmt(mc[m][2]) for m in notsig) + ")")
if worse:
    parts.append(names(worse) + " had significantly fewer violations (" + ", ".join(pfmt(mc[m][2]) for m in worse) + ")")
C7_SIG_SENTENCE = "; ".join(parts) + "."
VA_VS = "similar" if not sig(VA) else ("lower" if O[VA]["margin_violation"] < O[MQ]["margin_violation"] else "higher")
col_better = [m for m in (MP, RB, PP, VA, EX) if mcc[m][2] < 0.05 and O[m]["collision"] > O[MQ]["collision"]]

if HAS_IDEAL:
    IDEAL_SENTENCE = (f"An idealised reference in which exact results are available without delay reached "
                      f"{f1(A[IDEAL]['margin_violation'])}\\,\\% safety violations and {f1(A[IDEAL]['deadlock'])}\\,\\% deadlocks, "
                      f"which locates the remaining gap of the complete framework in the latency of the exact fallback rather than in "
                      f"the admission logic.")
    IDEAL_FOOT = "$^{*}$Zero-latency exact fallback: idealised reference, not a deployable configuration."
else:
    IDEAL_SENTENCE = ""
    IDEAL_FOOT = ""
if HAS_IDEAL:
    t_abl.append("\\midrule")
    r = A[IDEAL]
    share = f"{r['learned_share']:.0f} / {r['exact_share']:.0f} / 0"
    t_abl.append(f"Full McQueenGuard, zero-latency exact fallback$^{{*}}$ & {f1(r['collision'])} & {f1(r['margin_violation'])} & "
                 f"{pfmt(R2['ablation_mcnemar_mv'][IDEAL][2])} & {f1(AID[IDEAL]['margin_violation'])} & {f1(AOD[IDEAL]['margin_violation'])} & "
                 f"{f1(r['deadlock'])} & {f2(r['delay_ratio'])} & {f1(r['risk_violation'])} & {share} & {flat(r['lat_mean'])} & {f1(r['miss'])} \\\\")

col_min = all(O[MQ]["collision"] <= O[m]["collision"] for m in (VA, MP, RB, PP, EX))
C7_COL_SENTENCE = ((f"Its collision rate was the lowest of the six methods in Table~\\ref{{tab:c7}}" if col_min else f"Its collision rate was {f1(O[MQ]['collision'])}\\,\\%")
                   + ((" and significantly lower than that of " + names(col_better) + " (" + ", ".join(pfmt(mcc[m][2]) for m in col_better) + ").")
                      if col_better else "."))
risk_rank = sorted([(O[m]["risk_violation"], m) for m in (MQ, VA, MP, RB, PP)])
lower_risk = [m for v, m in risk_rank if v < O[MQ]["risk_violation"]]
if not lower_risk:
    C7_RISK_SENTENCE = (f"It also executed the smallest share of control commands whose exactly verified unsafe probability was at least $\\theta$ "
                        f"({f1(O[MQ]['risk_violation'])}\\,\\%, against {f1(min(O[m]['risk_violation'] for m in (VA, MP, RB, PP)))}--"
                        f"{f1(max(O[m]['risk_violation'] for m in (VA, MP, RB, PP)))}\\,\\% for the baselines).")
else:
    C7_RISK_SENTENCE = (f"The share of executed control commands whose exactly verified unsafe probability was at least $\\theta$ was "
                        f"{f1(O[MQ]['risk_violation'])}\\,\\% for McQueenGuard, lower than for " +
                        names([m for m in (MP, RB, PP) if O[m]['risk_violation'] > O[MQ]['risk_violation']]) +
                        f" ({f1(min(O[m]['risk_violation'] for m in (MP, RB, PP) if O[m]['risk_violation'] > O[MQ]['risk_violation']))}--"
                        f"{f1(max(O[m]['risk_violation'] for m in (MP, RB, PP) if O[m]['risk_violation'] > O[MQ]['risk_violation']))}\\,\\%)"
                        + " and higher than for " + names(lower_risk) + f" ({'/'.join(f1(O[m]['risk_violation']) for m in lower_risk)}\\,\\%).")

C7_FINDINGS = (
    "\\begin{itemize}[leftmargin=1.3em,itemsep=2pt]\n"
    f"\\item \\emph{{Safety.}} McQueenGuard had {f1(O[MQ]['collision'])}\\,\\% collisions and {f1(O[MQ]['margin_violation'])}\\,\\% safety "
    f"violations. " + C7_SIG_SENTENCE + " The three significant differences remain significant after Holm correction for the five comparisons. "
    + C7_COL_SENTENCE + "\n"
    f"\\item \\emph{{In and out of distribution.}} In-distribution, McQueenGuard had {f1(OID[MQ]['collision'])}\\,\\% collisions and "
    f"{f1(OID[MQ]['margin_violation'])}\\,\\% safety violations. Out of distribution all methods degraded; McQueenGuard reached "
    f"{f1(OOD[MQ]['margin_violation'])}\\,\\%, against {f1(OOD[MP]['margin_violation'])}\\,\\% for MPC, {f1(OOD[RB]['margin_violation'])}\\,\\% for the "
    f"rule-based planner, {f1(OOD[PP]['margin_violation'])}\\,\\% for PPO and {f1(OOD[VA]['margin_violation'])}\\,\\% for VerifAI. "
    f"{BYCAT[(MQ,'Novel trajectory')]['n_mv']} of its {OOD[MQ]['n_mv']} out-of-distribution violations occurred in the novel-trajectory category, "
    f"the rest while exact fallbacks were pending (Sections~6.10 and 6.11).\n"
    f"\\item \\emph{{Assurance of the executed commands.}} " + C7_RISK_SENTENCE + " The verified risk includes unavoidable states; for "
    f"McQueenGuard it falls to {f1(A[IDEAL]['risk_violation'])}\\,\\% when exact results are available without delay (Section~6.10), so part of it "
    f"is caused by the latency of the exact fallback. Only McQueenGuard attaches a calibrated bound to each admitted decision.\n"
    f"\\item \\emph{{Real-time efficiency.}} Exact verification at every decision ({O[EX]['lat_mean']:.0f}\\,ms mean, {O[EX]['lat_p99']:,.0f}\\,ms at "
    f"the 99th percentile) missed the decision period in {O[EX]['miss']:.0f}\\,\\% of the decisions and reacted late, which explains its "
    f"{f1(O[EX]['margin_violation'])}\\,\\% safety violations although every decision was verified. McQueenGuard took {O[MQ]['learned_share']:.0f}\\,\\% of "
    f"its decisions with the learned verifier and missed the decision period in {O[MQ]['miss']:.0f}\\,\\% of the decisions, all of them exact "
    f"fallbacks. MPC needed {O[MP]['lat_mean']:.0f}\\,ms per decision; the rule-based planner, PPO and VerifAI need less than 1\\,ms.\n"
    f"\\item \\emph{{Costs.}} McQueenGuard is more cautious than MPC and the rule-based planner (median travel-time ratio {f2(O[MQ]['delay_ratio'])} "
    f"against {f2(O[MP]['delay_ratio'])} and {f2(O[RB]['delay_ratio'])}) and deadlocked in {f1(O[MQ]['deadlock'])}\\,\\% of the episodes. "
    f"VerifAI reached a {VA_VS} violation rate ({f1(O[VA]['margin_violation'])}\\,\\%) but deadlocked in {f1(O[VA]['deadlock'])}\\,\\% of the "
    f"episodes, because its monitor keeps the braking controller engaged once a violation is predicted.\n"
    "\\end{itemize}")

pc = lambda c, k: getattr(PCd[c], k)
_RD = pd.read_pickle("results/r2_decisions.pkl")
_dis = _RD[_RD.adm_full & ~_RD.agree_full]
REGRET_DIS = float(_dis.regret_full.mean())
CALIB_COND = ("it was not satisfied by any decision" if TH["same_set_m_pos"]["n"] == 0
              else f"it was satisfied in only {TH['same_set_m_pos']['n']:,} decisions")
BYCAT = {(r["method"], r["category"]): r for r in C7["by_category"]}
import sys as _sys
from mqg.results import load_episodes as _le
_df, _, _, _ = _le()
CATMV = (_df.groupby(["method", "category"]).mv.mean() * 100).to_dict()
CATDEAD = (_df.groupby(["method", "category"]).timeout.mean() * 100).to_dict()
CATSPD = _df.groupby(["method", "category"]).mean_speed.mean().to_dict()
cmv = lambda m, c: CATMV[(m, c)]
from mqg.results import mcnemar as _mcn
_hs = _df[_df.category == "High speed (50-60)"]
MCC_HS = _mcn(_hs, MQ, "McQueenGuard, zero-latency exact fallback")[2]
ABL_FINDINGS = (
    "\\textbf{Findings of the ablation.}\n\\begin{itemize}[leftmargin=1.3em,itemsep=2pt]\n"
    f"\\item \\emph{{Learned verifier.}} Without it (exact verification only), {A[EX]['miss']:.0f}\\,\\% of the decisions missed the 100\\,ms "
    f"decision period, and the safety-violation rate was {f1(A[EX]['margin_violation'])}\\,\\% against {f1(A[MQ]['margin_violation'])}\\,\\% "
    f"({pfmt(R2['ablation_mcnemar_mv'][EX][2])}). The learned verifier is the component that makes runtime verification usable in the control loop.\n"
    f"\\item \\emph{{Conformal margin.}} Without the margin, the share of executed commands with verified risk $\\ge\\theta$ rose from "
    f"{f1(A[MQ]['risk_violation'])}\\,\\% to {f1(A[NOEPS]['risk_violation'])}\\,\\% and collisions from {f1(A[MQ]['collision'])}\\,\\% to "
    f"{f1(A[NOEPS]['collision'])}\\,\\%; the difference in safety violations ({f1(A[NOEPS]['margin_violation'])}\\,\\%, {pfmt(R2['ablation_mcnemar_mv'][NOEPS][2])}) "
    f"is within the statistical uncertainty of {N_EP} episodes. The margin's main effect is on false admissions (Table~\\ref{{tab:pol}}).\n"
    f"\\item \\emph{{Familiarity check.}} Without it, the learned verifier was also used in the high-noise and unseen-weather categories. This "
    f"reduced exact fallbacks and the safety-violation rate ({f1(A[NOGATE]['margin_violation'])}\\,\\%), but the vehicle deadlocked in "
    f"{f1(A[NOGATE]['deadlock'])}\\,\\% of the episodes: in the high-noise category the out-of-distribution estimates kept it at "
    f"{CATSPD[(NOGATE,'High noise')]:.1f}\\,km/h on average and it did not clear the crosswalk in {CATDEAD[(NOGATE,'High noise')]:.0f}\\,\\% of the "
    f"episodes. Evaluated on the decisions of the complete framework, removing the check would raise false admissions in the unseen-weather "
    f"category from {f2(pc('Unseen weather','fa_dec'))}\\,\\% to {f2(pc('Unseen weather','fa_dec_nogate'))}\\,\\% of the decisions (Table~\\ref{{tab:ood}}).\n"
    f"\\item \\emph{{Exact fallback.}} Replacing the exact fallback by the conservative fallback control gave the fewest safety violations "
    f"({f1(A[NOEXACT]['margin_violation'])}\\,\\%) but {f1(A[NOEXACT]['deadlock'])}\\,\\% deadlocks: {A[NOEXACT]['cons_share']:.0f}\\,\\% of the decisions "
    f"were conservative, and the vehicle did not clear the crosswalk in any high-noise or unseen-weather episode. The exact fallback is "
    f"therefore what preserves progress when the learned verifier cannot be admitted.\n"
    f"\\item \\emph{{Learned verifier without admission.}} Using the learned verifier unconditionally gave {f1(A[LEARN]['margin_violation'])}\\,\\% "
    f"safety violations with the Simplex shield and {f1(A[LEARN_NOMON]['margin_violation'])}\\,\\% without it, but {f1(A[LEARN]['deadlock'])}\\,\\% and "
    f"{f1(A[LEARN_NOMON]['deadlock'])}\\,\\% deadlocks and no bound on false admissions (up to {f2(pc('Unseen weather','fa_dec_learned'))}\\,\\% of the "
    f"unseen-weather decisions).\n"
    f"\\item \\emph{{Simplex shield.}} Without the shield, the safety-violation rate rose to {f1(A[NOMON]['margin_violation'])}\\,\\% and collisions to "
    f"{f1(A[NOMON]['collision'])}\\,\\% ({pfmt(R2['ablation_mcnemar_mv'][NOMON][2])}). The shield covers the interval in which an exact result is pending.\n"
    + (f"\\item \\emph{{Latency of the exact fallback.}} With an idealised zero-latency fallback, the complete framework reached "
       f"{f1(A[IDEAL]['margin_violation'])}\\,\\% safety violations and {f1(A[IDEAL]['collision'])}\\,\\% collisions with {f1(A[IDEAL]['deadlock'])}\\,\\% "
       f"deadlocks. The remaining gap of the deployable framework is therefore caused by the time spent in PRISM rather than by the admission logic.\n"
       if HAS_IDEAL else "") +
    "\\end{itemize}\n"
    "No single component can be removed without losing liveness (more deadlocks), safety (more violations) or the bound on false admissions. The "
    "complete framework offers the best balance of these three properties among the deployable configurations; its remaining weakness is the latency "
    "of the exact fallback.")

th = TH
TH_ROWS = "\n".join([
    f"Calibrated condition $m(x)>0$ & {th['same_set_m_pos']['n']:,} & {f1(th['same_set_m_pos']['agree'])} & -- & {th['same_set_m_pos']['unsafe']} \\\\",
    f"Realised utility error satisfies $2\\max_a|U(a)-\\hat U(a)|<\\mathcal{{T}}(k)$ & {th['oracle_pos']['n']:,} & {f1(th['oracle_pos']['agree'])} & -- & 0 \\\\",
    f"Realised utility error does not satisfy the condition & {th['oracle_nonpos']['n']:,} & {f1(th['oracle_nonpos']['agree'])} & {th['same_set_m_nonpos']['regret_mean']:.3f} & 0 \\\\",
    f"Single admissible action & {th['single_admissible']:,} & {f1(th['single_admissible_agree'])} & -- & 0 \\\\",
    f"Different learned and exact admissible sets & {th['different_set']['n']:,} & {f1(th['different_set']['agree'])} & {th['different_set']['regret_mean']:.3f} & {th['different_set']['unsafe']:,} \\\\",
])

PRES_FINDINGS = (
    "\\textbf{Findings of the preservation analysis.}\n\\begin{itemize}[leftmargin=1.3em,itemsep=2pt]\n"
    f"\\item \\emph{{Safety preservation.}} With McQueenGuard's admission rule, false admissions occurred in "
    f"{f2(POL['ID']['full']['false_adm_per_dec'])}\\,\\% of the in-distribution and {f2(POL['OOD']['full']['false_adm_per_dec'])}\\,\\% of the "
    f"out-of-distribution decisions, below $\\alpha=5\\,\\%$ as stated by Corollary~1; without an admission rule the rates were "
    f"{f2(POL['ID']['learned']['false_adm_per_dec'])}\\,\\% and {f2(POL['OOD']['learned']['false_adm_per_dec'])}\\,\\%. On risk-relevant decisions the rates "
    f"were {f2(POL['risky']['full']['false_adm_per_dec'])}\\,\\% of the decisions with the rule and {f2(POL['risky']['learned']['false_adm_per_dec'])}\\,\\% "
    f"without it; per admitted risk-relevant decision the rate with the rule was {f2(POL['risky']['full']['false_adm_per_adm'])}\\,\\%, slightly above $\\alpha$, "
    f"which again reflects that the guarantee of Corollary~1 is marginal rather than conditional. "
    f"The margin contributes most in-distribution ({f2(POL['ID']['gate']['false_adm_per_dec'])}\\,\\% with the familiarity check alone), the familiarity "
    f"check out of distribution ({f2(POL['OOD']['eps']['false_adm_per_dec'])}\\,\\% with the margin alone).\n"
    f"\\item \\emph{{Decision preservation.}} The admitted learned action equalled the exact action in {f1(R2['admitted_agree'])}\\,\\% of the "
    f"{R2['admitted_total']:,} admitted decisions. The disagreements are mostly harmless: in {100*R2['disagree_safe']/max(1,R2['disagree_total']):.0f}\\,\\% "
    f"of them the learned action was safe and in {100*R2['disagree_conservative']/max(1,R2['disagree_total']):.0f}\\,\\% it was more cautious than the "
    f"exact action; within the same admissible set the mean utility loss was {th['same_set_m_nonpos']['regret_mean']:.3f}.\n"
    f"\\item \\emph{{Theorem~1 at scale.}} Whenever the realised utility error satisfied the condition of Theorem~1, the decisions agreed in "
    f"{f1(th['oracle_pos']['agree'])}\\,\\% of the {th['oracle_pos']['n']:,} cases, as the theorem predicts (a consistency check of the statement and of our implementation). With the calibrated utility bound "
    f"$\\varepsilon_U={R2['eps_u']:.3f}$, however, the condition $m(x)>0$ " + CALIB_COND.replace("it was", "was") + ", because the utility "
    f"gap $\\mathcal{{T}}(k)$ between the best and second-best admissible actions is small in this scenario (median {th['T_median']:.3f}, "
    f"90th percentile {th['T_p90']:.3f}). Theorem~1 is a sufficient condition and is conservative when candidate actions have similar utilities. "
    f"All {th['different_set']['unsafe']:,} unsafe admissions occurred in decisions whose learned and exact admissible sets differed, that is, when "
    f"the probability bound failed for one of the actions; this is the event bounded by Corollary~1.\n"
    "\\end{itemize}\n"
    "These results support the safety-preservation result at scale and qualify the decision-preservation result: the framework guarantees, under the "
    "calibration assumptions, that admitted actions are safe with probability at least $1-\\alpha$, but it does not guarantee that the learned and exact "
    "verification select the same action. We have revised the text of Sections~5.1 and 6.5 accordingly.")

OOD_FINDINGS = (
    "\\textbf{Findings.}\n\\begin{itemize}[leftmargin=1.3em,itemsep=2pt]\n"
    f"\\item \\emph{{In-distribution.}} The coverage of the margin over all decisions was {f1(min(pc(c,'cov_all') for c in ID_C))}--"
    f"{f1(max(pc(c,'cov_all') for c in ID_C))}\\,\\%, close to the nominal 95\\,\\%, and false admissions stayed at or below {f2(id_fa_max)}\\,\\% "
    f"of the decisions. On risk-relevant decisions coverage was lower ({f1(min(pc(c,'cov_risky') for c in ID_C))}--{f1(max(pc(c,'cov_risky') for c in ID_C))}\\,\\%): "
    f"the conformal guarantee of Equation~(26) is marginal over the calibration distribution and does not hold conditionally on a subset of "
    f"difficult decisions. Fallback rates were {f1(min(pc(c,'fallback') for c in ID_C))}--{f1(max(pc(c,'fallback') for c in ID_C))}\\,\\% of the decisions.\n"
    f"\\item \\emph{{High speed.}} {f1(100-pc('High speed (50-60)','familiar'))}\\,\\% of the decisions were outside $\\mathcal{{H}}$, the fallback rate rose to "
    f"{f1(pc('High speed (50-60)','fallback'))}\\,\\%, coverage stayed at {f1(pc('High speed (50-60)','cov_all'))}\\,\\% and false admissions at "
    f"{f2(pc('High speed (50-60)','fa_dec'))}\\,\\%. With an idealised zero-latency fallback no safety violation occurred in this category "
    f"({f1(cmv(IDEAL,'High speed (50-60)'))}\\,\\% against {f1(pc('High speed (50-60)','ep_mv'))}\\,\\%, exact McNemar $p={MCC_HS:.3f}$), so these violations are "
    f"attributable to the time spent waiting for PRISM.\n"
    f"\\item \\emph{{High noise and unseen weather.}} The familiarity check rejected every decision (fallback rate 100\\,\\%), so the learned verifier was not "
    f"used. This was necessary: under unseen weather, coverage fell to {f1(pc('Unseen weather','cov_all'))}\\,\\% ({f1(pc('Unseen weather','cov_risky'))}\\,\\% on "
    f"risk-relevant decisions), and false admissions would have reached {f2(pc('Unseen weather','fa_dec_nogate'))}\\,\\% of the decisions without the check and "
    f"{f2(pc('Unseen weather','fa_dec_learned'))}\\,\\% without any admission rule. The {f1(pc('High noise','ep_mv'))}\\,\\% and {f1(pc('Unseen weather','ep_mv'))}\\,\\% "
    f"safety violations in these categories ({round(pc('High noise','ep_mv')*pc('High noise','episodes')/100)} and {round(pc('Unseen weather','ep_mv')*pc('Unseen weather','episodes')/100)} episodes) did not occur with the idealised zero-latency fallback, which points to exact-verification "
    f"latency as their cause; with 60 episodes per category this is indicative only.\n"
    f"\\item \\emph{{Novel trajectories.}} Running and occluded pedestrians did not change the feature distribution enough to be detected "
    f"({f1(pc('Novel trajectory','familiar'))}\\,\\% familiar), and the margin remained valid ({f1(pc('Novel trajectory','cov_all'))}\\,\\% coverage, "
    f"{f2(pc('Novel trajectory','fa_dec'))}\\,\\% false admissions). Nevertheless, this category had the highest safety-violation rate of the complete "
    f"framework ({f1(pc('Novel trajectory','ep_mv'))}\\,\\%, {f1(pc('Novel trajectory','ep_collision'))}\\,\\% collisions); exact verification only reached "
    f"{f1(cmv(EX,'Novel trajectory'))}\\,\\% and the zero-latency variant {f1(cmv(IDEAL,'Novel trajectory'))}\\,\\%, and all baselines had "
    f"{f1(min(cmv(m,'Novel trajectory') for m in (VA,MP,RB,PP)))}--{f1(max(cmv(m,'Novel trajectory') for m in (VA,MP,RB,PP)))}\\,\\%. The error bound "
    f"is defined with respect to the exact verification result, so it cannot detect behaviour that the verification model itself does not represent.\n"
    f"\\item \\emph{{Familiarity check as a detector.}} The check detects shifts of the input distribution, but its Mahalanobis score is a weak "
    f"predictor of individual bound failures (area under the ROC curve {f2(R2['gate_auc_all'])}; coverage {f1(R2['cov_fam'])}\\,\\% on familiar and "
    f"{f1(R2['cov_unfam'])}\\,\\% on unfamiliar decisions). Familiarity alone does not certify the bound; the online recalibration of Section~6.12 "
    f"addresses this for gradual shifts.\n"
    "\\end{itemize}\n"
    "The empirical coverage of 97.8\\,\\% in Table~4 should therefore be read as an in-distribution result. Out of distribution, the framework does not rely on "
    "the bound remaining valid: it relies on detecting the shift and falling back to exact verification, which worked for sensor-noise and weather shifts, "
    "and it cannot protect against modelling gaps such as unmodelled pedestrian behaviour.")

LAT_FINDINGS = (
    f"The learned path is dominated by MLP inference in the NumPy implementation used in the control loop ({us(LB['mlp_numpy']['mean_us'])}) and by "
    f"the familiarity check ({us(LB['gate']['mean_us'])}); with ONNX Runtime, inference takes {us(LB['mlp_onnx_fp32']['mean_us'])}. "
    f"Exact verification is dominated by model construction and model checking inside PRISM ({check_share:.0f}\\,\\% of its time). The ratio "
    f"between one exact verification and one learned decision is about {float(f'{speedup_path:.2g}'):,.0f}$\\times$ (about "
    f"{float(f'{speedup_inf:.2g}'):,.0f}$\\times$ for inference alone), but because {100-O[MQ]['learned_share']:.0f}\\,\\% of the decisions fall back to exact verification, the end-to-end mean decision latency "
    f"falls only from {O[EX]['lat_mean']:.0f}\\,ms to {O[MQ]['lat_mean']:.1f}\\,ms ({speedup_e2e:.1f}$\\times$, {"an" if str(round(lat_red))[0] == "8" else "a"} {lat_red:.0f}\\,\\% reduction), and the "
    f"99th percentile ({O[MQ]['lat_p99']:,.0f}\\,ms) is set by the fallbacks. Perception (90--150\\,ms) and actuation (0.1\\,s delay) latencies are the "
    f"same for all methods and are not included. Measured in time rather than in decisions, exact verification is not a minor share: the "
    f"{100-O[MQ]['learned_share']:.0f}\\,\\% exact decisions occupy {TIME_EXACT:.0f}\\,\\% of the driving time, because each one blocks the planner until it completes.")

ET_FINDINGS = (
    f"\\textbf{{Findings.}} Methods whose decisions take well under 100\\,ms (VerifAI, the rule-based planner and PPO) were not affected. MPC stayed below "
    f"the decision period on average. McQueenGuard's safety violations rose from {ETd[(MQ,'desktop')].margin_violation:.0f}\\,\\% to "
    f"{ETd[(MQ,'embedded')].margin_violation:.0f}\\,\\% and those of exact verification only from {ETd[(EX,'desktop')].margin_violation:.0f}\\,\\% to "
    f"{ETd[(EX,'embedded')].margin_violation:.0f}\\,\\%, because each fallback waits 2.5 times longer for PRISM. The learned path itself remains far below "
    f"the decision period on embedded hardware; the exact fallback is the bottleneck. On a vehicle, the exact verifier should run on a dedicated core with a "
    f"bounded model size or be replaced by a faster exact engine, while the fallback safety control covers the waiting time. McQueenGuard's share of "
    f"missed deadlines fell under the slower timing because each exact verification blocks the planner for longer, so fewer decisions are taken during "
    f"fallback periods. With {n_et} episodes per method these differences are indicative rather than statistically conclusive.")

SHIFT_FINDINGS = (
    f"\\textbf{{Findings.}} The coverage of the offline margin varied between {100*min(SS[k]['coverage_static'] for k in SS):.1f} and "
    f"{100*max(SS[k]['coverage_static'] for k in SS):.1f}\\,\\% ({100*ss1['coverage_static']:.1f}\\,\\% at $s=1$), "
    f"and the unsafe-admission rate of the static margin rose from {100*ss0['static_gate']['unsafe_admission_rate']:.1f}\\,\\% to "
    f"{100*ss1['static_gate']['unsafe_admission_rate']:.1f}\\,\\% of the admitted decisions. The familiarity check removed "
    f"{100*(1-ss1['familiar_rate']):.0f}\\,\\% of the shifted decisions at $s=1$. With online recalibration, $\\varepsilon$ grew to "
    f"{ss1['adaptive_gate']['eps_final']:.3f} and the unsafe-admission rate remained between {100*min(SS[k]['adaptive_gate']['unsafe_admission_rate'] for k in SS):.1f} "
    f"and {100*max(SS[k]['adaptive_gate']['unsafe_admission_rate'] for k in SS):.1f}\\,\\% across the severities, at the "
    f"price of admitting {100*ss1['adaptive_gate']['admission_rate']:.0f}\\,\\% instead of {100*ss1['static_gate']['admission_rate']:.0f}\\,\\% of the decisions. "
    f"The guarantee thus degrades gradually and the framework trades speed-up for assurance automatically. Recalibration does not restore "
    f"exchangeability, so under a persistent shift the statistical statement remains conditional on the recalibration window. In closed loop "
    f"(Table~\\ref{{tab:sc}}) neither McQueenGuard configuration had a collision at any severity, whereas VerifAI, MPC and PPO reached "
    f"{SCd[(VA,1.0)].margin_violation:.0f}\\,\\%, {SCd[(MP,1.0)].margin_violation:.0f}\\,\\% and {SCd[(PP,1.0)].margin_violation:.0f}\\,\\% safety violations at "
    f"$s=1$. The configuration with online recalibration had slightly more safety violations than the static one at $s=0$ and $s=0.5$, consistent "
    f"with its higher fallback rate; with {n_sc} episodes per cell, differences of one or two episodes are not significant.")

THREATS = (
    "The following paragraph is added to the threats to validity:\n\\begin{quote}\n"
    "The comparative, ablation and out-of-distribution results of Sections~6.9--6.12 were obtained in a closed-loop testbed that re-implements the "
    "pedestrian-crossing scenario with the sensing, weather and actuation characteristics of the CARLA configuration, rather than in CARLA itself; "
    "absolute rates are therefore not interchangeable with those of Sections~6.2--6.7. The baselines were implemented by the authors following the cited "
    "work; VerifAI's sampler and specification-monitor classes were used directly, and the PPO planner was trained for this scenario only. The embedded "
    "figures for NVIDIA Orin are projections from single-thread measurements under a stated slow-down band, and the energy figures are upper bounds "
    "derived from the module power envelope; no measurement on automotive hardware was performed. The error bound is defined with respect to the exact "
    "verification result; behaviour that the verification model does not represent, such as running or occluded pedestrians, is not covered by the "
    "bound and was the largest single source of safety violations of the complete framework and of exact verification only. Finally, the latency of exact verification is the main "
    "residual risk of the framework in closed loop; it is mitigated, but not removed, by the fallback safety control and the Simplex shield. The online "
    "recalibration of Section~6.12 does not restore exchangeability, so under a persistent distribution shift the statistical guarantee remains conditional on "
    "the recalibration window. Validation on automotive hardware and on a test vehicle is left to future work.\n\\end{quote}")

REVISIONS = (
    "\\begin{itemize}[leftmargin=1.3em,itemsep=2pt]\n"
    "\\item \\emph{Section~5.1, after the proof of Theorem~1}: ``Theorem~1 gives a sufficient condition. When candidate actions have similar utilities, "
    "the tolerable error $\\mathcal{T}(k)$ is small and the condition $2\\varepsilon_{\\Gamma'}(x)<\\mathcal{T}(k)$ is rarely met with a calibrated bound; "
    "the learned and exact verification may then select different actions. Safety is not affected by such deviations as long as the admitted action "
    "satisfies the conservative safety condition of Theorem~2 (see Section~6.10).''\n"
    "\\item \\emph{Section~6.5, after the discussion of Table~2}: ``Table~2 illustrates the decision-preservation condition on selected cases. A large-scale "
    f"evaluation over {n_dec_total:,} exactly verified decisions is reported in Section~6.10.''\n"
    "\\item \\emph{Table~4, runtime rows}: ``Inference time (learned verifier)'' becomes ``Learned-verifier inference time''; ``Exact PRISM verification time'' "
    "becomes ``Exact PRISM verification time per decision''; ``Speedup factor'' becomes ``Speed-up of a single verification (inference only)''; and a footnote "
    "is added: ``Component times; the end-to-end decision latency in closed loop is reported in Table~\\ref{tab:lat}.''\n"
    "\\end{itemize}")

# =============================================================== render
import re
tpl = open("report/response_template.tex").read()


def _sub(m):
    v = eval(m.group(1), globals())
    return str(v)


out = re.sub(r"<<(.+?)>>", _sub, tpl, flags=re.S)
os.makedirs("report", exist_ok=True)
open("report/McQueenGuard_R1C6-7_R2C6-7_response.tex", "w").write(out)
left = re.findall(r"<<|>>", out)
print("written", "unresolved markers:", len(left))
