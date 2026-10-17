"""Build the LaTeX report (PDF) with the Comment 6/7 experiments from the result files."""
import json, sys, os
import pandas as pd
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg.style import METHOD_STYLE

LBL = {k: v["label"] for k, v in METHOD_STYLE.items()}
S = json.load(open("results/compare_summary.json"))
V = json.load(open("results/verifier_stats.json"))
E = json.load(open("results/embedded_profile.json"))
SS = json.load(open("results/shift_stream.json"))
SC = pd.read_csv("results/shift_closed_summary.csv")
ET = pd.read_csv("results/embedded_timing_summary.csv")
PV = pd.read_csv("data/prism_validation.csv")


def g(rows, m):
    return next(r for r in rows if r["method"] == m)


def f1(x):
    return f"{x:.1f}"


def flat(x):
    if x < 0.01:
        return "$<$0.01"
    if x < 1:
        return f"{x:.2f}"
    if x < 10:
        return f"{x:.1f}"
    return f"{x:.0f}"


MQ, EX, AB, VA, MP, RB, PP = ("McQueenGuard", "McQueenGuard (exact only)", "McQueenGuard w/o admission (ablation)",
                              "VerifAI runtime assurance", "MPC (Zhang et al. 2022)", "Rule-based (IDM + AEB)",
                              "End-to-end RL (PPO)")
O = {r["method"]: r for r in S["overall"]}
ID = {r["method"]: r for r in S["ID"]}
OD = {r["method"]: r for r in S["OOD"]}
mc = S["mcnemar_mv"]
mcc = S["mcnemar_col"]
n_ep = O[MQ]["episodes"]


def pfmt(p):
    return "$p<0.001$" if p < 0.001 else f"$p={p:.3f}$"


# ---------------------------------------------------------------- tables
t_overall = []
for m in (MQ, EX, AB, VA, MP, RB, PP):
    r = O[m]
    t_overall.append(f"{LBL[m]} & {f1(r['collision'])} & {f1(r['margin_violation'])} & {f1(r['risk_violation'])} & "
                     f"{f1(r['deadlock'])} & {r['delay_ratio']:.2f} & {flat(r['lat_mean'])} & {flat(r['lat_p99'])} & {f1(r['miss'])} \\\\")
t_idood = []
for m in (MQ, EX, AB, VA, MP, RB, PP):
    a, b = ID[m], OD[m]
    ls = f"{a['learned_share']:.0f} / {b['learned_share']:.0f}" if "McQueenGuard" in m and m != EX else "--"
    t_idood.append(f"{LBL[m]} & {f1(a['collision'])} & {f1(a['margin_violation'])} & {f1(a['deadlock'])} & "
                   f"{f1(b['collision'])} & {f1(b['margin_violation'])} & {f1(b['deadlock'])} & {ls} \\\\")

L = E["verifier_latency"]
p = E["learned_decision_path"]
q = E["prism_latency_ms"]
mem = E["memory_mb"]
mod = E["model"]
proj = E["orin_projection"]
t_emb = []
for k, v in L.items():
    t_emb.append(f"Learned verifier only -- {k} & {v['mean_us']:.1f}\\,$\\mu$s & {v['p99_us']:.1f}\\,$\\mu$s \\\\")
t_emb.append(f"Complete learned decision (features, MLP, gate, admission, action) & {p['mean_us']:.1f}\\,$\\mu$s & {p['p99_us']:.1f}\\,$\\mu$s \\\\")
t_emb.append(f"Exact verification, PRISM 4.8.1 warm server (sparse engine) & {q['mean']:.0f}\\,ms & {q['p99']:.0f}\\,ms \\\\")
t_proj = []
for k in ("low", "central", "high"):
    pr = proj[k]
    t_proj.append(f"{k.capitalize()} ($\\times${pr['factor']}) & {pr['learned_mean_ms']*1000:.0f}\\,$\\mu$s & {pr['learned_p99_ms']*1000:.0f}\\,$\\mu$s & "
                  f"{pr['exact_mean_ms']:.0f}\\,ms & {pr['exact_p99_ms']:.0f}\\,ms & "
                  f"{pr['energy_upper_mJ']['15 W']['learned']:.1f}\\,mJ / {pr['energy_upper_mJ']['MAXN 60 W']['learned']:.1f}\\,mJ & "
                  f"{pr['energy_upper_mJ']['15 W']['exact']/1000:.1f}\\,J / {pr['energy_upper_mJ']['MAXN 60 W']['exact']/1000:.1f}\\,J \\\\")

t_et = []
for m in (MQ, EX, VA, MP, RB, PP, AB):
    d = ET[ET.method == m]
    if len(d) < 2:
        continue
    a = d[d.platform == "desktop"].iloc[0]
    b = d[d.platform == "embedded"].iloc[0]
    t_et.append(f"{LBL[m]} & {flat(a.lat_mean)} & {f1(a.deadline_miss)} & {f1(a.margin_violation)} & "
                f"{flat(b.lat_mean)} & {f1(b.deadline_miss)} & {f1(b.margin_violation)} \\\\")
n_et = int(ET.episodes.max())

t_ss = []
for k in sorted(SS):
    s = SS[k]
    t_ss.append(f"{float(k):.1f} & {100*s['coverage_static']:.1f} & {100*s['familiar_rate']:.1f} & "
                f"{100*s['static_nogate']['admission_rate']:.1f} / {100*s['static_nogate']['unsafe_admission_rate']:.1f} & "
                f"{100*s['static_gate']['admission_rate']:.1f} / {100*s['static_gate']['unsafe_admission_rate']:.1f} & "
                f"{100*s['adaptive_gate']['admission_rate']:.1f} / {100*s['adaptive_gate']['unsafe_admission_rate']:.1f} & "
                f"{s['adaptive_gate']['eps_final']:.3f} \\\\")
t_sc = []
order_sc = ["McQueenGuard (static bound)", "McQueenGuard (online recalibration)", VA, MP, RB, PP]
sevs = sorted(SC.severity.unique())
for m in order_sc:
    cells = []
    for s in sevs:
        d = SC[(SC.method == m) & (SC.severity == s)]
        cells.append(f"{f1(d.collision.iloc[0])} / {f1(d.margin_violation.iloc[0])}" if len(d) else "--")
    t_sc.append(f"{LBL[m]} & " + " & ".join(cells) + " \\\\")
n_sc = int(SC.episodes.max())
sc_hdr = " & ".join(f"$s={s:.1f}$" for s in sevs)

ss0, ss1 = SS[sorted(SS)[0]], SS[sorted(SS)[-1]]

tex = r"""\documentclass[11pt]{article}
\usepackage[a4paper,margin=2.2cm]{geometry}
\usepackage{graphicx,booktabs,amsmath,amssymb,xcolor,caption,subcaption,tabularx,array,enumitem}
\usepackage[hidelinks]{hyperref}
\usepackage{mathptmx}
\usepackage{adjustbox}
\captionsetup{font=small,labelfont=bf}
\setlength{\parskip}{3pt}
\newcolumntype{Y}{>{\raggedright\arraybackslash}X}
\title{\vspace{-1.2cm}McQueenGuard -- JSA-D-26-01243\\[2pt]\large Additional experiments for Reviewer~\#1, Comments~6 and~7}
\date{}
\begin{document}
\maketitle
\vspace{-1.2cm}
\noindent\textit{All numbers below were produced by the experiments described here; every assumption that replaces a
measurement we could not make is stated explicitly (Section~\ref{sec:assump}). Figure/table numbers are local to this
document.}

\section{Common experimental setup}\label{sec:setup}
\textbf{Closed-loop testbed.} To run every method on \emph{identical, paired} scenario instances and to measure
computation latency inside the loop, all experiments use a reproducible Python re-implementation of the paper's
pedestrian-crossing scenario (Section~6.1). The ego vehicle (length 4.7\,m) approaches a crosswalk from 70\,m; a
pedestrian waits at the kerb and crosses with stochastic intent, speed (1.0--1.8\,m/s) and hesitation (30\,\%). Weather
presets (clear, rain, fog) use the friction, camera-noise, LiDAR-accuracy and perception-latency values of the McQueenGuard
CARLA configuration; detection probability decays with distance and visibility, tracks drop out, measurements are noisy
and arrive with 90--130\,ms perception latency; actuation has a 0.1\,s delay and a 0.2\,s first-order lag. Every planner is
invoked every 100\,ms and its command takes effect only after its own computation time has elapsed (the previous command is
held meanwhile). The ten scenario categories of Table~3 are used (six in-distribution categories plus high speed
50--60\,km/h, novel trajectory [running or occluded pedestrians], high sensor noise [$\times$3 noise, +15\,\% drop-outs] and
unseen weather [snow, $\mu\approx0.3$]).

\textbf{McQueenGuard implementation.} At each decision point the parametric MDP is instantiated with the current
valuation $\lambda$ (friction, perception-error probability, visibility, pedestrian speed and intent, presence belief) and
checked for the five actions of Table~2 (accelerate, maintain, reduce, brake, emergency brake) against
$P_{\max}{=}?[F^{\le 4\,s}\,\text{unsafe}]$ (collision \emph{or} ego within $d_{SM}=2$\,m of a pedestrian in the lane) and
$R_{\max}\{\text{performance}\}{=}?[C^{\le 4\,s}]$, with $\theta=0.05$ and the safety-first lexicographic rule of
Eqs.~(16)--(17). Exact verification is performed by \textbf{PRISM~4.8.1} (persistent warm JVM, sparse engine); an
equivalent compiled backward induction, which agrees with PRISM to $\max|\Delta P| = """ + f"{PV.max_dP.max()/10**int(f'{PV.max_dP.max():e}'.split('e')[1]):.1f}" + r"""\times10^{""" + f"{int(f'{PV.max_dP.max():e}'.split('e')[1])}" + r"""}$ on
""" + f"{len(PV)}" + r""" runtime queries, is used to label data. The learned verifier is the MLP of Section~6.6
(18 inputs, 64--64--32 hidden units), trained on """ + f"{V['n_total']:,}" + r""" verified decision points
(""" + f"{V['n_episodes']}" + r""" episodes, episode-wise 70/15/15 split): MAE of the unsafe probability
""" + f"{V['mae_p']:.4f}" + r""", MAE of the utility """ + f"{V['mae_u']:.4f}" + r""". The split-conformal margin
(Eq.~25, $\alpha=0.05$) is $\varepsilon=""" + f"{V['eps']:.3f}" + r"""$ with an empirical test coverage of
""" + f"{100*V['coverage_test']:.1f}" + r"""\,\% and an unsafe-admission rate of """ + f"{100*V['unsafe_admission_rate_test']:.1f}" + r"""\,\%
of admitted test decisions ($\le\alpha$, as predicted by Corollary~1). The familiarity set $\mathcal{H}$ is implemented as a
Mahalanobis gate (99th calibration percentile) combined with the training envelope of every feature. While an exact result
is pending after a non-admitted decision, the fallback control of Algorithm~1 (line~26) applies at least the reduce-speed
action; the Simplex monitor of Section~4.5 runs every 50\,ms.

\textbf{Hardware.} All timings were measured on the same machine (Intel Xeon @\,2.1\,GHz, 2 vCPU, single-threaded
inference). This machine differs from the desktop used in Section~6, so absolute times are not directly comparable with
Fig.~14 and Table~4.

\section{Comment 7 -- Comparison with learning-assisted verification, conventional planners and end-to-end RL}\label{sec:c7}
\subsection{Baselines}
\begin{itemize}[leftmargin=1.3em,itemsep=1pt]
\item \textbf{VerifAI runtime assurance} (learning-assisted verification; Dreossi et al., CAV 2019; Torfah et al., RV 2021):
the scenario space (weather, speed, pedestrian timing, speed, intent, hesitation) is defined with the VerifAI feature API;
VerifAI's cross-entropy sampler falsifies the learning-enabled controller (the PPO planner below) against the specification
$\mathbf{G}(\text{gap}\ge d_{SM})\wedge\neg\text{collision}$ (400 falsification runs); a decision-tree monitor learned on the
resulting traces predicts a violation within 2\,s (threshold chosen for $\ge$95\,\% recall) and switches to a braking safe
controller (Simplex).
\item \textbf{MPC} (conventional planner; Zhang et al., IEEE T-ITS 23(7), 2022, and the authors' public MPC-ROS code): receding-horizon QP (4\,s horizon, 0.1\,s step) with the speed-tracking and smoothness cost of their Eq.~(23),
input/speed bounds and friction-limited deceleration (Eqs.~8--10, 14), constant-velocity prediction of the pedestrian
(Eqs.~19--20) and the safe-distance constraint with reaction time (Eqs.~15--16); CVXPY\,+\,OSQP as in MPC-ROS.
\item \textbf{Rule-based}: Intelligent Driver Model (Treiber et al., 2000) with a virtual stop line for a detected crossing
pedestrian and TTC-triggered emergency braking (1.4\,s).
\item \textbf{End-to-end RL}: PPO policy (Stable-Baselines3, 128--128 MLP) mapping the perception features and the desired
speed directly to acceleration, trained for 2.5\,M steps on the in-distribution categories with a CaRL-style reward (route
progress multiplied by soft comfort/speed penalties; termination and penalty on collision; Jaeger et al., CoRL 2025).
\item \textbf{McQueenGuard (exact only)} -- exact PRISM verification at every decision; and \textbf{learned without
admission} -- the learned verifier used unconditionally (ablation of the error-aware admission and the exact fallback).
\end{itemize}
Each method ran on the same """ + f"{n_ep}" + r""" scenario instances (30 per category). \emph{Safety violation} = collision or
safe-margin violation (gap $<2$\,m while the pedestrian is in the lane). \emph{Verified risk} = share of audited control
commands (one audit per second) whose exact 4-s unsafe probability is $\ge\theta$; it includes unavoidable states and is
therefore non-zero for every method. \emph{Deadlock} = the vehicle did not clear the crosswalk within $D_0/v_0+30$\,s.
\emph{Travel-time ratio} = time to clear the crosswalk divided by the free-flow time.

\begin{table}[h]
\centering\small
\caption{Overall results over all ten scenario categories (""" + f"{n_ep}" + r""" paired episodes per method).}\label{tab:c7}
\begin{adjustbox}{max width=\linewidth}\begin{tabular}{lcccccccc}
\toprule
 & Collision & Safety viol. & Verified risk & Deadlock & Travel-time & \multicolumn{2}{c}{Decision latency (ms)} & Deadline \\
Method & (\%) & (\%) & $\ge\theta$ (\%) & (\%) & ratio & mean & p99 & misses (\%) \\
\midrule
""" + "\n".join(t_overall) + r"""
\bottomrule
\end{tabular}\end{adjustbox}
\end{table}

\begin{table}[h]
\centering\small
\caption{In-distribution (6 categories, 180 episodes) vs.\ out-of-distribution (4 categories, 120 episodes) results.
Last column: share of decisions taken by the learned verifier (ID / OOD).}\label{tab:c7b}
\begin{adjustbox}{max width=\linewidth}\begin{tabular}{lccccccc}
\toprule
 & \multicolumn{3}{c}{In-distribution} & \multicolumn{3}{c}{Out-of-distribution} & Learned \\
\cmidrule(lr){2-4}\cmidrule(lr){5-7}
Method & Collision & Safety viol. & Deadlock & Collision & Safety viol. & Deadlock & share (\%) \\
\midrule
""" + "\n".join(t_idood) + r"""
\bottomrule
\end{tabular}\end{adjustbox}
\end{table}

\begin{figure}[h]
\centering
\includegraphics[width=0.92\linewidth]{../figures/fig_c7_safety_by_group.pdf}
\caption{Safety violations (collision or safe-margin violation) per scenario group with 95\,\% Wilson intervals.}\label{fig:c7a}
\end{figure}
\begin{figure}[h]
\centering
\includegraphics[width=0.92\linewidth]{../figures/fig_c7_risk.pdf}
\caption{Share of audited control commands whose verified 4-s unsafe probability exceeds $\theta=0.05$.}\label{fig:c7b}
\end{figure}
\begin{figure}[h]
\centering
\includegraphics[width=0.92\linewidth]{../figures/fig_c7_latency.pdf}
\caption{Decision latency per method (log scale). McQueenGuard's mean includes the decisions that fall back to PRISM; its
learned decisions alone take $""" + f"{p['mean_us']:.0f}" + r"""\,\mu$s on average (Table~\ref{tab:emb}).}\label{fig:c7c}
\end{figure}

\subsection{Findings}
\begin{itemize}[leftmargin=1.3em,itemsep=2pt]
\item \textbf{Safety.} McQueenGuard had the fewest collisions together with the ablation (""" + f"{O[MQ]['collision']:.1f}" + r"""\,\% vs.\
""" + f"{O[MP]['collision']:.1f}" + r"""\,\% for MPC, """ + f"{O[RB]['collision']:.1f}" + r"""\,\% rule-based, """ + f"{O[PP]['collision']:.1f}" + r"""\,\% PPO and
""" + f"{O[VA]['collision']:.1f}" + r"""\,\% VerifAI) and """ + f"{O[MQ]['margin_violation']:.1f}" + r"""\,\% safety violations. In paired McNemar
tests McQueenGuard had significantly fewer safety violations than exact-only verification (""" + pfmt(mc[EX][2]) + r"""), MPC
(""" + pfmt(mc[MP][2]) + r""") and end-to-end PPO (""" + pfmt(mc[PP][2]) + r"""); the differences to the rule-based planner
(""" + pfmt(mc[RB][2]) + r""") and VerifAI (""" + pfmt(mc[VA][2]) + r""") are not significant. Its collision rate is significantly
lower than MPC's (""" + pfmt(mcc[MP][2]) + r""").
\item \textbf{In vs.\ out of distribution.} In-distribution, McQueenGuard had no collision and """ + f"{ID[MQ]['margin_violation']:.1f}" + r"""\,\% safety
violations with """ + f"{ID[MQ]['learned_share']:.0f}" + r"""\,\% learned decisions. Out of distribution the familiarity gate routed
""" + f"{100-OD[MQ]['learned_share']:.0f}" + r"""\,\% of the decisions to exact verification; violations rose for every method, most
for MPC (""" + f"{OD[MP]['margin_violation']:.1f}" + r"""\,\%) and the rule-based planner (""" + f"{OD[RB]['margin_violation']:.1f}" + r"""\,\%), whose
deterministic prediction ignores perception uncertainty and reduced friction.
\item \textbf{Assurance of the executed commands.} McQueenGuard executed the smallest share of commands with verified risk
above $\theta$ apart from VerifAI (Fig.~\ref{fig:c7b}); MPC, rule-based and PPO commands exceeded $\theta$ 1.3--1.6$\times$
more often. Only McQueenGuard attaches a quantitative, calibrated bound to each admitted decision.
\item \textbf{Real-time efficiency.} Exact verification at every step (""" + f"{O[EX]['lat_mean']:.0f}" + r"""\,ms mean,
""" + f"{O[EX]['lat_p99']:.0f}" + r"""\,ms p99) missed the 100\,ms deadline in """ + f"{O[EX]['miss']:.0f}" + r"""\,\% of decisions and
reacted late, which is why it had """ + f"{O[EX]['margin_violation']:.1f}" + r"""\,\% safety violations despite verifying every decision.
McQueenGuard missed """ + f"{O[MQ]['miss']:.0f}" + r"""\,\% of deadlines, all of them fallbacks to PRISM in uncertain situations.
\item \textbf{Trade-offs, reported openly.} VerifAI reached a similar violation rate (""" + f"{O[VA]['margin_violation']:.1f}" + r"""\,\%)
but deadlocked in """ + f"{O[VA]['deadlock']:.1f}" + r"""\,\% of episodes because its conservative monitor keeps braking. The ablation
without admission had fewer safety violations in this testbed (""" + f"{O[AB]['margin_violation']:.1f}" + r"""\,\%, no fallback latency)
but """ + f"{O[AB]['deadlock']:.1f}" + r"""\,\% deadlocks (""" + f"{OD[AB]['deadlock']:.1f}" + r"""\,\% out of distribution) and no statistical
guarantee; under sensor-noise shift its unsafe-admission rate is the highest (Section~\ref{sec:shift}). McQueenGuard is
more cautious than MPC and the rule-based planner (travel-time ratio """ + f"{O[MQ]['delay_ratio']:.2f}" + r""" vs.\
""" + f"{O[MP]['delay_ratio']:.2f}" + r"""/""" + f"{O[RB]['delay_ratio']:.2f}" + r""").
\end{itemize}

\section{Comment 6 -- Embedded deployment profile and sim-to-real robustness}\label{sec:c6}
\subsection{Latency, memory and energy of the learned verifier}
The learned verifier was exported to ONNX and measured in four runtimes with one thread and batch size one
(""" + f"{20000:,}" + r""" inferences each); the complete learned decision path was measured on 2,000 recorded runtime
observations. The MLP has """ + f"{mod['params']:,}" + r""" parameters and """ + f"{mod['macs']:,}" + r""" multiply-accumulates per
inference.

\begin{table}[h]
\centering\small
\caption{Measured latency (single thread) on the testbed CPU.}\label{tab:emb}
\begin{adjustbox}{max width=\linewidth}\begin{tabular}{lcc}
\toprule
Component & mean & p99 \\
\midrule
""" + "\n".join(t_emb) + r"""
\bottomrule
\end{tabular}\end{adjustbox}
\end{table}

\textbf{Memory.} Weights occupy """ + f"{mod['fp32_kib']:.1f}" + r"""\,KiB in FP32 (ONNX file """ + f"{mod['onnx_fp32_kib']:.1f}" + r"""\,KiB;
INT8 file """ + f"{mod['onnx_int8_kib']:.1f}" + r"""\,KiB) and activations """ + f"{mem['mlp_activations_bytes']}" + r"""\,bytes. A
stand-alone Python/ONNX Runtime process serving the INT8 model has a resident set of
""" + f"{mem['ort_int8_process']:.0f}" + r"""\,MB, of which the inference session itself accounts for """ + f"{mem['ort_int8_session_increment']:.0f}" + r"""\,MB,
whereas the PRISM verification server peaks at """ + f"{mem['prism_jvm_peak_rss']:.0f}" + r"""\,MB. INT8 dynamic quantisation changes
the predictions by at most """ + f"{E['int8_accuracy']['max_dev_int8_vs_fp32']:.4f}" + r""" (MAE """ + f"{E['int8_accuracy']['mae_fp32']:.4f}" + r"""
$\rightarrow$ """ + f"{E['int8_accuracy']['mae_int8']:.4f}" + r"""); after re-calibration $\varepsilon$ is """ + f"{E['int8_accuracy']['eps_int8']:.3f}" + r"""
with """ + f"{100*E['int8_accuracy']['coverage_int8']:.1f}" + r"""\,\% coverage: the admission mechanism remains valid after
re-calibration, with a wider margin than the FP32 model ("""+ f"{E['int8_accuracy']['eps_fp32']:.3f}" + r"""). On this x86 CPU the INT8 model is not faster than
FP32; INT8 mainly reduces the model size.

\textbf{Projection to NVIDIA Jetson AGX Orin.} We did not have access to an Orin board. Table~\ref{tab:orin} therefore
\emph{projects} the measured latencies to the Orin CPU (12$\times$ Arm Cortex-A78AE at up to 2.2\,GHz, 15--60\,W module power;
NVIDIA AGX Orin technical brief) with an explicit single-thread slow-down band of 1.5--4$\times$ relative to the testbed
core, and bounds the energy per decision by \emph{module power envelope $\times$ decision time} (a conservative upper bound,
since the module power includes idle subsystems).

\begin{table}[h]
\centering\small
\caption{Projected decision latency and energy per decision on Jetson AGX Orin (assumption-based, see text).}\label{tab:orin}
\begin{adjustbox}{max width=\linewidth}\begin{tabular}{lcccccc}
\toprule
Slow-down & \multicolumn{2}{c}{Learned decision} & \multicolumn{2}{c}{Exact (PRISM)} & \multicolumn{2}{c}{Energy per decision (upper bound), 15\,W / 60\,W} \\
\cmidrule(lr){2-3}\cmidrule(lr){4-5}\cmidrule(lr){6-7}
assumption & mean & p99 & mean & p99 & learned & exact \\
\midrule
""" + "\n".join(t_proj) + r"""
\bottomrule
\end{tabular}\end{adjustbox}
\end{table}

\subsection{Closed loop under embedded timing}
To test whether the safety results survive slower hardware, """ + f"{n_et}" + r""" episodes (10 per category) were re-run with every
measured computation time multiplied by 2.5 (the central projection), on the same scenario instances as Section~\ref{sec:c7}.

\begin{table}[h]
\centering\small
\caption{Testbed timing vs.\ embedded timing ($\times$2.5), """ + f"{n_et}" + r""" paired episodes per method.}\label{tab:et}
\begin{adjustbox}{max width=\linewidth}\begin{tabular}{lcccccc}
\toprule
 & \multicolumn{3}{c}{Testbed timing} & \multicolumn{3}{c}{Embedded timing ($\times$2.5)} \\
\cmidrule(lr){2-4}\cmidrule(lr){5-7}
Method & latency (ms) & misses (\%) & safety viol. (\%) & latency (ms) & misses (\%) & safety viol. (\%) \\
\midrule
""" + "\n".join(t_et) + r"""
\bottomrule
\end{tabular}\end{adjustbox}
\end{table}
\begin{figure}[h]
\centering
\includegraphics[width=0.95\linewidth]{../figures/fig_c6_embedded_timing.pdf}
\caption{Deadline misses and safety violations with testbed vs.\ embedded timing (MQG = McQueenGuard).}\label{fig:et}
\end{figure}
\textbf{Findings.} Methods whose decisions take well under 100\,ms (VerifAI, rule-based, PPO, the learned-only ablation) were
unaffected. McQueenGuard's safety violations rose from """ + f"{ET[(ET.method==MQ)&(ET.platform=='desktop')].margin_violation.iloc[0]:.0f}" + r"""\,\% to
""" + f"{ET[(ET.method==MQ)&(ET.platform=='embedded')].margin_violation.iloc[0]:.0f}" + r"""\,\% and exact-only verification from
""" + f"{ET[(ET.method==EX)&(ET.platform=='desktop')].margin_violation.iloc[0]:.0f}" + r"""\,\% to """ + f"{ET[(ET.method==EX)&(ET.platform=='embedded')].margin_violation.iloc[0]:.0f}" + r"""\,\%,
because every fallback now waits 2.5 times longer for PRISM (about 1\,s on average). The learned path itself stays far below the
deadline on embedded hardware; the exact fallback is the bottleneck. On a vehicle the exact verifier should therefore run on a
dedicated core with a bounded model size, or be replaced by a faster exact engine, while the fallback control covers the
waiting time. With 100 episodes per method these increases are indicative rather than statistically conclusive.

\subsection{Shift of the sensor-noise distribution (simulation-to-real)}\label{sec:shift}
At test time the perception noise model was replaced by a \emph{combined} shift of severity $s\in\{0,0.5,1\}$ that the system
is not told about: heavy-tailed Student-$t$ noise (degrees of freedom $6-4s$, scale $\times(1+1.5s)$), a systematic lateral bias
of $0.6s$\,m, $+25s$\,\% drop-outs and $+120s$\,ms latency. McQueenGuard handles such a shift with three mechanisms that are
already part of the framework or follow from it: (i) the familiarity gate (Mahalanobis distance + training envelope) routes
unfamiliar feature vectors to exact verification; (ii) the exact results produced by these fallbacks, plus shadow audits of
10\,\% of the admitted decisions (run off the critical path), feed a \emph{shift-aware recalibration} that recomputes the
conformal quantile of Eq.~(25) over a rolling window of the last 200 residuals and never lowers $\varepsilon$ below the offline
value; (iii) the Simplex monitor and exact fallback remain active regardless of the learned verifier.

\begin{table}[h]
\centering\small
\caption{Decision-stream analysis under shift ("""+ f"{ss0['n']:,}" + r"""--""" + f"{max(SS[k]['n'] for k in SS):,}" + r""" exactly verified decision points per
severity). Admission / unsafe-admission rates in \% (unsafe = admitted while the exact unsafe probability is $\ge\theta$).}\label{tab:ss}
\begin{adjustbox}{max width=\linewidth}\begin{tabular}{ccccccc}
\toprule
 & Coverage of & Familiar & Static bound, & Static bound & + online & $\varepsilon$ after \\
$s$ & static bound (\%) & (\%) & no gate & + gate & recalibration & recalibration \\
\midrule
""" + "\n".join(t_ss) + r"""
\bottomrule
\end{tabular}\end{adjustbox}
\end{table}
\begin{figure}[h]
\centering
\includegraphics[width=\linewidth]{../figures/fig_c6_shift_stream.pdf}
\caption{Effect of the sensor-noise shift on (a) coverage and familiarity, (b) unsafe-admission rate and (c) admission rate.}\label{fig:ss}
\end{figure}

\begin{table}[h]
\centering\small
\caption{Closed loop under the shift: collision / safety-violation rate (\%), """ + f"{n_sc}" + r""" episodes per severity and method
(in-distribution categories).}\label{tab:sc}
\begin{adjustbox}{max width=\linewidth}\begin{tabular}{l""" + "c" * len(sevs) + r"""}
\toprule
Method & """ + sc_hdr + r""" \\
\midrule
""" + "\n".join(t_sc) + r"""
\bottomrule
\end{tabular}\end{adjustbox}
\end{table}

\textbf{Findings.} The coverage of the offline bound fell from """ + f"{100*ss0['coverage_static']:.1f}" + r"""\,\% to
""" + f"{100*ss1['coverage_static']:.1f}" + r"""\,\% and the unsafe-admission rate of the static bound rose from
""" + f"{100*ss0['static_gate']['unsafe_admission_rate']:.1f}" + r"""\,\% to """ + f"{100*ss1['static_gate']['unsafe_admission_rate']:.1f}" + r"""\,\%
of admitted decisions. The gate removed """ + f"{100*(1-ss1['familiar_rate']):.0f}" + r"""\,\% of the shifted decisions at $s=1$. With
online recalibration $\varepsilon$ grew to """ + f"{ss1['adaptive_gate']['eps_final']:.3f}" + r""" and the unsafe-admission rate stayed at
""" + f"{100*ss1['adaptive_gate']['unsafe_admission_rate']:.1f}" + r"""\,\%, at the price of admitting """ + f"{100*ss1['adaptive_gate']['admission_rate']:.0f}" + r"""\,\%
instead of """ + f"{100*ss1['static_gate']['admission_rate']:.0f}" + r"""\,\% of the decisions to the learned verifier. The guarantee therefore
degrades gracefully and the system trades speed-up for assurance automatically; recalibration does not restore exchangeability,
so under persistent shift the statistical statement remains conditional on the recalibration window.
In closed loop (Table~\ref{tab:sc}) McQueenGuard had no collision at any severity, whereas VerifAI, MPC and PPO reached
""" + f"{SC[(SC.method==VA)&(SC.severity==1.0)].margin_violation.iloc[0]:.0f}" + r""", """ + f"{SC[(SC.method==MP)&(SC.severity==1.0)].margin_violation.iloc[0]:.0f}" + r""" and
""" + f"{SC[(SC.method==PP)&(SC.severity==1.0)].margin_violation.iloc[0]:.0f}" + r"""\,\% safety violations at $s=1$. With 30 episodes per cell a
difference of one or two episodes is not significant, so this table complements, rather than replaces, the decision-stream
analysis.

\section{Suggested answers for the revision letter}
\textbf{Reviewer~1, Comment~6.} \emph{We thank the reviewer for this suggestion. We have added a deployment study
(Section~6.X, Tables~X--X). The learned verifier has """ + f"{mod['params']:,}" + r""" parameters (""" + f"{mod['fp32_kib']:.0f}" + r"""\,KiB FP32,
""" + f"{mod['onnx_int8_kib']:.0f}" + r"""\,KiB INT8) and a complete learned decision takes """ + f"{p['mean_us']:.0f}" + r"""\,$\mu$s on average
(""" + f"{p['p99_us']:.0f}" + r"""\,$\mu$s p99) on a single CPU thread, versus """ + f"{q['mean']:.0f}" + r"""\,ms for PRISM; an ONNX Runtime process
needs """ + f"{mem['ort_int8_process']:.0f}" + r"""\,MB versus """ + f"{mem['prism_jvm_peak_rss']:.0f}" + r"""\,MB for the PRISM server. We did not
have access to an NVIDIA Orin board; we therefore report projected Orin figures under an explicit 1.5--4$\times$ CPU slow-down
assumption, an upper bound on the energy per decision derived from the module's 15--60\,W power envelope, and a closed-loop
re-run with all computation times scaled by 2.5. In that re-run the learned path stayed far below the 100\,ms deadline, but
McQueenGuard's safety violations rose from """ + f"{ET[(ET.method==MQ)&(ET.platform=='desktop')].margin_violation.iloc[0]:.0f}" + r"""\,\% to """ + f"{ET[(ET.method==MQ)&(ET.platform=='embedded')].margin_violation.iloc[0]:.0f}" + r"""\,\%
because fallbacks wait longer for PRISM; we therefore recommend running the exact verifier on a dedicated core. We also release the ONNX models and the benchmarking script so that the
measurements can be repeated on the target hardware. Regarding domain adaptation, we evaluated an unannounced shift of the
sensor-noise distribution (heavy tails, bias, drop-outs and latency). The coverage of the offline conformal bound decreased
from """ + f"{100*ss0['coverage_static']:.1f}" + r"""\,\% to """ + f"{100*ss1['coverage_static']:.1f}" + r"""\,\%; the familiarity gate routed
unfamiliar inputs to exact verification, and a shift-aware recalibration that reuses the fallback results kept the
unsafe-admission rate at """ + f"{100*ss1['adaptive_gate']['unsafe_admission_rate']:.1f}" + r"""\,\%. We discuss in Section~6.8 that this
does not restore exchangeability and that on-vehicle validation remains future work.}

\textbf{Reviewer~1, Comment~7.} \emph{We thank the reviewer. We have added a closed-loop comparison (Section~6.Y,
Tables~Y--Y, Figs.~Y--Y) with (1) a VerifAI-based runtime-assurance pipeline (falsification with VerifAI's cross-entropy
sampler, learned monitor, Simplex switching), (2) the collision-avoidance MPC of Zhang et al.\ (T-ITS 2022) and an IDM-based
rule planner, and (3) an end-to-end PPO planner trained with a CaRL-style reward, plus McQueenGuard with exact verification
only and an ablation without the admission mechanism. Over """ + f"{n_ep}" + r""" paired episodes in ten scenario categories,
McQueenGuard had a """ + f"{O[MQ]['collision']:.1f}" + r"""\,\% collision rate and """ + f"{O[MQ]['margin_violation']:.1f}" + r"""\,\% safety
violations, significantly fewer violations than MPC, end-to-end RL and exact-only verification, while making
""" + f"{O[MQ]['learned_share']:.0f}" + r"""\,\% of its decisions in microseconds. The comparison also shows the costs of the approach:
it is more conservative than MPC and the rule-based planner, and decisions that fall back to PRISM can miss the 100\,ms
deadline; the Simplex monitor and the fallback control cover these intervals.}

\section{Assumptions and limitations}\label{sec:assump}
\begin{enumerate}[leftmargin=1.5em,itemsep=1pt]
\item The comparison uses a reproducible Python testbed modelled on the paper's CARLA scenario, not CARLA itself; absolute
rates are therefore not interchangeable with Section~6.
\item Orin latency and energy are projections (slow-down band 1.5--4$\times$, energy bounded by the module power envelope);
no Orin hardware was measured.
\item In the single-lane testbed the yaw-angle, lane-deviation and relative-vehicle-velocity slots of the 18-feature vector
carry the pedestrian presence belief, the kerb waiting time and the speed relative to the desired speed.
\item Baselines were implemented by us following the cited papers; VerifAI's sampler and specification-monitor classes were
used directly, the MPC follows Zhang et al.\ and the public MPC-ROS solver set-up, and the PPO reward follows the CaRL design.
\item 30 episodes per category (300 per method) for the main comparison, 10 per category under embedded timing and
""" + f"{n_sc//6}" + r""" per category under shift; confidence intervals are shown where space permits.
\end{enumerate}
\end{document}
"""
os.makedirs("report", exist_ok=True)
open("report/experiments_R1_C6_C7.tex", "w").write(tex)
print("written")
