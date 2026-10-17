"""Comment 6 (domain adaptation): sim-to-real shift of the perception-noise distribution.

Part A (decision-stream analysis): decision points visited under shifted perception (heavy-tailed
Student-t noise, systematic bias, extra drop-outs and latency, jointly scaled by a severity s in
[0,1]) are labelled by exact verification.  For each severity we report conformal coverage of the
offline bound, familiarity rate, admission rate and unsafe-admission rate for three admission
policies: (i) static bound without gate, (ii) static bound + familiarity gate (McQueenGuard),
(iii) + shift-aware online recalibration (rolling conformal window, 10 % shadow audits).
Part B (closed loop): collision / margin-violation rates for McQueenGuard (static, adaptive) and the
baselines under the same shifts.
Usage: shift_experiment.py gen <severity> <first_ep> <n_ep>   |   shift_experiment.py analyse
       shift_experiment.py closed <severity> <worker> <n_workers> <n_per_cat>
"""
import sys, random, pickle, time, json, glob
import numpy as np
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg.config import NOMINAL, THETA, ALPHA_CONF, EPS_MAX
from mqg.world import sample_episode, NoiseShift, query
from mqg.sim import run_episode
from mqg.verifier import LearnedVerifier, admissible_residual, conformal_eps
from mqg.controllers import (McQueenGuard, McQueenGuardExact, IDMRule, LatencyModel, OnlineRecalibrator,
                             lexicographic_select)

SEVERITIES = [0.0, 0.5, 1.0]


def gen(sev, first, n):
    import os
    out = f"data/shift_{sev:.2f}_{first}.npz"
    if os.path.exists(out):
        return
    sys.argv = ["gen_dataset.py", out, str(first), str(n), "combined", str(sev)]
    import runpy
    runpy.run_path(__file__.rsplit("/", 1)[0] + "/gen_dataset.py", run_name="__main__")


def analyse():
    ver = LearnedVerifier.load("models/verifier")
    out = {}
    for sev in SEVERITIES:
        files = sorted(glob.glob(f"data/shift_{sev:.2f}_*.npz"))
        if not files:
            continue
        D = [np.load(f) for f in files]
        X = np.concatenate([d["X"] for d in D]).astype(np.float64)
        P = np.concatenate([d["P"] for d in D])
        U = np.concatenate([d["U"] for d in D])
        Ph, Uh = ver.predict(X)
        fam = ver.familiar(X)
        r = admissible_residual(P, Ph)
        cov = float(np.mean(r <= ver.eps))
        stats = dict(n=int(len(X)), coverage_static=cov, familiar_rate=float(fam.mean()),
                     mae=float(np.abs(Ph - P).mean()))
        for pol in ("static_nogate", "static_gate", "adaptive_gate"):
            rec = OnlineRecalibrator(ver, seed=1) if pol == "adaptive_gate" else None
            adm = viol = 0
            eps_trace = []
            for i in range(len(X)):
                eps = rec.eps() if rec else ver.eps
                a, ok = lexicographic_select(Ph[i], Uh[i], eps)
                admitted = ok and eps <= EPS_MAX and (fam[i] or pol == "static_nogate")
                if admitted:
                    adm += 1
                    viol += int(P[i, a] >= THETA)
                    if rec is not None and rec.should_audit():
                        rec.update(P[i], Ph[i])
                elif rec is not None:
                    rec.update(P[i], Ph[i])          # exact fallback result is fed back
                eps_trace.append(eps)
            stats[pol] = dict(admission_rate=adm / len(X), unsafe_admissions=viol,
                              unsafe_admission_rate=viol / max(1, adm),
                              unsafe_admission_per_decision=viol / len(X),
                              eps_final=float(eps_trace[-1]), eps_mean=float(np.mean(eps_trace)))
        out[f"{sev:.2f}"] = stats
        print(sev, json.dumps(stats))
    json.dump(out, open("results/shift_stream.json", "w"), indent=1)


def closed(sev, wk, nw, n_per_cat):
    from mqg.baselines import MPCPlanner, PPOPlanner, VerifAIRA
    from mqg.pmdp import ExactSolver
    lm = LatencyModel("data/prism_latency.json")
    ver = LearnedVerifier.load("models/verifier")
    ppo = PPOPlanner("models/ppo_planner")
    methods = [McQueenGuard(ver, lm, name="McQueenGuard (static bound)"),
               McQueenGuard(ver, lm, online_recal=OnlineRecalibrator(ver, seed=wk), name="McQueenGuard (online recalibration)"),
               VerifAIRA(ppo, "models/verifai_monitor.joblib"), MPCPlanner(), IDMRule(), ppo]
    solver = ExactSolver()

    def auditor(obs, u):
        return float(solver.solve(query(obs), accs=[u])[0][0])
    shift = NoiseShift("combined", sev)
    import os
    jobs = [(ci, e) for e in range(n_per_cat) for ci in range(len(NOMINAL))][wk::nw]
    path = f"results/shift_closed_{sev:.2f}_w{wk}.pkl"
    out = pickle.load(open(path, "rb")) if os.path.exists(path) else []
    cnt = {}
    for x in out:
        cnt[x.seed] = cnt.get(x.seed, 0) + 1
    out = [x for x in out if cnt[x.seed] == len(methods)]
    done = {k for k, v in cnt.items() if v == len(methods)}
    t0 = time.time()
    for j, (ci, e) in enumerate(jobs):
        cat = NOMINAL[ci].name
        rng = random.Random(77_777 * (ci + 1) + 31 * e)
        spec = sample_episode(cat, rng, seed=500_000 + 1000 * ci + e, shift=shift)
        if spec.seed in done:
            continue
        for m in methods:
            r = run_episode(spec, m, auditor=auditor)
            r.decisions = []
            r.severity = sev
            out.append(r)
        if j % 5 == 0:
            print(f"[{wk}] sev {sev} {j}/{len(jobs)} {time.time() - t0:.0f}s", flush=True)
            with open(path + ".tmp", "wb") as f:
                pickle.dump(out, f)
            os.replace(path + ".tmp", path)
    with open(path, "wb") as f:
        pickle.dump(out, f)


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "gen":
        gen(float(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]))
    elif mode == "analyse":
        analyse()
    elif mode == "closed":
        closed(float(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]))
