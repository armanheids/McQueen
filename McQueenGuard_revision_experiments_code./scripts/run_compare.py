"""Comment 7: closed-loop comparison of McQueenGuard with VerifAI, MPC, rule-based and end-to-end RL.

All methods run on identical (paired) scenario instances.  Usage:
  run_compare.py <n_per_category> <worker> <n_workers> [out_tag] [platform_scale] [exact_scale]
"""
import sys, random, pickle, time
import numpy as np
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg.config import CATEGORIES
from mqg.world import sample_episode, query
from mqg.sim import run_episode
from mqg.controllers import McQueenGuard, McQueenGuardExact, IDMRule, LatencyModel
from mqg.baselines import MPCPlanner, PPOPlanner, VerifAIRA
from mqg.verifier import LearnedVerifier
from mqg.pmdp import ExactSolver


def build_methods(lm, ver, only=None):
    if only is not None and only[0].startswith("="):
        names = [o[1:] for o in only]
        return [m for m in build_methods(lm, ver, None) if m.name in names]
    want = lambda name: only is None or any(o in name for o in only)
    ms = []
    if want("McQueenGuard"):
        ms += [McQueenGuard(ver, lm), McQueenGuardExact(lm),
               McQueenGuard(ver, lm, admission=False, name="McQueenGuard w/o admission (ablation)")]
    if want("VerifAI") or want("PPO"):
        ppo = PPOPlanner("models/ppo_planner")
        if want("VerifAI"):
            ms.append(VerifAIRA(ppo, "models/verifai_monitor.joblib"))
    if want("MPC"):
        ms.append(MPCPlanner())
    if want("Rule"):
        ms.append(IDMRule())
    if want("PPO"):
        ms.append(ppo)
    return ms


def main():
    n, wk, nw = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
    tag = sys.argv[4] if len(sys.argv) > 4 else "compare"
    pscale = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0
    escale = float(sys.argv[6]) if len(sys.argv) > 6 else None
    only = sys.argv[7].split(",") if len(sys.argv) > 7 else None
    lm = LatencyModel("data/prism_latency.json")
    ver = LearnedVerifier.load("models/verifier")
    methods = build_methods(lm, ver, only)
    solver = ExactSolver()

    def auditor(obs, u):
        pu, _ = solver.solve(query(obs), accs=[u])
        return float(pu[0])

    jobs = [(ci, e) for ci in range(len(CATEGORIES)) for e in range(n)]
    jobs = jobs[wk::nw]
    import os
    path = f"results/{tag}_w{wk}.pkl"
    out = pickle.load(open(path, "rb")) if os.path.exists(path) else []
    done = {(x.category, x.seed) for x in out}
    nm = len(methods)
    # keep only fully completed jobs
    cnt = {}
    for x in out:
        cnt[(x.category, x.seed)] = cnt.get((x.category, x.seed), 0) + 1
    out = [x for x in out if cnt[(x.category, x.seed)] == nm]
    done = {k for k, v in cnt.items() if v == nm}
    t0 = time.time()
    for j, (ci, e) in enumerate(jobs):
        cat = CATEGORIES[ci].name
        rng = random.Random(1_000_003 * (ci + 1) + 7919 * e)
        spec = sample_episode(cat, rng, seed=10_000 * (ci + 1) + e)
        if (cat, spec.seed) in done:
            continue
        for m in methods:
            r = run_episode(spec, m, auditor=auditor, platform_scale=pscale, exact_scale=escale)
            r.decisions = []
            out.append(r)
        if j % 10 == 0:
            print(f"[{wk}] {j}/{len(jobs)} {cat} ep{e} {time.time() - t0:.0f}s", flush=True)
            with open(path + ".tmp", "wb") as f:
                pickle.dump(out, f)
            os.replace(path + ".tmp", path)
    with open(path, "wb") as f:
        pickle.dump(out, f)
    print("done", len(out), f"{time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
