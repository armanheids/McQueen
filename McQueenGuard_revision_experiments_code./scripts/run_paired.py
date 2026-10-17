"""Paired closed-loop runs for all methods/variants on the same scenario instances.

Episode (ci, e) of category ci uses the same scenario for every method (seed 10000*(ci+1)+e).
  run_paired.py --methods full,noeps,... --e0 0 --n 60 --worker 0 --nworkers 2 --tag abl [--label 3]
With --label k, every k-th decision of the full McQueenGuard is additionally labelled with exact
verification of all actions (off the timed path) for the decision-level analyses (R2-C6/C7).
"""
import sys, os, random, pickle, time, argparse
import numpy as np
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg.config import CATEGORIES
from mqg.world import sample_episode, query, features
from mqg.sim import run_episode
from mqg.controllers import McQueenGuard, McQueenGuardExact, IDMRule, LatencyModel
from mqg.verifier import LearnedVerifier
from mqg.pmdp import ExactSolver

NAMES = {
    "full": "McQueenGuard",
    "noeps": "McQueenGuard - conformal margin",
    "nogate": "McQueenGuard - familiarity gate",
    "noexact": "McQueenGuard - exact fallback",
    "learned": "McQueenGuard w/o admission (ablation)",
    "nomon": "McQueenGuard - Simplex shield",
    "learned_nomon": "Learned verifier only - Simplex shield",
    "exact": "McQueenGuard (exact only)",
    "verifai": "VerifAI runtime assurance",
    "mpc": "MPC (Zhang et al. 2022)",
    "rule": "Rule-based (IDM + AEB)",
    "ppo": "End-to-end RL (PPO)",
    "ideal": "McQueenGuard, zero-latency exact fallback",
}


class Labelled(McQueenGuard):
    """Full McQueenGuard that records every k-th decision with its exact verification labels."""

    def __init__(self, *a, every=3, **k):
        super().__init__(*a, **k)
        self.every = every
        self.rows = []

    def reset(self, spec):
        super().reset(spec)
        self.k = 0

    def decide(self, obs):
        d = super().decide(obs)
        self.k += 1
        if self.k % self.every == 0:
            if "pu" in d.info:
                P, U = d.info["pu"], d.info["U"]
            else:
                P, U = self.solver.solve(query(obs))
            self.rows.append(dict(cat=self.spec.category, seed=self.spec.seed, t=obs["t"], x=d.info["x"],
                                  ph=np.asarray(d.info["pu_hat"]), uh=np.asarray(d.info["U_hat"]),
                                  fam=bool(d.info["familiar"]), mode=d.mode, act=int(d.info["action"]),
                                  eps=self._last["eps"], sm=bool(self._last["sm_active"]),
                                  P=np.asarray(P), U=np.asarray(U), tracked=bool(obs["tracked"]),
                                  d_hat=float(obs["d_hat"]), lat=d.latency))
        return d


def build(keys, lm, ver, label_every):
    out = []
    ppo = None
    for k in keys:
        if k == "full":
            out.append(Labelled(ver, lm, every=label_every) if label_every else McQueenGuard(ver, lm))
        elif k == "noeps":
            out.append(McQueenGuard(ver, lm, use_eps=False, name=NAMES[k]))
        elif k == "nogate":
            out.append(McQueenGuard(ver, lm, use_gate=False, name=NAMES[k]))
        elif k == "noexact":
            out.append(McQueenGuard(ver, lm, exact_fallback=False, name=NAMES[k]))
        elif k == "learned":
            out.append(McQueenGuard(ver, lm, admission=False, name=NAMES[k]))
        elif k == "nomon":
            out.append(McQueenGuard(ver, lm, use_monitor=False, name=NAMES[k]))
        elif k == "learned_nomon":
            out.append(McQueenGuard(ver, lm, admission=False, use_monitor=False, name=NAMES[k]))
        elif k == "exact":
            out.append(McQueenGuardExact(lm))
        elif k == "ideal":
            m = McQueenGuard(ver, lm, name=NAMES[k])
            m.exact_scale = 0.0          # exact results available without delay (idealised reference)
            out.append(m)
        elif k in ("verifai", "ppo"):
            from mqg.baselines import PPOPlanner, VerifAIRA
            ppo = ppo or PPOPlanner("models/ppo_planner")
            out.append(VerifAIRA(ppo, "models/verifai_monitor.joblib") if k == "verifai" else ppo)
        elif k == "mpc":
            from mqg.baselines import MPCPlanner
            out.append(MPCPlanner())
        elif k == "rule":
            out.append(IDMRule())
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--methods", required=True)
    ap.add_argument("--e0", type=int, default=0)
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--worker", type=int, default=0)
    ap.add_argument("--nworkers", type=int, default=1)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--label", type=int, default=0)
    a = ap.parse_args()
    import warnings
    warnings.filterwarnings("ignore")
    lm = LatencyModel("data/prism_latency.json")
    ver = LearnedVerifier.load("models/verifier")
    keys = a.methods.split(",")
    methods = build(keys, lm, ver, a.label)
    solver = ExactSolver()

    def auditor(obs, u):
        return float(solver.solve(query(obs), accs=[u])[0][0])

    jobs = [(ci, e) for ci in range(len(CATEGORIES)) for e in range(a.e0, a.n)]
    jobs = [j for j in jobs if j[1] % a.nworkers == a.worker]
    path = f"results/{a.tag}_w{a.worker}.pkl"
    state = pickle.load(open(path, "rb")) if os.path.exists(path) else dict(res=[], rows=[], done=set())
    lab = next((m for m in methods if isinstance(m, Labelled)), None)
    t0 = time.time()
    for j, (ci, e) in enumerate(jobs):
        cat = CATEGORIES[ci].name
        rng = random.Random(1_000_003 * (ci + 1) + 7919 * e)
        spec = sample_episode(cat, rng, seed=10_000 * (ci + 1) + e)
        if (cat, spec.seed) in state["done"]:
            continue
        if lab is not None:
            lab.rows = []
        for m in methods:
            r = run_episode(spec, m, auditor=auditor, exact_scale=getattr(m, "exact_scale", None))
            r.decisions = []
            state["res"].append(r)
        if lab is not None:
            state["rows"].extend(lab.rows)
        state["done"].add((cat, spec.seed))
        if j % 5 == 0 or j == len(jobs) - 1:
            with open(path + ".tmp", "wb") as f:
                pickle.dump(state, f)
            os.replace(path + ".tmp", path)
            print(f"[{a.worker}] {j + 1}/{len(jobs)} {cat} e{e} {time.time() - t0:.0f}s", flush=True)
    with open(path + ".tmp", "wb") as f:
        pickle.dump(state, f)
    os.replace(path + ".tmp", path)
    print("done", len(state["res"]), len(state["rows"]), f"{time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
