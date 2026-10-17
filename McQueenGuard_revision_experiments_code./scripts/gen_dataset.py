"""Generate the verified dataset D0 for the learned verifier (Algorithm 1, offline phase).

Decision points are visited by the exact-verification controller with 20 % exploratory actions
(Safe-DAgger-style state coverage) in the nominal scenario categories; every recorded decision
point is labelled with the exact property vector (p_unsafe(a), U(a)) for all five actions.
Usage: gen_dataset.py <out.npz> <first_episode> <n_episodes> [shift_kind severity]
"""
import sys, random, time
import numpy as np
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg.config import NOMINAL, CATEGORIES
from mqg.world import sample_episode, NoiseShift, features, query
from mqg.sim import run_episode, Decision
from mqg.controllers import McQueenGuardExact, LatencyModel


class Explorer(McQueenGuardExact):
    name = "explorer"

    def __init__(self, eps=0.2, every=5, seed=0, **kw):
        super().__init__(LatencyModel(default=0.05), **kw)
        self.eps = eps
        self.every = every
        self.rng = random.Random(seed)
        self.rows = []
        self.k = 0

    def decide(self, obs):
        d = super().decide(obs)
        self.k += 1
        if self.k % self.every == 0:
            self.rows.append((features(obs), d.info["pu"], d.info["U"], self.spec.category, self.spec.seed,
                              [obs["rho"], obs["stand_time"], obs["sig_y"], obs["conf"], float(obs["tracked"]), obs["y_hat"], obs["vy_hat"], obs["pe_hat"]]))
        if self.rng.random() < self.eps:
            q = query(obs)
            a = self.rng.randrange(5)
            return Decision(acc=q.action_acc()[a], latency=d.latency, mode="explore")
        return d


def main():
    out, first, n = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    cats = NOMINAL
    shift = None
    if len(sys.argv) > 4:
        shift = NoiseShift(sys.argv[4], float(sys.argv[5]))
        cats = [c for c in CATEGORIES if c.in_distribution]
    ex = Explorer(seed=first)
    t0 = time.time()
    for ep in range(first, first + n):
        rng = random.Random(ep * 9973 + 17)
        cat = cats[ep % len(cats)].name
        spec = sample_episode(cat, rng, seed=ep, shift=shift)
        run_episode(spec, ex)
        if (ep - first) % 20 == 0:
            print(f"ep {ep} rows {len(ex.rows)} {time.time() - t0:.0f}s", flush=True)
    X = np.stack([r[0] for r in ex.rows])
    P = np.stack([r[1] for r in ex.rows])
    U = np.stack([r[2] for r in ex.rows])
    cat = np.array([r[3] for r in ex.rows])
    ep = np.array([r[4] for r in ex.rows])
    R = np.array([r[5] for r in ex.rows])
    np.savez_compressed(out, X=X, P=P, U=U, cat=cat, ep=ep, R=R)
    print("done", X.shape, f"{time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
