"""Safe-DAgger-style aggregation round: states visited by McQueenGuard running the current learned
verifier are labelled by exact verification and aggregated into the verified dataset.
Usage: dagger_round.py <out.npz> <first_ep> <n_ep>
"""
import sys, random, time
import numpy as np
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg.config import NOMINAL
from mqg.world import sample_episode, features, query
from mqg.sim import run_episode
from mqg.controllers import McQueenGuard, LatencyModel
from mqg.verifier import LearnedVerifier


class Recorder(McQueenGuard):
    def __init__(self, *a, every=5, **k):
        super().__init__(*a, **k)
        self.rows, self.k, self.every = [], 0, every

    def decide(self, obs):
        d = super().decide(obs)
        self.k += 1
        if self.k % self.every == 0:
            pu, U = self.solver.solve(query(obs))
            self.rows.append((features(obs), pu, U, self.spec.category, self.spec.seed,
                              [obs["rho"], obs["stand_time"], obs["sig_y"], obs["conf"], float(obs["tracked"]),
                               obs["y_hat"], obs["vy_hat"], obs["pe_hat"]]))
        return d


def main():
    out, first, n = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    ver = LearnedVerifier.load("models/verifier")
    rec = Recorder(ver, LatencyModel(default=0.05))
    t0 = time.time()
    for ep in range(first, first + n):
        rng = random.Random(ep * 7919 + 3)
        spec = sample_episode(NOMINAL[ep % len(NOMINAL)].name, rng, seed=ep)
        run_episode(spec, rec)
        if (ep - first) % 20 == 0:
            print(f"ep {ep} rows {len(rec.rows)} {time.time() - t0:.0f}s", flush=True)
    rows = rec.rows
    np.savez_compressed(out, X=np.stack([r[0] for r in rows]), P=np.stack([r[1] for r in rows]),
                        U=np.stack([r[2] for r in rows]), cat=np.array([r[3] for r in rows]),
                        ep=np.array([r[4] for r in rows]), R=np.array([r[5] for r in rows]))
    print("done", len(rows))


if __name__ == "__main__":
    main()
