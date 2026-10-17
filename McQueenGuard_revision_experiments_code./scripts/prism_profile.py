"""Validate the numba solver against PRISM and build the empirical PRISM latency model.

For every scenario category, decision points are collected from exact-mode closed-loop episodes;
each is verified by PRISM 4.8.1 (sparse engine, persistent warm JVM) and by the numba solver.
Outputs: data/prism_latency.json (ms per category), data/prism_validation.csv
"""
import sys, random, json, time, csv
import numpy as np
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg.config import CATEGORIES
from mqg.world import sample_episode, query
from mqg.sim import run_episode
from mqg.controllers import McQueenGuardExact, LatencyModel
from mqg.pmdp import ExactSolver
from mqg.prism_engine import PrismEngine

per_cat = int(sys.argv[1]) if len(sys.argv) > 1 else 60


class Collector(McQueenGuardExact):
    def __init__(self):
        super().__init__(LatencyModel(default=0.05))
        self.qs = []
        self.k = 0

    def decide(self, obs):
        self.k += 1
        if self.k % 7 == 0:
            self.qs.append(query(obs))
        return super().decide(obs)


def main():
    eng = PrismEngine("sparse")
    solver = ExactSolver()
    # warm-up the JVM (JIT) so that latencies reflect a running verification server
    rng = random.Random(0)
    warm = Collector()
    run_episode(sample_episode("Clear (35-50)", rng, seed=999), warm)
    for q in warm.qs[:25]:
        eng.verify(q)
    lat = {}
    rows = []
    for ci, cat in enumerate(CATEGORIES):
        col = Collector()
        rng = random.Random(5000 + ci)
        e = 0
        while len(col.qs) < per_cat:
            run_episode(sample_episode(cat.name, rng, seed=50000 + ci * 100 + e), col)
            e += 1
        qs = col.qs[:per_cat]
        lat[cat.name] = []
        for q in qs:
            pp, up, wall, t_parse, t_check = eng.verify(q)
            t0 = time.perf_counter()
            pn, un = solver.solve(q)
            tn = (time.perf_counter() - t0) * 1000
            lat[cat.name].append(wall)
            rows.append(dict(category=cat.name, prism_ms=wall, parse_ms=t_parse, check_ms=t_check, numba_ms=tn,
                             max_dP=float(np.abs(pp - pn).max()), max_dU=float(np.abs(up - un).max()),
                             det=q.det, d_hat=q.d_hat, v_hat=q.v_hat))
        a = np.array(lat[cat.name])
        print(f"{cat.name:20s} PRISM mean {a.mean():6.1f} ms p95 {np.percentile(a,95):6.1f}  maxdP {max(r['max_dP'] for r in rows if r['category']==cat.name):.1e}", flush=True)
    eng.close()
    json.dump(lat, open("data/prism_latency.json", "w"))
    with open("data/prism_validation.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print("max |dP| overall", max(r["max_dP"] for r in rows), "max |dU|", max(r["max_dU"] for r in rows))


if __name__ == "__main__":
    main()
