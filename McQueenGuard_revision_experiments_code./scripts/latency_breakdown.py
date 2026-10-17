"""R2-C7: decomposition of decision latency into its components (run on an idle machine).

Learned path: feature extraction, MLP inference (NumPy / ONNX Runtime), familiarity gate,
admission + lexicographic selection, complete learned decision.  Exact path: pMDP instantiation
(valuation + PRISM model generation), PRISM parse, PRISM model construction + model checking,
and file/IPC overhead (from data/prism_validation.csv, warm PRISM server).
"""
import sys, os, json, time, random
import numpy as np
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
os.environ.setdefault("OMP_NUM_THREADS", "1")
import onnxruntime as ort
import pandas as pd
from mqg.verifier import LearnedVerifier
from mqg.config import NOMINAL, CATEGORIES, EPS_MAX
from mqg.world import sample_episode, features, query
from mqg.sim import run_episode
from mqg.controllers import McQueenGuard, LatencyModel, lexicographic_select
from mqg.pmdp import prism_model

N = 20000


def bench(fn, n=N, warm=500):
    for _ in range(warm):
        fn()
    ts = np.empty(n)
    for i in range(n):
        t0 = time.perf_counter_ns()
        fn()
        ts[i] = time.perf_counter_ns() - t0
    us = ts / 1000.0
    return dict(mean_us=float(us.mean()), p50_us=float(np.median(us)), p99_us=float(np.percentile(us, 99)))


def main():
    ver = LearnedVerifier.load("models/verifier")
    obs_list = []

    class Rec(McQueenGuard):
        def decide(self, obs):
            obs_list.append(obs)
            return super().decide(obs)
    rec = Rec(ver, LatencyModel("data/prism_latency.json"))
    rng = random.Random(11)
    for k, c in enumerate(CATEGORIES):
        run_episode(sample_episode(c.name, rng, seed=880 + k), rec)
    rng2 = random.Random(5)
    rng2.shuffle(obs_list)
    obs_list = obs_list[:3000]
    X = [features(o) for o in obs_list]
    Q = [query(o) for o in obs_list]
    H = [ver.predict_one(x) for x in X]
    it = {"i": 0}

    def nxt():
        it["i"] = (it["i"] + 1) % len(obs_list)
        return it["i"]
    so = ort.SessionOptions(); so.intra_op_num_threads = 1; so.inter_op_num_threads = 1
    s32 = ort.InferenceSession("models/verifier.onnx", so, providers=["CPUExecutionProvider"])
    Xf = [x.astype(np.float32)[None] for x in X]

    def mlp_np(x):
        h = (x - ver.mu) / ver.sd
        for i in range(len(ver.W)):
            h = ver.W[i] @ h + ver.b[i]
            if i < len(ver.W) - 1:
                np.maximum(h, 0.0, out=h)
        return h

    def gate(x):
        z = ((x - ver.mu) / ver.sd)[~ver.const] - ver.gate_mean
        return float(z @ ver.gate_prec @ z) <= ver.gate_thr and bool(np.all((x >= ver.env_lo) & (x <= ver.env_hi)))

    def admission(i):
        ph, uh, fam = H[i]
        a, ok = lexicographic_select(ph, uh, ver.eps)
        adm = fam and ver.eps <= EPS_MAX and ok
        return Q[i].action_acc()[a] if adm else None

    def full_path(o):
        x = features(o)
        ph, uh, fam = ver.predict_one(x)
        q = query(o)
        a, ok = lexicographic_select(ph, uh, ver.eps)
        return q.action_acc()[a] if (fam and ok) else None

    res = {}
    res["features"] = bench(lambda: features(obs_list[nxt()]))
    res["mlp_numpy"] = bench(lambda: mlp_np(X[nxt()]))
    res["mlp_onnx_fp32"] = bench(lambda: s32.run(None, {"x": Xf[nxt()]}))
    res["gate"] = bench(lambda: gate(X[nxt()]))
    res["admission_select"] = bench(lambda: admission(nxt()))
    res["valuation_query"] = bench(lambda: query(obs_list[nxt()]))
    res["learned_path"] = bench(lambda: full_path(obs_list[nxt()]))
    res["prism_model_generation"] = bench(lambda: prism_model(Q[nxt()]), n=3000, warm=100)
    pv = pd.read_csv("data/prism_validation.csv")
    res["prism"] = dict(wall_mean_ms=float(pv.prism_ms.mean()), wall_p99_ms=float(np.percentile(pv.prism_ms, 99)),
                        parse_mean_ms=float(pv.parse_ms.mean()), check_mean_ms=float(pv.check_ms.mean()),
                        io_mean_ms=float((pv.prism_ms - pv.parse_ms - pv.check_ms).mean()),
                        parse_p99_ms=float(np.percentile(pv.parse_ms, 99)), check_p99_ms=float(np.percentile(pv.check_ms, 99)),
                        n=int(len(pv)))
    json.dump(res, open("results/latency_breakdown.json", "w"), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
