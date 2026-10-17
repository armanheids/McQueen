"""Comment 6: deployment profile of the learned verifier vs exact verification.

Measured on the testbed CPU (single thread): latency of the learned verifier in four runtimes
(NumPy, PyTorch eager, ONNX Runtime FP32, ONNX Runtime INT8), latency of the complete learned
decision path, exact PRISM verification latency, model and process memory footprints.
Projected for NVIDIA Jetson AGX Orin (Cortex-A78AE @ 2.2 GHz, 15-60 W) with an explicit CPU
slow-down band, and an upper bound on energy per decision from the module power envelope.
"""
import sys, os, json, time, random, subprocess, resource, gc
import numpy as np
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
os.environ.setdefault("OMP_NUM_THREADS", "1")
import torch
torch.set_num_threads(1)
import onnxruntime as ort
from onnxruntime.quantization import quantize_dynamic, QuantType

from mqg.verifier import LearnedVerifier, admissible_residual, conformal_eps
from mqg.config import NOMINAL, ALPHA_CONF, THETA
from mqg.world import sample_episode, features, query
from mqg.sim import run_episode
from mqg.controllers import McQueenGuard, LatencyModel, lexicographic_select

OUT = "results/embedded_profile.json"
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
    return dict(mean_us=float(us.mean()), p50_us=float(np.median(us)), p99_us=float(np.percentile(us, 99)),
                max_us=float(us.max()))


def rss_of(code):
    """Peak RSS (MB) of a fresh Python process executing `code`, minus that of an empty interpreter."""
    base = subprocess.run([sys.executable, "-c", "import resource;print(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)"],
                          capture_output=True, text=True).stdout.strip()
    out = subprocess.run([sys.executable, "-c", code + "\nimport resource;print(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)"],
                         capture_output=True, text=True, cwd=os.getcwd()).stdout.strip().splitlines()[-1]
    return int(out) / 1024.0, int(base) / 1024.0


def main():
    ver = LearnedVerifier.load("models/verifier")
    import glob
    D = [np.load(f) for f in sorted(glob.glob("data/d0_*.npz"))]
    X = np.concatenate([d["X"] for d in D]).astype(np.float64)
    P = np.concatenate([d["P"] for d in D])
    ep = np.concatenate([d["ep"] for d in D])
    split = np.load("data/split_ids.npz")
    cal, te = np.isin(ep, split["cal"]), np.isin(ep, split["te"])
    x1 = X[te][0].copy()
    xf = x1.astype(np.float32)[None]
    res = {}

    # ---------------- model size / compute
    n_params = sum(p.numel() for p in ver.model.parameters())
    macs = sum(w.shape[0] * w.shape[1] for w in ver.W)
    res["model"] = dict(params=int(n_params), macs=int(macs), flops=int(2 * macs),
                        fp32_kib=n_params * 4 / 1024, int8_kib=n_params / 1024,
                        onnx_fp32_kib=os.path.getsize("models/verifier.onnx") / 1024)
    quantize_dynamic("models/verifier.onnx", "models/verifier_int8.onnx", weight_type=QuantType.QInt8)
    res["model"]["onnx_int8_kib"] = os.path.getsize("models/verifier_int8.onnx") / 1024

    # ---------------- latency of the verifier alone
    so = ort.SessionOptions()
    so.intra_op_num_threads = 1
    so.inter_op_num_threads = 1
    so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    s32 = ort.InferenceSession("models/verifier.onnx", so, providers=["CPUExecutionProvider"])
    s8 = ort.InferenceSession("models/verifier_int8.onnx", so, providers=["CPUExecutionProvider"])
    xt = torch.tensor(xf)
    mu_t, sd_t = torch.tensor(ver.mu, dtype=torch.float32), torch.tensor(ver.sd, dtype=torch.float32)

    def torch_fn():
        with torch.no_grad():
            ver.model((xt - mu_t) / sd_t)
    lat = {}
    lat["NumPy (FP64)"] = bench(lambda: ver.predict_one(x1))
    lat["PyTorch eager (FP32)"] = bench(torch_fn, n=10000)
    lat["ONNX Runtime (FP32)"] = bench(lambda: s32.run(None, {"x": xf}))
    lat["ONNX Runtime (INT8)"] = bench(lambda: s8.run(None, {"x": xf}))
    res["verifier_latency"] = lat

    # accuracy of the INT8 model (re-calibrated)
    Xc32 = X[cal].astype(np.float32)
    Xt32 = X[te].astype(np.float32)
    def ort_pred(sess, Xb):
        return np.vstack([sess.run(None, {"x": Xb[i:i + 1]})[0] for i in range(len(Xb))])
    yc8 = ort_pred(s8, Xc32); yt8 = ort_pred(s8, Xt32); yt32 = ort_pred(s32, Xt32)
    Pc8, Pt8, Pt32 = np.clip(yc8[:, :5], 0, 1), np.clip(yt8[:, :5], 0, 1), np.clip(yt32[:, :5], 0, 1)
    eps8 = conformal_eps(admissible_residual(P[cal], Pc8), ALPHA_CONF)
    res["int8_accuracy"] = dict(mae_fp32=float(np.abs(Pt32 - P[te]).mean()), mae_int8=float(np.abs(Pt8 - P[te]).mean()),
                                max_dev_int8_vs_fp32=float(np.abs(Pt8 - Pt32).max()), eps_fp32=float(ver.eps),
                                eps_int8=float(eps8),
                                coverage_int8=float(np.mean(admissible_residual(P[te], Pt8) <= eps8)))

    # ---------------- complete learned decision path on recorded runtime observations
    obs_list = []

    class Rec(McQueenGuard):
        def decide(self, obs):
            obs_list.append(obs)
            return super().decide(obs)
    lm = LatencyModel("data/prism_latency.json")
    rec = Rec(ver, lm)
    rng = random.Random(3)
    for k in range(6):
        run_episode(sample_episode(NOMINAL[k].name, rng, seed=900 + k), rec)
    obs_list = obs_list[:2000]

    def path(o):
        x = features(o)
        ph, uh, fam = ver.predict_one(x)
        q = query(o)
        a, ok = lexicographic_select(ph, uh, ver.eps)
        return q.action_acc()[a]
    it = iter(range(10 ** 9))
    lat_path = bench(lambda: path(obs_list[next(it) % len(obs_list)]), n=N)
    res["learned_decision_path"] = lat_path

    # ---------------- exact verification (PRISM, warm server) from the latency model + numba reference
    pl = json.load(open("data/prism_latency.json"))
    allp = np.concatenate([np.array(v) for v in pl.values()])
    res["prism_latency_ms"] = dict(mean=float(allp.mean()), p50=float(np.median(allp)), p99=float(np.percentile(allp, 99)),
                                   max=float(allp.max()), n=int(len(allp)))

    # ---------------- memory footprints
    code_np = ("import numpy as np,sys;sys.path.insert(0,'.');from mqg.verifier import LearnedVerifier\n"
               "z=np.load('models/verifier.npz');import torch")
    code_ort = ("import onnxruntime as ort,numpy as np\nso=ort.SessionOptions();so.intra_op_num_threads=1\n"
                "s=ort.InferenceSession('models/verifier_int8.onnx',so,providers=['CPUExecutionProvider'])\n"
                "x=np.zeros((1,18),np.float32)\nfor i in range(1000): s.run(None,{'x':x})")
    code_ort32 = code_ort.replace("verifier_int8", "verifier")
    m_ort8, base = rss_of(code_ort)
    m_ort32, _ = rss_of(code_ort32)
    # PRISM verification server (JVM): sample RSS while verifying runtime queries
    import psutil
    from mqg.prism_engine import PrismEngine
    eng = PrismEngine("sparse")
    proc = psutil.Process(eng.p.pid)
    peak = 0
    qs = [query(o) for o in obs_list[::40][:40]]
    for q in qs:
        eng.verify(q)
        for c in [proc] + proc.children(recursive=True):
            try:
                peak = max(peak, c.memory_info().rss)
            except Exception:
                pass
    eng.close()
    res["memory_mb"] = dict(python_interpreter=base, ort_int8_process=m_ort8, ort_fp32_process=m_ort32,
                            ort_int8_increment=m_ort8 - base, ort_fp32_increment=m_ort32 - base,
                            prism_jvm_peak_rss=peak / 2 ** 20,
                            mlp_weights_fp32_kib=res["model"]["fp32_kib"], mlp_activations_bytes=4 * (64 + 64 + 32 + 10))

    # ---------------- projection to Jetson AGX Orin (assumption-driven)
    slow = dict(low=1.5, central=2.5, high=4.0)       # A78AE @2.2 GHz vs testbed core, single thread
    power = {"15 W": 15.0, "30 W": 30.0, "MAXN 60 W": 60.0}
    t_learn_ms = lat_path["mean_us"] / 1000.0
    t_learn_p99 = lat_path["p99_us"] / 1000.0
    t_ex_ms = res["prism_latency_ms"]["mean"]
    t_ex_p99 = res["prism_latency_ms"]["p99"]
    proj = {}
    for k, f in slow.items():
        proj[k] = dict(factor=f, learned_mean_ms=t_learn_ms * f, learned_p99_ms=t_learn_p99 * f,
                       exact_mean_ms=t_ex_ms * f, exact_p99_ms=t_ex_p99 * f,
                       energy_upper_mJ={pm: dict(learned=pw * t_learn_ms * f, exact=pw * t_ex_ms * f) for pm, pw in power.items()})
    res["orin_projection"] = proj
    json.dump(res, open(OUT, "w"), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
