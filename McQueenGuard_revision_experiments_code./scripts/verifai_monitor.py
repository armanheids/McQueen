"""VerifAI baseline: falsification-driven data generation + learned runtime monitor.

1. Scenario space defined with VerifAI features; VerifAI's cross-entropy sampler proposes scenarios.
2. The learning-enabled controller (PPO planner) is simulated; the MTL-style safety specification
   G(gap >= d_SM) (and no collision) is monitored with a VerifAI specification monitor; the
   robustness value is fed back to the sampler (falsification).
3. A decision-tree monitor is learned on per-tick features to predict a violation within 2 s
   (Torfah et al., RV 2021); its threshold is chosen for >= 95 % recall on held-out episodes.
"""
import sys, random, math, time
import numpy as np
import joblib
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from verifai.features import Feature, FeatureSpace, Box, Categorical
from verifai.samplers import FeatureSampler
from verifai.monitor import specification_monitor
from sklearn.tree import DecisionTreeClassifier
from sklearn.metrics import recall_score, precision_score

from mqg.config import WEATHERS, DT_DEC, DT_SIM, D_SM
from mqg.world import sample_episode, World, Perception, features
from mqg.baselines import PPOPlanner
from dataclasses import replace

N_EP = int(sys.argv[1]) if len(sys.argv) > 1 else 800
HORIZON = 2.0

space = FeatureSpace({
    "weather": Feature(Categorical("clear", "rain", "fog")),
    "v0": Feature(Box([10.0, 50.0])),
    "tau": Feature(Box([1.6, 5.0])),
    "ped_v": Feature(Box([1.0, 1.8])),
    "cross": Feature(Box([0.0, 1.0])),
    "hes": Feature(Box([0.0, 1.0])),
    "y0": Feature(Box([-1.6, -0.8])),
})


class SafetySpec(specification_monitor):
    """rho = min over the trace of (gap - d_SM) while the pedestrian is in the lane; -10 on collision."""
    def __init__(self):
        def spec(traj):
            if traj["collided"]:
                return -10.0
            return traj["min_gap"] - D_SM if math.isfinite(traj["min_gap"]) else 5.0
        super().__init__(spec)


def simulate(spec, ppo):
    w = World(spec)
    per = Perception(w)
    feats, times = [], []
    acc = 0.0
    nxt = 0.0
    viol_t = None
    while not w.done():
        obs = per.tick()
        if w.t + 1e-9 >= nxt:
            acc = ppo.act(obs)
            nxt = w.t + DT_DEC
        feats.append(features(obs))
        times.append(w.t)
        w.step(acc)
        if viol_t is None and (w.collided or w.margin_violation):
            viol_t = w.t
    t = np.array(times)
    y = np.zeros(len(t), dtype=int) if viol_t is None else (t >= viol_t - HORIZON).astype(int)
    return np.array(feats), y, dict(collided=w.collided, min_gap=w.min_gap)


def main():
    ppo = PPOPlanner("models/ppo_planner")
    sampler = FeatureSampler.crossEntropySamplerFor(space)
    it = iter(sampler)
    smp = next(it)
    monitor = SafetySpec()
    rng = random.Random(77)
    X, Y, G = [], [], []
    rhos = []
    t0 = time.time()
    for e in range(N_EP):
        base = sample_episode("Clear (10-30)", rng, seed=700000 + e)
        w0 = WEATHERS[smp.weather]
        wth = replace(w0, friction=max(0.15, w0.friction + rng.uniform(-0.05, 0.05)),
                      vis_range=w0.vis_range * rng.uniform(0.85, 1.15))
        hes = smp.hes[0] > 0.7
        spec = replace(base, category="VerifAI-falsification", weather=wth, v0=smp.v0[0] / 3.6,
                       trig_tau=smp.tau[0], ped_v=smp.ped_v[0], ped_cross=bool(smp.cross[0] > 0.25),
                       hes_y=(0.3 + 2.7 * rng.random()) if hes else None, hes_dur=rng.uniform(0.5, 1.5) if hes else 0.0,
                       ped_y0=smp.y0[0])
        f, y, traj = simulate(spec, ppo)
        rho = monitor.evaluate(traj)
        rhos.append(rho)
        X.append(f); Y.append(y); G.append(np.full(len(y), e))
        smp = it.send(rho)
        if e % 100 == 0:
            print(f"falsification ep {e}: violations so far {np.mean(np.array(rhos) < 0):.3f} ({time.time()-t0:.0f}s)", flush=True)
    X = np.concatenate(X); Y = np.concatenate(Y); G = np.concatenate(G)
    tr = G < int(0.8 * N_EP)
    clf = DecisionTreeClassifier(max_depth=12, min_samples_leaf=20, class_weight="balanced", random_state=0)
    clf.fit(X[tr], Y[tr])
    p = clf.predict_proba(X[~tr])[:, 1]
    thr = 0.5
    for cand in np.linspace(0.95, 0.05, 91):
        if recall_score(Y[~tr], p >= cand) >= 0.95:
            thr = cand
            break
    rec = recall_score(Y[~tr], p >= thr)
    prec = precision_score(Y[~tr], p >= thr)
    joblib.dump(dict(clf=clf, thr=float(thr), feat_idx=np.arange(X.shape[1])), "models/verifai_monitor.joblib")
    np.savez("data/verifai_falsification.npz", rho=np.array(rhos))
    print(f"episodes {N_EP}, falsified (rho<0) {np.mean(np.array(rhos) < 0):.3f}; monitor thr {thr:.2f} recall {rec:.3f} precision {prec:.3f}")


if __name__ == "__main__":
    main()
