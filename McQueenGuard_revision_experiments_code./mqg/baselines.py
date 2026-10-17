"""Baselines requested by Reviewer 1 (Comment 7).

* MPCPlanner      - receding-horizon collision-avoidance MPC following Zhang et al. (IEEE T-ITS 2022,
                    Eqs. 8-16, 19-20, 23-24) adapted to the longitudinal crossing scenario; solved with
                    CVXPY + OSQP as in the authors' public MPC-ROS code.
* PPOPlanner      - end-to-end RL planner (perception features -> acceleration), PPO (Stable-Baselines3)
                    trained with a CaRL-style simple reward (progress x soft penalties, termination on
                    collision).
* VerifAIRA       - VerifAI-based runtime assurance (Torfah et al., RV 2021): falsification with VerifAI's
                    cross-entropy sampler over the scenario space, MTL safety specification, a learned
                    (decision-tree) monitor predicting violations of the learning-enabled controller, and a
                    Simplex switch to a safe (braking) controller.
"""
import math
import time
import numpy as np

from .config import G, LANE_W, PED_R, D_SM, DT_DEC, A_MAX_ACC, DT_SIM, FEATURES
from .sim import Decision
from .world import features


class MPCPlanner:
    name = "MPC (Zhang et al. 2022)"

    def __init__(self, N=40, dt=0.1, Q=1.0, W=4.0, R=0.2, tau=0.6, d_min=D_SM + 1.0, rho=1e4):
        import cvxpy as cp
        self.N, self.dt, self.tau, self.d_min = N, dt, tau, d_min
        s = cp.Variable(N + 1)
        v = cp.Variable(N + 1)
        a = cp.Variable(N)
        sl = cp.Variable(N + 1, nonneg=True)
        self.p_v0 = cp.Parameter()
        self.p_vref = cp.Parameter(nonneg=True)
        self.p_aprev = cp.Parameter()
        self.p_amin = cp.Parameter()
        self.p_bound = cp.Parameter(N + 1)
        cons = [s[0] == 0, v[0] == self.p_v0]
        cost = 0
        for k in range(N):
            cons += [s[k + 1] == s[k] + v[k] * dt + 0.5 * a[k] * dt * dt,
                     v[k + 1] == v[k] + a[k] * dt]
            cost += Q * cp.square(v[k + 1] - self.p_vref) + R * cp.square(a[k])
            prev = self.p_aprev if k == 0 else a[k - 1]
            cost += W * cp.square(a[k] - prev)
            cons += [cp.abs(a[k] - prev) <= 2.0]
        cons += [v >= 0, v <= 17.0, a >= self.p_amin, a <= A_MAX_ACC]
        cons += [s + tau * v <= self.p_bound + sl]
        cost += rho * cp.sum(sl) + rho * cp.sum_squares(sl)
        self.prob = cp.Problem(cp.Minimize(cost), cons)
        self.a = a
        self.a_prev = 0.0
        self.cp = cp

    def reset(self, spec):
        self.spec = spec
        self.a_prev = 0.0

    def monitor(self, obs):
        return None

    def _occupancy(self, obs):
        """Constant-velocity prediction (Zhang et al. Eqs. 19-20) of the pedestrian over the horizon."""
        occ = np.zeros(self.N + 1, dtype=bool)
        if not obs["tracked"]:
            return occ
        y, vy = obs["y_hat"], obs["vy_hat"]
        m = PED_R + 0.3 + obs["sig_y"]
        t = np.arange(self.N + 1) * self.dt + obs["latency"]
        yk = y + vy * t
        occ = (yk >= -m) & (yk <= LANE_W + m)
        return occ

    def decide(self, obs):
        t0 = time.perf_counter()
        d_obs = obs["d_hat"] - PED_R
        occ = self._occupancy(obs)
        big = 1e3
        bound = np.where(occ & (d_obs > -0.5), d_obs - self.d_min, big)
        self.p_v0.value = max(0.0, obs["v"])
        self.p_vref.value = max(0.5, obs["v_ref"])
        # previous input = measured actual acceleration (stopped vehicle -> 0), keeps the jerk constraint feasible
        a_meas = 0.0 if obs["v"] < 0.2 else obs["a"]
        self.p_aprev.value = float(np.clip(a_meas, -0.95 * obs["mu_hat"] * G + 1e-3, A_MAX_ACC - 1e-3))
        self.p_amin.value = -0.95 * obs["mu_hat"] * G           # Eq. (14): a_max = mu g
        self.p_bound.value = bound
        try:
            self.prob.solve(solver=self.cp.OSQP, warm_start=True, eps_abs=1e-4, eps_rel=1e-4, max_iter=4000)
            acc = float(self.a.value[0]) if self.a.value is not None else (-0.95 * obs["mu_hat"] * G if obs["v"] > 0.2 else 0.0)
        except Exception:
            acc = -0.95 * obs["mu_hat"] * G
        self.a_prev = acc
        return Decision(acc=acc, latency=time.perf_counter() - t0, mode="mpc")


# ----------------------------------------------------------------------------- PPO environment
FEAT_SCALE = np.array([15, 5, 0.05, 0.5, 60, 4, 2, 1, 10, 1, 1, 1, 1, 0.5, 30, 40, 1, 4], dtype=np.float32)


def obs_vec(obs):
    f = np.clip(features(obs) / FEAT_SCALE, -5, 5)
    return np.append(f, obs["v_ref"] / 15.0).astype(np.float32)


import gymnasium as _gym


class CrossingEnv(_gym.Env):
    """Gymnasium environment for training the end-to-end PPO planner."""

    def __init__(self, cats, seed=0):
        from gymnasium import spaces
        super().__init__()
        self.cats = cats
        self.observation_space = spaces.Box(-5, 5, shape=(19,), dtype=np.float32)
        self.action_space = spaces.Box(-1, 1, shape=(1,), dtype=np.float32)
        self.rng = __import__("random").Random(seed)
        self.render_mode = None

    def reset(self, seed=None, options=None):
        from .world import sample_episode, World, Perception
        super().reset(seed=seed)
        if seed is not None:
            self.rng.seed(seed)
        cat = self.rng.choice(self.cats)
        self.ep = sample_episode(cat, self.rng)
        self.w = World(self.ep)
        self.per = Perception(self.w)
        self.obs = self.per.tick()
        self.prev_acc = 0.0
        return obs_vec(self.obs), {}

    def step(self, action):
        a = float(np.clip(action[0], -1, 1))
        mu = self.obs["mu_hat"]
        acc = a * 0.95 * mu * G if a < 0 else a * A_MAX_ACC
        x0 = self.w.x
        mv0 = self.w.margin_violation
        for _ in range(int(round(DT_DEC / DT_SIM))):
            self.w.step(acc)
            self.obs = self.per.tick()
            if self.w.done():
                break
        vref = max(1.0, self.ep.v0)
        prog = max(0.0, min(1.0, (self.w.x - x0) / (vref * DT_DEC)))
        factor = 1.0
        if self.w.v > 1.1 * vref:
            factor *= 0.5
        factor *= 1.0 - 0.5 * min(1.0, abs(acc - self.prev_acc) / 4.0)
        r_pen = 0.0
        if self.w.margin_violation and not mv0:
            factor *= 0.0
            r_pen = -20.0
        self.prev_acc = acc
        r = prog * factor + r_pen
        term = False
        if self.w.collided:
            r = -100.0
            term = True
        elif self.w.done():
            term = True
        return obs_vec(self.obs), r, term, False, {}


class PPOPlanner:
    name = "End-to-end RL (PPO)"

    def __init__(self, path):
        from stable_baselines3 import PPO
        self.model = PPO.load(path, device="cpu")
        pol = self.model.policy
        # extract the deterministic actor (mlp_extractor.policy_net + action_net) as numpy for speed
        import torch
        self.layers = []
        for m in list(pol.mlp_extractor.policy_net) + [pol.action_net]:
            if isinstance(m, torch.nn.Linear):
                self.layers.append((m.weight.detach().numpy().astype(np.float64), m.bias.detach().numpy().astype(np.float64)))
        self.act_fn = type(pol.mlp_extractor.policy_net[1]).__name__

    def reset(self, spec):
        self.spec = spec

    def monitor(self, obs):
        return None

    def act(self, obs):
        h = obs_vec(obs).astype(np.float64)
        for i, (W, b) in enumerate(self.layers):
            h = W @ h + b
            if i < len(self.layers) - 1:
                h = np.tanh(h) if self.act_fn == "Tanh" else np.maximum(h, 0)
        a = float(np.clip(h[0], -1, 1))
        mu = obs["mu_hat"]
        return a * 0.95 * mu * G if a < 0 else a * A_MAX_ACC

    def decide(self, obs):
        t0 = time.perf_counter()
        acc = self.act(obs)
        return Decision(acc=acc, latency=time.perf_counter() - t0, mode="rl")


class VerifAIRA:
    """VerifAI-style runtime assurance around the learning-enabled (PPO) controller."""
    name = "VerifAI runtime assurance"

    def __init__(self, ppo: PPOPlanner, monitor_path, hold=0.5):
        import joblib
        self.ppo = ppo
        bundle = joblib.load(monitor_path)
        self.clf = bundle["clf"]
        self.thr = bundle["thr"]
        self.feat_idx = bundle["feat_idx"]
        self.hold = hold
        self.tree = self.clf.tree_

    def reset(self, spec):
        self.spec = spec
        self.safe_until = -1.0
        self.t = 0.0

    def _p_unsafe(self, x):
        t = self.tree
        node = 0
        while t.children_left[node] != -1:
            node = t.children_left[node] if x[t.feature[node]] <= t.threshold[node] else t.children_right[node]
        v = t.value[node][0]
        return v[1] / v.sum()

    def monitor(self, obs):
        x = features(obs)[self.feat_idx]
        if self._p_unsafe(x) >= self.thr:
            self.safe_until = obs["t"] + self.hold
        if obs["t"] <= self.safe_until and obs["d_hat"] - PED_R > -0.5:
            return -0.95 * obs["mu_hat"] * G
        return None

    def decide(self, obs):
        t0 = time.perf_counter()
        acc = self.ppo.act(obs)
        _ = self._p_unsafe(features(obs)[self.feat_idx])
        return Decision(acc=acc, latency=time.perf_counter() - t0, mode="rl+monitor")
