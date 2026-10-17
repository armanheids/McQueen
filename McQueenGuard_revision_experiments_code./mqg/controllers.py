"""Decision-making methods compared in the revision experiments."""
import math
import time
import json
import random
import numpy as np

from .config import (ACTIONS, THETA, G, LANE_W, PED_R, D_SM, DT_DEC, A_MAX_ACC, EPS_MAX, CATEGORIES)
from .pmdp import ExactSolver, Query
from .sim import Decision
from .world import d_stop, features, query, sigmoid

EMERGENCY = 4


def lexicographic_select(pu, U, eps=0.0, theta=THETA):
    """Safety-first lexicographic rule (Eqs. 16-17): admissible set, then max utility."""
    safe = [i for i in range(len(pu)) if pu[i] + eps < theta]
    if not safe:
        return EMERGENCY, False
    return max(safe, key=lambda i: U[i]), True


class LatencyModel:
    """Empirical latency of exact PRISM verification (warm JVM), sampled per scenario category."""

    def __init__(self, path=None, default=0.1):
        self.table = {}
        self.default = default
        self.rng = random.Random(12345)
        if path:
            with open(path) as f:
                self.table = {k: np.array(v) for k, v in json.load(f).items()}

    def sample(self, category):
        arr = self.table.get(category)
        if arr is None or len(arr) == 0:
            allv = np.concatenate(list(self.table.values())) if self.table else None
            if allv is None:
                return self.default
            arr = allv
        return float(arr[self.rng.randrange(len(arr))]) / 1000.0


class Base:
    name = "base"
    has_monitor = False

    def reset(self, spec):
        self.spec = spec

    def monitor(self, obs):
        return None


class SimplexMonitor:
    """Physical safety constraint of Section 4.5 with hysteresis (d_obs vs d_stop)."""

    def __init__(self, eps_h=0.15):
        self.eps_h = eps_h
        self.active = False

    def threat(self, obs):
        if not obs["tracked"]:
            return False
        y, vy = obs["y_hat"], obs["vy_hat"]
        in_lane = -PED_R - 0.2 <= y <= LANE_W + PED_R
        entering = (-1.2 <= y < -PED_R - 0.2) and vy > 0.3
        return in_lane or entering

    def __call__(self, obs):
        d_obs = obs["d_hat"] - PED_R
        if d_obs < -0.5:
            self.active = False
            return None
        thr = self.threat(obs)
        ds = d_stop(obs["v"], obs["mu_hat"], obs["latency"])
        if thr and d_obs <= ds:
            self.active = True
        elif self.active and (not thr or d_obs > (1 + self.eps_h) * ds):
            self.active = False
        if self.active:
            return -0.95 * obs["mu_hat"] * G
        return None


class McQueenGuardExact(Base):
    """McQueenGuard with exact verification at every decision point."""
    name = "McQueenGuard (exact only)"

    def __init__(self, latency_model: LatencyModel, solver=None):
        self.solver = solver or ExactSolver()
        self.lat = latency_model
        self.sm = SimplexMonitor()

    def reset(self, spec):
        super().reset(spec)
        self.sm = SimplexMonitor()

    def decide(self, obs):
        q = query(obs)
        pu, U = self.solver.solve(q)
        a, ok = lexicographic_select(pu, U)
        acc = q.action_acc()[a]
        le = self.lat.sample(self.spec.category)
        return Decision(acc=acc, latency=le, mode="exact" if ok else "fallback",
                        info=dict(action=a, pu=pu, U=U, exact_lat=le))

    def monitor(self, obs):
        return self.sm(obs)


class OnlineRecalibrator:
    """Shift-aware recalibration: rolling split-conformal margin over the most recent exact
    verification residuals (fallback results + a small rate of shadow audits of admitted decisions).
    The margin never drops below the offline-certified value."""

    def __init__(self, verifier, window=200, min_n=30, audit_rate=0.1, seed=0):
        from collections import deque
        self.ver = verifier
        self.res = deque(maxlen=window)
        self.min_n = min_n
        self.audit_rate = audit_rate
        self.rng = random.Random(seed)
        self._eps = verifier.eps

    def should_audit(self):
        return self.rng.random() < self.audit_rate

    def update(self, P, Ph):
        from .verifier import admissible_residual, conformal_eps
        self.res.append(float(admissible_residual(np.asarray(P)[None], np.asarray(Ph)[None])[0]))
        if len(self.res) >= self.min_n:
            self._eps = max(self.ver.eps, conformal_eps(np.array(self.res), self.ver.alpha))

    def eps(self):
        return self._eps


class McQueenGuard(Base):
    """Full McQueenGuard: learned verifier + error-aware admission + exact fallback + Simplex."""
    name = "McQueenGuard"

    def __init__(self, verifier, latency_model: LatencyModel, solver=None, admission=True,
                 name=None, online_recal=None, use_eps=True, use_gate=True, exact_fallback=True,
                 use_monitor=True):
        """Ablation switches: use_eps (conformal margin in Eq. 16), use_gate (familiarity set H),
        exact_fallback (PRISM on non-admitted decisions; otherwise the conservative fallback control
        is applied directly), use_monitor (Simplex shield of Section 4.5)."""
        self.ver = verifier
        self.solver = solver or ExactSolver()
        self.lat = latency_model
        self.admission = admission
        self.online_recal = online_recal
        self.use_eps, self.use_gate = use_eps, use_gate
        self.exact_fallback, self.use_monitor = exact_fallback, use_monitor
        if name:
            self.name = name
        self.sm = SimplexMonitor()

    def reset(self, spec):
        super().reset(spec)
        self.sm = SimplexMonitor()
        self.last_mode = "exact"

    def decide(self, obs):
        d = self._decide(obs)
        prev = self.last_mode
        self.last_mode = d.mode
        if d.mode != "learned" and prev != "learned":
            d.info.pop("provisional", None)   # held command was itself exactly verified: keep it
        return d

    def _decide(self, obs):
        t0 = time.perf_counter()
        x = features(obs)
        pu_hat, U_hat, familiar = self.ver.predict_one(x)
        q = query(obs)
        sm_active = self.sm.active
        if self.admission:
            eps = self.online_recal.eps() if self.online_recal is not None else self.ver.eps
            if not self.use_eps:
                eps = 0.0
            a, ok = lexicographic_select(pu_hat, U_hat, eps)
            fam = familiar if self.use_gate else True
            admitted = fam and eps <= EPS_MAX and ok and not sm_active
        else:
            eps = 0.0
            a, ok = lexicographic_select(pu_hat, U_hat, 0.0)
            admitted = True
        t_learn = time.perf_counter() - t0
        self._last = dict(eps=eps, sm_active=sm_active)
        if not admitted and not self.exact_fallback:
            acc_all = q.action_acc()
            acc = acc_all[EMERGENCY] if not ok else min(acc_all[a], acc_all[2])
            return Decision(acc=acc, latency=t_learn, mode="conservative",
                            info=dict(action=a, pu_hat=pu_hat, U_hat=U_hat, familiar=familiar, x=x))
        if admitted:
            if self.online_recal is not None and self.online_recal.should_audit():
                pu_s, _ = self.solver.solve(q)          # shadow audit, off the critical path
                self.online_recal.update(pu_s, pu_hat)
            return Decision(acc=q.action_acc()[a], latency=t_learn, mode="learned" if ok else "fallback",
                            info=dict(action=a, pu_hat=pu_hat, U_hat=U_hat, familiar=familiar, x=x))
        pu, U = self.solver.solve(q)
        if self.online_recal is not None:
            self.online_recal.update(pu, pu_hat)
        a2, ok2 = lexicographic_select(pu, U)
        le = self.lat.sample(self.spec.category)
        # Algorithm 1, line 26: while the exact result is pending the fallback safety control is active:
        # a learned command that has lost its admission is not kept; at least the reduce-speed action is
        # applied (the Simplex monitor may still command emergency braking)
        return Decision(acc=q.action_acc()[a2], latency=t_learn + le,
                        mode="exact" if ok2 else "fallback",
                        info=dict(action=a2, pu=pu, U=U, pu_hat=pu_hat, U_hat=U_hat, familiar=familiar, x=x,
                                  exact_lat=le, provisional=q.action_acc()[2], learned_action=a))

    def monitor(self, obs):
        return self.sm(obs) if self.use_monitor else None


class IDMRule(Base):
    """Rule-based planner: IDM car-following with a virtual stop line + TTC-triggered AEB."""
    name = "Rule-based (IDM + AEB)"

    def __init__(self, a_max=1.5, b=2.5, T=1.2, s0=3.0, ttc_aeb=1.4):
        self.a_max, self.b, self.T, self.s0, self.ttc_aeb = a_max, b, T, s0, ttc_aeb

    def decide(self, obs):
        t0 = time.perf_counter()
        v, v0 = obs["v"], max(1.0, obs["v_ref"])
        acc = self.a_max * (1 - (v / v0) ** 4)
        d_obs = obs["d_hat"] - PED_R
        if obs["tracked"] and d_obs > -0.5:
            y, vy = obs["y_hat"], obs["vy_hat"]
            in_lane = -PED_R - 0.2 <= y <= LANE_W + PED_R
            t_ped = ((-PED_R - y) / vy) if (vy > 0.3 and y < -PED_R) else math.inf
            t_ego = d_obs / max(v, 0.1)
            will_conflict = in_lane or (t_ped < t_ego + 1.5)
            if will_conflict:
                gap = max(0.1, d_obs - 1.0)
                s_star = self.s0 + v * self.T + v * v / (2 * math.sqrt(self.a_max * self.b))
                acc = self.a_max * (1 - (v / v0) ** 4 - (s_star / gap) ** 2)
                if t_ego < self.ttc_aeb:
                    acc = -0.95 * obs["mu_hat"] * G
        acc = max(-0.95 * obs["mu_hat"] * G, min(A_MAX_ACC, acc))
        return Decision(acc=acc, latency=time.perf_counter() - t0, mode="rule")
