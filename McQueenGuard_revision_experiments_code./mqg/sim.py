"""Closed-loop episode runner with computation-latency injection.

Every planner is invoked every DT_DEC seconds; its command takes effect only after its compute
latency has elapsed (the previous command is held meanwhile).  A new decision cannot start
before the previous one has finished.  Controllers may additionally expose a per-tick safety
monitor (Simplex), whose cost is negligible.
"""
import math
import time
from dataclasses import dataclass, field
import numpy as np

from .config import DT_SIM, DT_DEC, D_SM
from .world import World, Perception, EpisodeSpec


@dataclass
class Decision:
    acc: float
    latency: float            # seconds of compute to inject
    mode: str = "-"
    info: dict = field(default_factory=dict)


@dataclass
class EpisodeResult:
    category: str
    seed: int
    method: str
    collided: bool
    margin_violation: bool
    conflict: bool
    min_gap: float
    t_clear: float
    timeout: bool
    mean_speed: float
    max_decel: float
    rms_jerk: float
    n_dec: int
    lat_mean: float
    lat_p99: float
    lat_max: float
    miss_rate: float          # decisions whose latency exceeded DT_DEC
    n_learned: int
    n_exact: int
    n_fallback_ctrl: int
    n_override_ticks: int
    v0: float
    decisions: list = field(default_factory=list)
    audits: list = field(default_factory=list)
    lats: list = field(default_factory=list)
    n_conservative: int = 0


def run_episode(spec: EpisodeSpec, ctrl, platform_scale=1.0, record_decisions=False, record_trace=False,
                auditor=None, audit_every=1.0, exact_scale=None):
    w = World(spec)
    per = Perception(w)
    ctrl.reset(spec)
    cmd = 0.0
    pending = None              # (ready_time, acc)
    next_dec = 0.0
    lats = []
    modes = {"learned": 0, "exact": 0, "fallback": 0, "conservative": 0}
    overrides = 0
    decs = []
    trace = []
    audits = []
    next_audit = 0.5
    while not w.done():
        obs = per.tick()
        t = w.t
        if pending is not None and t + 1e-9 >= pending[0]:
            cmd = pending[1]
            pending = None
        if pending is None and t + 1e-9 >= next_dec:
            d = ctrl.decide(obs)
            if exact_scale is not None and "exact_lat" in d.info:
                lat = (d.latency - d.info["exact_lat"]) * platform_scale + d.info["exact_lat"] * exact_scale
            else:
                lat = d.latency * platform_scale
            lats.append(lat)
            if d.mode in modes:
                modes[d.mode] += 1
            if record_decisions:
                decs.append(dict(t=t, acc=d.acc, lat=lat, mode=d.mode, obs=obs, **d.info))
            if lat <= DT_SIM * 0.5:
                cmd = d.acc
            else:
                pending = (t + lat, d.acc)
                if "provisional" in d.info:
                    cmd = min(cmd, d.info["provisional"])
            next_dec = t + max(DT_DEC, lat)
        u = cmd
        ov = ctrl.monitor(obs)
        if ov is not None:
            u = ov
            overrides += 1
        if auditor is not None and t + 1e-9 >= next_audit and obs["d_hat"] > -1.0:
            audits.append(auditor(obs, u))
            next_audit = t + audit_every
        if record_trace:
            trace.append((t, w.x, w.v, w.a, w.py, u, obs["tracked"], obs["y_hat"]))
        w.step(u)
    lat_arr = np.array(lats) if lats else np.zeros(1)
    t_clear = w.passed_time if w.passed_time is not None else math.nan
    dist_travel = w.x + 70.0
    res = EpisodeResult(
        category=spec.category, seed=spec.seed, method=ctrl.name, collided=w.collided,
        margin_violation=w.margin_violation or w.collided, conflict=w.conflict,
        min_gap=(0.0 if w.collided else (w.min_gap if w.conflict else math.nan)),
        t_clear=t_clear, timeout=(not w.collided and w.passed_time is None),
        mean_speed=dist_travel / max(w.t, 1e-6), max_decel=w.max_decel,
        rms_jerk=math.sqrt(w.jerk_sq / max(1, w.n_ticks)), n_dec=len(lats),
        lat_mean=float(lat_arr.mean() * 1000), lat_p99=float(np.percentile(lat_arr, 99) * 1000),
        lat_max=float(lat_arr.max() * 1000), miss_rate=float((lat_arr > DT_DEC).mean()),
        n_learned=modes["learned"], n_exact=modes["exact"], n_fallback_ctrl=modes["fallback"],
        n_override_ticks=overrides, v0=spec.v0, decisions=decs, audits=audits, lats=lats,
        n_conservative=modes["conservative"])
    if record_trace:
        res.trace = trace
    return res
