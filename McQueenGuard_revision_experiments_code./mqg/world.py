"""Ground-truth pedestrian-crossing simulator with weather, sensing and actuation imperfections.

Longitudinal ego dynamics with 0.1 s actuation delay + 0.2 s first-order lag and a friction limit;
a pedestrian crossing at a marked crosswalk (crossing line at s = 0) with stochastic intent,
speed and hesitation; a perception pipeline with distance-/weather-dependent detection, track
drop-outs, Gaussian (or shifted, heavy-tailed) measurement noise and perception latency.
"""
import math
import random
from dataclasses import dataclass, replace, field
from collections import deque
import numpy as np

from .config import (G, DT_SIM, CAR_LEN, LANE_W, PED_R, D_SM, WEATHERS, CAT_BY_NAME, Weather,
                     FEATURES, A_MAX_ACC, H_MDP, DT_MDP)
from .pmdp import Query

D0 = 70.0          # initial distance of the ego front to the crossing line [m]
X_END = 12.0       # episode ends when ego front is X_END past the crossing line
T_MAX = None  # per episode: D0/v0 + 30 s


def sigmoid(z):
    if z < -40:
        return 0.0
    if z > 40:
        return 1.0
    return 1.0 / (1.0 + math.exp(-z))


@dataclass
class NoiseShift:
    """Sim-to-real perturbation of the perception noise model (Comment 6 experiment)."""
    kind: str = "none"         # none | heavy_tail | bias | dropout | latency | combined
    severity: float = 0.0      # 0..1
    def scale(self):
        return 1.0 + 1.5 * self.severity if self.kind in ("heavy_tail", "combined") else 1.0


@dataclass
class EpisodeSpec:
    category: str
    weather: Weather
    v0: float                  # desired / initial speed [m/s]
    ped_cross: bool
    ped_y0: float
    trig_tau: float            # ped starts when ego distance <= v0 * trig_tau
    ped_v: float
    hes_y: float               # lateral position at which the pedestrian hesitates (or None)
    hes_dur: float
    occluded: bool
    noise_scale: float
    extra_dropout: float
    eta_brake: float           # true brake efficiency
    mu_bias: float
    vis_bias: float
    seed: int
    shift: NoiseShift = field(default_factory=NoiseShift)


def sample_episode(cat_name, rng: random.Random, seed=None, shift=None):
    cat = CAT_BY_NAME[cat_name]
    wname = rng.choice(cat.weathers)
    w0 = WEATHERS[wname]
    w = replace(w0,
                friction=max(0.15, w0.friction + rng.uniform(-0.05, 0.05)),
                vis_range=w0.vis_range * rng.uniform(0.85, 1.15),
                cam_noise=w0.cam_noise * rng.uniform(0.9, 1.1),
                lidar_acc=min(0.99, w0.lidar_acc * rng.uniform(0.98, 1.02)),
                latency_ms=w0.latency_ms + rng.uniform(-15, 15))
    v0 = rng.uniform(*cat.v_range) / 3.6
    if cat.ped_mode == "novel":
        runner = rng.random() < 0.5
        spec = dict(ped_cross=True, ped_y0=rng.uniform(-1.4, -0.9),
                    trig_tau=rng.uniform(1.4, 3.2) if runner else rng.uniform(1.6, 3.6),
                    ped_v=rng.uniform(2.5, 3.5) if runner else rng.uniform(1.2, 2.0),
                    hes_y=None, hes_dur=0.0, occluded=not runner)
    else:
        cross = rng.random() < 0.75
        hes = rng.random() < 0.3
        spec = dict(ped_cross=cross, ped_y0=rng.uniform(-1.6, -0.8), trig_tau=rng.uniform(1.6, 5.0),
                    ped_v=rng.uniform(1.0, 1.8), hes_y=rng.uniform(0.3, 3.0) if hes else None,
                    hes_dur=rng.uniform(0.5, 1.5) if hes else 0.0, occluded=False)
    return EpisodeSpec(category=cat_name, weather=w, v0=v0, noise_scale=cat.noise_scale,
                       extra_dropout=cat.extra_dropout, eta_brake=rng.uniform(0.9, 1.0),
                       mu_bias=rng.gauss(0.0, 0.03), vis_bias=rng.gauss(0.0, 0.08),
                       seed=seed if seed is not None else rng.randrange(1 << 30),
                       shift=shift or NoiseShift(), **spec)


def perception_error(w: Weather, noise_scale=1.0):
    pe = 0.10 * w.rain + 0.15 * w.fog + 0.15 * w.snow + 0.35 * w.cam_noise * noise_scale + 0.45 * (1 - w.lidar_acc)
    return min(0.6, max(0.02, pe))


class World:
    def __init__(self, spec: EpisodeSpec):
        self.s = spec
        self.rng = random.Random(spec.seed)
        w = spec.weather
        self.mu = w.friction
        self.t = 0.0
        self.x = -D0               # ego front position (crossing line at 0)
        self.v = spec.v0
        self.a = 0.0
        self.cmd_buf = deque([0.0, 0.0])   # 0.1 s pure actuation delay
        self.tau_act = 0.2
        # pedestrian
        self.py = spec.ped_y0
        self.pstate = "wait"       # wait | walk | hesitate | done | stand
        self.hes_left = spec.hes_dur
        self.stop_wait = 0.0
        self.collided = False
        self.min_gap = math.inf
        self.margin_violation = False
        self.conflict = False
        self.passed_time = None
        self.max_decel = 0.0
        self.jerk_sq = 0.0
        self.n_ticks = 0
        self.prev_a = 0.0

    # ---------------------------------------------------------------- truth
    def ped_in_lane(self, margin=0.0):
        return -PED_R - margin <= self.py <= LANE_W + PED_R + margin

    def step(self, u):
        s = self.s
        dt = DT_SIM
        a_min = -self.mu * G * s.eta_brake
        u = max(min(u, A_MAX_ACC), -12.0)
        self.cmd_buf.append(u)
        ud = self.cmd_buf.popleft()
        self.a += (ud - self.a) * dt / self.tau_act
        self.a = max(a_min, min(A_MAX_ACC, self.a))
        v_new = self.v + self.a * dt
        if v_new < 0:
            self.x += self.v * self.v / (2 * max(1e-6, -self.a)) if self.a < 0 else 0.0
            self.v = 0.0
            self.a = 0.0 if self.a < 0 else self.a
        else:
            self.x += self.v * dt + 0.5 * self.a * dt * dt
            self.v = v_new
        self.jerk_sq += ((self.a - self.prev_a) / dt) ** 2
        self.prev_a = self.a
        self.max_decel = max(self.max_decel, -self.a)
        # pedestrian
        d_ego = -self.x
        # yielding interaction: once the vehicle has (almost) stopped in front of the crosswalk,
        # a waiting pedestrian crosses
        if self.pstate in ("wait", "stand") and self.v < 0.8 and 0.0 < d_ego < 25.0:
            self.stop_wait += dt
        elif self.pstate in ("wait", "stand"):
            self.stop_wait = max(0.0, self.stop_wait - dt)
        if self.pstate == "wait":
            if d_ego <= s.v0 * s.trig_tau:
                self.pstate = "walk" if s.ped_cross else "stand"
            elif self.stop_wait > 1.5:
                self.pstate = "walk"
        elif self.pstate == "stand":
            self.py = min(-0.6, self.py + 0.3 * dt)       # steps to the kerb and waits
            if self.stop_wait > 1.5:
                self.pstate = "walk"
        elif self.pstate == "walk":
            self.py += s.ped_v * dt
            if s.hes_y is not None and self.hes_left > 0 and self.py >= s.hes_y:
                self.pstate = "hesitate"
            if self.py > LANE_W + 1.0:
                self.pstate = "done"
        elif self.pstate == "hesitate":
            self.hes_left -= dt
            if self.hes_left <= 0:
                self.pstate = "walk"
        self.t += dt
        self.n_ticks += 1
        # safety bookkeeping
        front, rear = self.x, self.x - CAR_LEN
        if self.ped_in_lane():
            if front >= -PED_R and rear <= PED_R:
                self.collided = True
            if front < -PED_R:
                gap = -PED_R - front
                self.conflict = True
                self.min_gap = min(self.min_gap, gap)
                if gap < D_SM:
                    self.margin_violation = True
        if self.passed_time is None and rear > PED_R:
            self.passed_time = self.t

    def done(self):
        return self.collided or self.x >= X_END or self.t >= D0 / max(self.s.v0, 1.0) + 30.0


class Perception:
    """Produces (delayed) perception snapshots of the world."""

    def __init__(self, world: World):
        self.w = world
        s = world.s
        self.rng = random.Random(s.seed * 7 + 3)
        we = s.weather
        self.pe = perception_error(we, s.noise_scale)
        self.p_drop = min(0.8, 0.3 * self.pe + s.extra_dropout + (0.25 * s.shift.severity if s.shift.kind in ("dropout", "combined") else 0.0))
        lat = we.latency_ms + (120.0 * s.shift.severity if s.shift.kind in ("latency", "combined") else 0.0)
        self.lat_ticks = max(0, int(round(lat / 1000.0 / DT_SIM)))
        self.sig_d0 = (0.10 + 1.5 * (1 - we.lidar_acc) + 0.3 * we.cam_noise) * s.noise_scale
        self.sig_y0 = (0.05 + 0.6 * we.cam_noise) * s.noise_scale
        self.tracked = False
        self.conf = 0.0
        self.y_est = None
        self.vy_est = 0.0
        self.miss = 0
        self.hist = deque(maxlen=self.lat_ticks + 1)
        # self-assessed context (estimates)
        self.mu_hat = max(0.1, world.mu + s.mu_bias)
        self.vis_hat = we.vis_range * (1 + s.vis_bias)
        self.pe_hat = min(0.6, max(0.02, self.pe * (1 + self.rng.gauss(0, 0.08))))
        self.noise_level = we.cam_noise * s.noise_scale
        self.weather_code = we.code
        self.rho = 0.3            # presence belief for an undetected pedestrian at the crosswalk
        self.stand_time = 0.0     # time the tracked pedestrian has been standing at the kerb
        self.cleared = False

    def _noise(self, sigma):
        s = self.w.s.shift
        if s.kind in ("heavy_tail", "combined") and s.severity > 0:
            nu = max(1.5, 6.0 - 4.0 * s.severity)          # Student-t, heavier tails with severity
            z = self.rng.gauss(0, 1) / math.sqrt(max(1e-6, self.rng.gammavariate(nu / 2, 2.0 / nu)))
            return sigma * s.scale() * z
        return self.rng.gauss(0, sigma)

    def _bias(self):
        s = self.w.s.shift
        return (0.6 * s.severity) if s.kind in ("bias", "combined") else 0.0

    def tick(self):
        w = self.w
        s = w.s
        we = s.weather
        dx = -w.x
        lat_off = w.py - LANE_W / 2
        dist = math.hypot(max(dx, 0.0), lat_off)
        visible = not (s.occluded and w.py < -0.3) and w.pstate != "done" and dx > -CAR_LEN
        meas = None
        if visible:
            p_frame = (1 - self.pe) * sigmoid((we.vis_range - dist) / 4.0)
            if not self.tracked:
                if self.rng.random() < p_frame:
                    self.tracked = True
                    meas = True
            else:
                meas = self.rng.random() >= self.p_drop
        if self.tracked:
            sig_y = self.sig_y0 * (0.5 + dist / 40.0)
            if meas:
                y_m = w.py + self._noise(sig_y) - self._bias()
                if self.y_est is None:
                    self.y_est, self.vy_est = y_m, 0.0
                else:
                    pred = self.y_est + self.vy_est * DT_SIM
                    r = y_m - pred
                    self.y_est = pred + 0.30 * r
                    self.vy_est = self.vy_est + 0.04 * r / DT_SIM
                    self.vy_est = max(-4.0, min(5.0, self.vy_est))
                self.miss = 0
                self.conf = min(1.0, 0.6 * self.conf + 0.4 * (0.55 + 0.45 * sigmoid((we.vis_range - dist) / 6.0)) * (1 - 0.5 * self.pe))
            else:
                self.miss += 1
                self.y_est = self.y_est + self.vy_est * DT_SIM if self.y_est is not None else None
                self.conf *= 0.92
                if self.miss > 30 or (self.y_est is not None and self.y_est > LANE_W + 1.2):
                    self.tracked = (self.y_est is not None and self.y_est <= LANE_W + 1.2) and self.miss <= 30
                    if not self.tracked:
                        self.conf = 0.0
                        if not self.cleared:
                            self.rho = 0.6      # a pedestrian was there and is no longer observed
            sig_track = sig_y * 0.7 + 0.05 * self.miss
            if self.y_est is not None and self.y_est > LANE_W + 0.6:
                self.cleared = True
            if self.y_est is not None and abs(self.vy_est) < 0.3 and self.y_est < -PED_R:
                self.stand_time += DT_SIM
            else:
                self.stand_time = 0.0
        else:
            sig_track = 1.0
            # Bayesian presence belief: no detection although the crosswalk is observable
            pf_hat = (1 - self.pe_hat) * sigmoid((self.vis_hat - max(dx, 0.0)) / 4.0)
            r = self.rho * (1 - pf_hat)
            self.rho = r / (r + (1 - self.rho))
            self.rho = self.rho + 0.002 * (1 - self.rho)      # hazard of an unseen arrival
            if self.cleared:
                self.rho = min(self.rho, 0.02)
        snap = dict(t=w.t, d_hat=dx + self.rng.gauss(0, 0.2), tracked=self.tracked and self.y_est is not None,
                    y_hat=self.y_est if self.y_est is not None else -6.0, vy_hat=self.vy_est if self.tracked else 0.0,
                    conf=self.conf if self.tracked else 0.0, sig_y=sig_track, rho=self.rho,
                    stand_time=self.stand_time)
        self.hist.append(snap)
        old = self.hist[0]
        # ego proprioception is fast (CAN); pedestrian perception arrives with latency
        return dict(old, v=max(0.0, w.v + self.rng.gauss(0, 0.05)), a=w.a + self.rng.gauss(0, 0.1),
                    d_hat=dx + self.rng.gauss(0, 0.2), mu_hat=self.mu_hat, vis_hat=self.vis_hat, pe_hat=self.pe_hat,
                    noise_level=self.noise_level, weather_code=self.weather_code, v_ref=s.v0,
                    yaw=self.rng.gauss(0, 0.005), lane_dev=self.rng.gauss(0, 0.05), latency=self.lat_ticks * DT_SIM)


T_REACT = 0.3


def d_stop(v, mu, latency=0.1):
    return v * (T_REACT + latency) + v * v / (2 * max(0.05, mu) * G) + D_SM


def features(obs):
    v = obs["v"]
    mu = obs["mu_hat"]
    tracked = obs["tracked"]
    d = obs["d_hat"]
    ped_lat = (obs["y_hat"] - LANE_W / 2) if tracked else -6.0
    # NOTE: in this single-lane longitudinal testbed the yaw-angle and lane-deviation slots of the
    # Section 6.6 feature vector carry the pedestrian existence belief and the kerb waiting time, and
    # the relative-vehicle-velocity slot is the ego speed relative to the reference traffic speed
    exist = obs["conf"] if tracked else obs.get("rho", 0.3)
    f = [v, obs["a"], exist, min(10.0, obs.get("stand_time", 0.0)), max(-10.0, min(80.0, d)), ped_lat,
         obs["vy_hat"] if tracked else 0.0, v - obs["v_ref"], min(10.0, max(0.0, d) / max(v, 0.1)),
         obs["conf"], obs["vis_hat"] / 100.0, obs["weather_code"], mu, obs["noise_level"],
         v * v / (2 * mu * G), max(-30.0, min(80.0, d - d_stop(v, mu, obs["latency"]))), 0.0, H_MDP * DT_MDP]
    return np.array(f, dtype=np.float32)


P_GO0 = 0.12


def p_go_hat(stand_time):
    """Crossing-intent estimate: a pedestrian who keeps waiting at the kerb is increasingly likely
    to be yielding to the vehicle."""
    if stand_time <= 1.5:
        return P_GO0
    return max(0.02, P_GO0 * math.exp(-(stand_time - 1.5) / 2.0))


def query(obs):
    tracked = obs["tracked"]
    vy = obs["vy_hat"] if tracked else 0.0
    p_go = p_go_hat(obs.get("stand_time", 0.0)) if tracked else P_GO0
    rho = obs.get("rho", 0.3)
    return Query(mu=obs["mu_hat"], pe=obs["pe_hat"], r_vis=obs["vis_hat"],
                 v_ped=max(0.8, min(3.5, abs(vy))) if abs(vy) > 0.4 else 1.4, p_go=p_go,
                 v_ref=max(1.0, obs["v_ref"]),
                 d_hat=obs["d_hat"] - PED_R, v_hat=obs["v"], det=int(tracked),
                 y_hat=obs["y_hat"] if tracked else -1.0,
                 sigma_y=max(0.1, obs["sig_y"]), p_walk=sigmoid((vy - 0.4) / 0.12) if tracked else 0.0,
                 rho=rho)
