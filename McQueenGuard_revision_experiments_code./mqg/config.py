"""Shared configuration for the McQueenGuard comparative testbed (JSA revision, R1-C6/C7).

Weather presets follow the McQueenGuard artifact ("Information of weather (Avg).txt");
snow is the unseen-weather condition. Scenario categories mirror Table 3 of the manuscript.
"""
from dataclasses import dataclass, field
import numpy as np

G = 9.81
DT_SIM = 0.05          # physics / safety-monitor tick [s]
DT_DEC = 0.10          # planner decision period for every method [s]
DT_MDP = 0.25          # pMDP time step [s]
H_MDP = 16             # pMDP look-ahead steps after the prior step (4 s)
CAR_LEN = 4.7          # ego length [m] (Tesla Model 3, as in Zhang et al. 2022)
LANE_W = 3.5           # ego lane width [m]
PED_R = 0.3            # pedestrian radius [m]
D_SM = 2.0             # required safe-margin distance [m]
THETA = 0.05           # admissible collision probability (bounded PCTL threshold)
ALPHA_CONF = 0.05      # conformal miscoverage level
EPS_MAX = 0.06         # maximum admissible approximation error
A_MAX_ACC = 2.5        # max commanded acceleration [m/s^2]
V_MAX_MDP = 17         # pMDP speed grid upper bound [m/s]

# candidate actions of the decision layer (Table 2 vocabulary + accelerate)
ACTIONS = ["accelerate", "maintain", "reduce", "brake", "emergency"]
ACTION_ACC = [1.5, 0.0, -2.0, -4.0, None]      # None -> friction-limited emergency braking


@dataclass
class Weather:
    name: str
    rain: float
    fog: float
    snow: float
    friction: float
    cam_noise: float
    lidar_acc: float
    latency_ms: float
    vis_range: float      # effective detection range [m]
    code: float           # weather-classifier output (clear 0, rain 0.5, fog/low-vis 1)


WEATHERS = {
    "clear": Weather("clear", 0.0, 0.0, 0.0, 0.90, 0.10, 0.94, 90.0, 80.0, 0.0),
    "rain":  Weather("rain", 0.6, 0.1, 0.0, 0.55, 0.18, 0.88, 110.0, 55.0, 0.5),
    "fog":   Weather("fog", 0.1, 0.7, 0.0, 0.60, 0.22, 0.85, 130.0, 38.0, 1.0),
    # unseen at training time: low friction, low visibility; classifier maps it to the nearest class
    "snow":  Weather("snow", 0.0, 0.3, 0.8, 0.30, 0.28, 0.80, 150.0, 32.0, 1.0),
}


@dataclass
class Category:
    name: str
    weathers: tuple
    v_range: tuple            # km/h
    ped_mode: str = "nominal" # nominal | novel
    noise_scale: float = 1.0
    extra_dropout: float = 0.0
    in_distribution: bool = True


CATEGORIES = [
    Category("Clear (10-30)", ("clear",), (10, 30)),
    Category("Clear (35-50)", ("clear",), (35, 50)),
    Category("Fog (10-30)", ("fog",), (10, 30)),
    Category("Fog (35-50)", ("fog",), (35, 50)),
    Category("Rain (10-30)", ("rain",), (10, 30)),
    Category("Rain (35-50)", ("rain",), (35, 50)),
    Category("High speed (50-60)", ("clear", "fog", "rain"), (50, 60), in_distribution=False),
    Category("Novel trajectory", ("clear", "fog", "rain"), (10, 50), ped_mode="novel", in_distribution=False),
    Category("High noise", ("clear", "fog", "rain"), (10, 50), noise_scale=3.0, extra_dropout=0.15, in_distribution=False),
    Category("Unseen weather", ("snow",), (10, 50), in_distribution=False),
]
NOMINAL = [c for c in CATEGORIES if c.in_distribution]
CAT_BY_NAME = {c.name: c for c in CATEGORIES}

FEATURES = [
    "ego_speed", "ego_acc", "exist_belief", "kerb_wait", "obstacle_dist", "ped_dist", "rel_ped_vel",
    "rel_ref_speed", "ttc", "det_conf", "visibility", "weather_code", "friction", "sensor_noise",
    "brake_dist", "stop_margin", "traffic_density", "plan_horizon",
]
