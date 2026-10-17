"""Single loader for the final paired episode set used by every table and figure.

Sources (all on the same scenario instances, seed 10000*(ci+1)+e, e = 0..59 per category):
  abl_w*.pkl       full McQueenGuard (+ labelled decisions) and five ablation variants, e = 0..59
  compare_w*.pkl   exact-only, learned-only, MPC, rule-based, e = 0..29
  compareB_w0.pkl  VerifAI RA, PPO, e = 0..29
  ext_w*.pkl       exact-only, learned-only, MPC, rule-based, VerifAI RA, PPO, e = 30..59
  ideal_w*.pkl     McQueenGuard with a zero-latency exact fallback (idealised reference), e = 0..59
"""
import glob, pickle, math
import numpy as np
import pandas as pd
from .config import THETA, CATEGORIES
from .world import D0

ID_CATS = [c.name for c in CATEGORIES if c.in_distribution]
OOD_CATS = [c.name for c in CATEGORIES if not c.in_distribution]


def _load(pattern):
    out, rows = [], []
    for f in sorted(glob.glob(pattern)):
        s = pickle.load(open(f, "rb"))
        if isinstance(s, dict):
            out += s["res"]
            rows += s.get("rows", [])
        else:
            out += s
    return out, rows


def load_episodes(n_max=60, extra_tags=()):
    res = []
    a, rows = _load("results/abl_w*.pkl")
    res += a
    for pat in ("results/compare_w*.pkl", "results/compareB_w0.pkl", "results/ext_w*.pkl", "results/ideal_w*.pkl") + tuple(extra_tags):
        res += _load(pat)[0]
    recs, lat, aud = [], {}, {}
    seen = set()
    for r in res:
        e = r.seed % 10_000
        if e >= n_max:
            continue
        key = (r.method, r.category, r.seed)
        if key in seen:
            continue
        seen.add(key)
        free = (D0 + 0.3) / max(r.v0, 0.5)
        n_dec = max(1, r.n_dec)
        recs.append(dict(method=r.method, category=r.category, seed=r.seed, e=e, ood=r.category in OOD_CATS,
                         collided=r.collided, mv=r.margin_violation, timeout=r.timeout, min_gap=r.min_gap,
                         t_clear=r.t_clear, delay_ratio=(r.t_clear / free) if r.t_clear == r.t_clear else math.nan,
                         mean_speed=r.mean_speed * 3.6, max_decel=r.max_decel, rms_jerk=r.rms_jerk,
                         n_dec=r.n_dec, n_learned=r.n_learned, n_exact=r.n_exact, n_fb=r.n_fallback_ctrl,
                         n_cons=getattr(r, "n_conservative", 0), miss=r.miss_rate, v0=r.v0 * 3.6,
                         lat_mean=r.lat_mean, n_aud=len(r.audits),
                         n_aud_viol=int(np.sum(np.array(r.audits) >= THETA)) if r.audits else 0,
                         overrides=r.n_override_ticks))
        lat.setdefault(r.method, []).extend(r.lats)
        aud.setdefault(r.method, []).extend(r.audits)
    df = pd.DataFrame(recs)
    return df, lat, aud, rows


def wilson(k, n, z=1.96):
    if n == 0:
        return (math.nan, math.nan)
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, c - h), min(1.0, c + h))


def mcnemar(df, a, b, col="mv"):
    from scipy.stats import binomtest
    A = df[df.method == a].set_index(["category", "seed"])[col]
    B = df[df.method == b].set_index(["category", "seed"])[col]
    j = A.index.intersection(B.index)
    A, B = A.loc[j].astype(bool), B.loc[j].astype(bool)
    n10 = int((A & ~B).sum())
    n01 = int((~A & B).sum())
    p = binomtest(n10, n10 + n01, 0.5).pvalue if n10 + n01 > 0 else 1.0
    return n10, n01, float(p)


def summarise(df, lat, methods, cats=None):
    out = []
    for m in methods:
        d = df[df.method == m]
        if cats is not None:
            d = d[d.category.isin(cats)]
        if len(d) == 0:
            continue
        n = len(d)
        L = np.array(lat.get(m, [0.0])) * 1000 if cats is None else None
        na = d.n_aud.sum()
        nd_all = (d.n_learned + d.n_exact + d.n_fb + d.n_cons).sum()
        row = dict(method=m, episodes=n,
                   collision=100 * d.collided.mean(), collision_ci=[100 * x for x in wilson(int(d.collided.sum()), n)],
                   margin_violation=100 * d.mv.mean(), mv_ci=[100 * x for x in wilson(int(d.mv.sum()), n)],
                   n_col=int(d.collided.sum()), n_mv=int(d.mv.sum()),
                   risk_violation=100 * d.n_aud_viol.sum() / max(1, na), audited=int(na),
                   deadlock=100 * d.timeout.mean(), n_dead=int(d.timeout.sum()), mean_speed=d.mean_speed.mean(),
                   delay_ratio=float(d.delay_ratio.median()),
                   learned_share=100 * d.n_learned.sum() / max(1, nd_all),
                   exact_share=100 * (d.n_exact + d.n_fb).sum() / max(1, nd_all),
                   cons_share=100 * d.n_cons.sum() / max(1, nd_all),
                   miss=100 * (d.miss * d.n_dec).sum() / max(1, d.n_dec.sum()),
                   lat_mean_ep=float((d.lat_mean * d.n_dec).sum() / max(1, d.n_dec.sum())))
        if L is not None and len(L):
            row.update(lat_mean=float(L.mean()), lat_p50=float(np.median(L)), lat_p95=float(np.percentile(L, 95)),
                       lat_p99=float(np.percentile(L, 99)), lat_max=float(L.max()))
        out.append(row)
    return out
