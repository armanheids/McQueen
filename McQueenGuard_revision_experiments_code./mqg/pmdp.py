"""Parametric MDP abstraction of the pedestrian-crossing decision problem.

One query = one runtime decision point x with parameter valuation lambda.  For every candidate
action a in ACTIONS, exact quantitative verification returns
    p_unsafe(x,a) = Pmax=? [ F<=17 "unsafe" ] (worst case over pedestrian hesitation); "unsafe" =
                    pedestrian in the ego lane while the ego front is within D_SM = 2 m of, or
                    inside, the crossing (collision or safe-margin violation)
    U(x,a)        = Rmax=? [ C<=17 ] / 16     (normalised performance reward)
The ego commits to `a` for one model step (0.25 s); afterwards the Simplex continuation policy runs
for the remaining 3.75 s: emergency braking when a detected pedestrian threatens the ego path,
otherwise nominal speed tracking towards the desired speed.

The same model is (i) emitted as a PRISM file (the authoritative engine, used for latency and
validation) and (ii) solved by an equivalent backward induction compiled with numba (used to label
large datasets quickly).  `validate_against_prism.py` checks that both agree.
"""
from dataclasses import dataclass
import math
import numpy as np
from numba import njit

from .config import G, DT_MDP, H_MDP, ACTION_ACC

D_MIN, D_MAX = -6, 60          # d = -6 is the absorbing "passed" state
ND = D_MAX - D_MIN + 1
VRES = 0.5                     # speed grid resolution [m/s]
NV = 35                        # v = 0 .. 17 m/s
V_TOP = (NV - 1) * VRES
NY = 13                        # lateral bins of 0.5 m, y = -2.0 + 0.5*iy ; iy = 12 -> cleared
NS = NY * 2                    # pedestrian state (iy, m)
IY_LO, IY_HI = 4, 11           # in-lane bins
D_UNSAFE_HI = 2                # unsafe if ego front within D_SM (2 m) of the crossing line
CW = 0.3                       # comfort weight of the performance reward
AMAX_R = 9.0
A_NOM = 1.5                    # nominal speed-tracking acceleration of the continuation policy
E_CONST = 2.718281828459045
FRAMES_PER_STEP = 5


@dataclass
class Query:
    # parameter valuation lambda
    mu: float            # estimated road friction
    pe: float            # perception error probability
    r_vis: float         # estimated visibility / detection range [m]
    v_ped: float         # pedestrian walking speed estimate [m/s]
    p_go: float          # per-step probability that a waiting pedestrian starts crossing
    v_ref: float         # desired speed [m/s]
    # runtime state estimate
    d_hat: float         # distance ego front -> pedestrian crossing line [m]
    v_hat: float         # ego speed [m/s]
    det: int             # pedestrian detected?
    y_hat: float = -1.0  # pedestrian lateral position (lane frame) [m]
    sigma_y: float = 0.3
    p_walk: float = 0.0
    rho: float = 0.3     # presence probability of an undetected pedestrian at the crosswalk

    @property
    def p_slip(self):
        return float(min(0.5, max(0.02, 0.6 * (0.9 - self.mu) + 0.02)))

    @property
    def a_brk(self):
        return -0.95 * self.mu * G

    def action_acc(self):
        out = []
        for a in ACTION_ACC:
            out.append(self.a_brk if a is None else (max(a, self.a_brk) if a < 0 else a))
        return out

    @property
    def r_walk(self):
        return max(0.05, min(2.0, self.v_ped * DT_MDP / 0.5))


# ----------------------------------------------------------------------------- prior
def _round_split(x, lo, hi):
    x = min(max(x, lo), hi)
    f = math.floor(x)
    w = x - f
    if w > 0 and f + 1 <= hi:
        return [(int(f), 1.0 - w), (int(f + 1), w)]
    return [(int(f), 1.0)]


def _norm_cdf(z):
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


def prior_lists(q: Query, tol=1e-4):
    """Independent prior marginals over ego (d, iv) and pedestrian (iy, m)."""
    if q.d_hat < -5.0:
        dl = [(D_MIN, 1.0)]
    else:
        dl = [((D_MIN if dd < -5 else dd), p) for dd, p in _round_split(q.d_hat, -5, D_MAX)]
    vl = _round_split(q.v_hat / VRES, 0, NV - 1)
    ego = {}
    for dd, pd_ in dl:
        for vv, pv in vl:
            ego[(dd, vv)] = ego.get((dd, vv), 0.0) + pd_ * pv
    ped = {}
    if q.det:
        sy = max(0.05, q.sigma_y)
        pw = min(1.0, max(0.0, q.p_walk))
        for iy in range(NY):
            yc = -2.0 + 0.5 * iy
            plo = 0.0 if iy == 0 else _norm_cdf((yc - 0.25 - q.y_hat) / sy)
            phi = 1.0 if iy == NY - 1 else _norm_cdf((yc + 0.25 - q.y_hat) / sy)
            p = max(0.0, phi - plo)
            if p <= 0:
                continue
            if iy == NY - 1:
                ped[(iy, 0)] = ped.get((iy, 0), 0.0) + p
            else:
                for m, pm in ((0, 1.0 - pw), (1, pw)):
                    if p * pm > 0:
                        ped[(iy, m)] = ped.get((iy, m), 0.0) + p * pm
    else:
        rho = min(1.0, max(0.0, q.rho))
        ped[(2, 0)] = rho / 2
        ped[(3, 0)] = rho / 2
        ped[(NY - 1, 0)] = 1.0 - rho

    def clean(dct):
        items = [(k, v) for k, v in dct.items() if v >= tol]
        s = sum(v for _, v in items)
        return [(k, v / s) for k, v in items]

    return clean(ego), clean(ped)


# ----------------------------------------------------------------------------- numba solver
@njit(cache=True)
def _ego_out(d, v, al, pslip, nv, odi, ovi, op):
    """8 probabilistic outcomes of one ego step; fills odi (d index), ovi (v index), op."""
    ps = pslip if al < 0 else 0.0
    k = 0
    for sl in range(2):
        a = al if sl == 0 else al * 0.6
        pw = (1.0 - ps) if sl == 0 else ps
        vn = min(V_TOP, max(0.0, v + a * DT_MDP))
        if v + a * DT_MDP >= 0:
            dist = v * DT_MDP + 0.5 * a * DT_MDP * DT_MDP
        else:
            dist = v * v / (2.0 * (-a))
        dn = d - dist
        fl = math.floor(dn)
        w = dn - fl
        df = -6 if fl < -5 else min(60, fl)
        dc = -6 if fl + 1 < -5 else min(60, fl + 1)
        vq = vn / VRES
        vf = math.floor(vq)
        wv = vq - vf
        vc = min(NV - 1, vf + 1)
        for j in range(2):
            dsel = df if j == 0 else dc
            pdw = (1.0 - w) if j == 0 else w
            for i in range(2):
                vsel = vf if i == 0 else vc
                pvw = (1.0 - wv) if i == 0 else wv
                odi[k] = min(int(dsel) - D_MIN, ND - 1)
                ovi[k] = min(int(vsel), nv - 1)
                op[k] = pw * pdw * pvw
                k += 1


@njit(cache=True)
def _reward(v, al, vref):
    prog = 1.0 - abs(v - vref) / vref
    if prog < 0.0:
        prog = 0.0
    return prog + CW * (1.0 - (al / AMAX_R) ** 2)


@njit(cache=True)
def _expect(VP, VR, A0P, A0R, odi, ovi, op, s, dc, pd, ch):
    ep = 0.0
    er = 0.0
    for k in range(8):
        pk = op[k]
        if pk == 0.0:
            continue
        a_ = odi[k]
        b_ = ovi[k]
        if ch == 0:
            if dc == 0:
                ep += pk * (pd * A0P[a_, b_, s, 1] + (1 - pd) * A0P[a_, b_, s, 0])
                er += pk * (pd * A0R[a_, b_, s, 1] + (1 - pd) * A0R[a_, b_, s, 0])
            else:
                ep += pk * A0P[a_, b_, s, 1]
                er += pk * A0R[a_, b_, s, 1]
        else:
            if dc == 0:
                ep += pk * (pd * VP[a_, b_, s, 1] + (1 - pd) * VP[a_, b_, s, 0])
                er += pk * (pd * VR[a_, b_, s, 1] + (1 - pd) * VR[a_, b_, s, 0])
            else:
                ep += pk * VP[a_, b_, s, 1]
                er += pk * VR[a_, b_, s, 1]
    return ep, er


@njit(cache=True)
def _ped_expect(VP, VR, A0P, A0R, pidx, pprob, nd, nv):
    for di in range(nd):
        for iv in range(nv):
            for s in range(NS):
                i0 = pidx[s, 0]
                i1 = pidx[s, 1]
                p0 = pprob[s, 0]
                p1 = pprob[s, 1]
                for j in range(2):
                    A0P[di, iv, s, j] = p0 * VP[di, iv, i0, j] + p1 * VP[di, iv, i1, j]
                    A0R[di, iv, s, j] = p0 * VR[di, iv, i0, j] + p1 * VR[di, iv, i1, j]


@njit(cache=True)
def _solve_nb(nd, nv, vref, abrk, pslip, pdet, pidx, pprob, hes, accs, prior_e, prior_ep, prior_s,
              prior_sp, det0, hsteps):
    VP = np.zeros((nd, nv, NS, 2))
    VR = np.zeros((nd, nv, NS, 2))
    col = np.zeros((nd, NS), dtype=np.bool_)
    for di in range(nd):
        d = D_MIN + di
        if d >= -5 and d <= D_UNSAFE_HI:
            for s in range(NS):
                iy = s // 2
                if iy >= IY_LO and iy <= IY_HI:
                    col[di, s] = True
                    VP[di, :, s, :] = 1.0
    odi_n = np.zeros(8, np.int64); ovi_n = np.zeros(8, np.int64); op_n = np.zeros(8)
    odi_b = np.zeros(8, np.int64); ovi_b = np.zeros(8, np.int64); op_b = np.zeros(8)
    A0P = np.zeros((nd, nv, NS, 2)); A0R = np.zeros((nd, nv, NS, 2))
    for step in range(hsteps):
        _ped_expect(VP, VR, A0P, A0R, pidx, pprob, nd, nv)
        nVP = np.zeros((nd, nv, NS, 2))
        nVR = np.zeros((nd, nv, NS, 2))
        for di in range(nd):
            d = D_MIN + di
            for iv in range(nv):
                v = iv * VRES
                if di == 0:              # passed: absorbing, hold speed
                    r = _reward(v, 0.0, vref)
                    for s in range(NS):
                        for c in range(2):
                            nVR[di, iv, s, c] = r + VR[0, iv, 0, 0]
                    continue
                a_nom = A_NOM if v < vref else 0.0
                _ego_out(d, v, a_nom, pslip, nv, odi_n, ovi_n, op_n)
                _ego_out(d, v, abrk, pslip, nv, odi_b, ovi_b, op_b)
                r_nom = _reward(v, a_nom, vref)
                r_brk = _reward(v, abrk, vref)
                pd = pdet[di]
                for s in range(NS):
                    if col[di, s]:
                        nVP[di, iv, s, 0] = 1.0
                        nVP[di, iv, s, 1] = 1.0
                        continue
                    iy = s // 2
                    m = s % 2
                    for dc in range(2):
                        threat = dc == 1 and (m == 1 or iy >= 3) and iy <= IY_HI and d >= -5
                        bp = -1.0
                        br = -1.0
                        nch = 2 if hes[s] else 1
                        for ch in range(nch):
                            if threat:
                                ep, er = _expect(VP, VR, A0P, A0R, odi_b, ovi_b, op_b, s, dc, pd, ch)
                            else:
                                ep, er = _expect(VP, VR, A0P, A0R, odi_n, ovi_n, op_n, s, dc, pd, ch)
                            if ep > bp:
                                bp = ep
                            if er > br:
                                br = er
                        nVP[di, iv, s, dc] = bp
                        nVR[di, iv, s, dc] = (r_brk if threat else r_nom) + br
        VP = nVP
        VR = nVR
    # commit step, evaluated at the prior support only
    _ped_expect(VP, VR, A0P, A0R, pidx, pprob, nd, nv)
    na = accs.shape[0]
    pu = np.zeros(na)
    uu = np.zeros(na)
    for ai in range(na):
        al = accs[ai]
        for ie in range(prior_e.shape[0]):
            di = prior_e[ie, 0]
            iv = prior_e[ie, 1]
            d = D_MIN + di
            v = iv * VRES
            if di > 0:
                _ego_out(d, v, al, pslip, nv, odi_n, ovi_n, op_n)
            for isx in range(prior_s.shape[0]):
                s = prior_s[isx]
                w = prior_ep[ie] * prior_sp[isx]
                if di == 0:
                    uu[ai] += w * (_reward(v, 0.0, vref) + VR[0, iv, 0, 0])
                    continue
                if col[di, s]:
                    pu[ai] += w
                    continue
                pd = pdet[di]
                bp = -1.0
                br = -1.0
                nch = 2 if hes[s] else 1
                for ch in range(nch):
                    ep, er = _expect(VP, VR, A0P, A0R, odi_n, ovi_n, op_n, s, det0, pd, ch)
                    if ep > bp:
                        bp = ep
                    if er > br:
                        br = er
                pu[ai] += w * bp
                uu[ai] += w * (_reward(v, al, vref) + br)
    return pu, uu / (hsteps + 1)


def ped_kernel(q: Query):
    pidx = np.zeros((NS, 2), np.int64)
    pprob = np.zeros((NS, 2))
    hes = np.zeros(NS, np.bool_)
    r = q.r_walk
    for iy in range(NY):
        for m in (0, 1):
            s = iy * 2 + m
            if iy == NY - 1:
                pidx[s] = (s, s); pprob[s] = (1.0, 0.0)
                continue
            if m == 0:
                pidx[s] = (min(NY - 1, iy + 1) * 2 + 1, s); pprob[s] = (q.p_go, 1.0 - q.p_go)
            else:
                if r <= 1:
                    pidx[s] = (min(NY - 1, iy + 1) * 2 + 1, s); pprob[s] = (r, 1.0 - r)
                else:
                    pidx[s] = (min(NY - 1, iy + 1) * 2 + 1, min(NY - 1, iy + 2) * 2 + 1); pprob[s] = (2.0 - r, r - 1.0)
                hes[s] = IY_LO <= iy <= IY_HI
    return pidx, pprob, hes


def pdet_vec(q: Query, nd):
    dped = np.maximum(0.5, (D_MIN + np.arange(nd)).astype(float))
    pf = (1 - q.pe) / (1 + np.power(E_CONST, -(q.r_vis - dped) / 4))
    return 1 - np.power(1 - pf, FRAMES_PER_STEP)


class ExactSolver:
    """Backward induction equivalent to the generated PRISM model (bounded Pmax / Rmax)."""

    def solve(self, q: Query, accs=None):
        accs = np.array(q.action_acc() if accs is None else accs, dtype=float)
        ego_p, ped_p = prior_lists(q)
        nd = max(7, max(dd for (dd, _), _ in ego_p) - D_MIN + 1)
        vmax = max(max(vv for (_, vv), _ in ego_p) * VRES, q.v_ref) + 1.0
        nv = int(min(NV, math.ceil(vmax / VRES) + 2))
        pidx, pprob, hes = ped_kernel(q)
        pe_arr = np.array([[dd - D_MIN, vv] for (dd, vv), _ in ego_p], np.int64)
        pe_p = np.array([p for _, p in ego_p])
        ps_arr = np.array([iy * 2 + m for (iy, m), _ in ped_p], np.int64)
        ps_p = np.array([p for _, p in ped_p])
        pu, uu = _solve_nb(nd, nv, float(q.v_ref), float(q.a_brk), float(q.p_slip), pdet_vec(q, nd),
                           pidx, pprob, hes, accs, pe_arr, pe_p, ps_arr, ps_p, int(q.det), H_MDP - 1)
        return pu, uu


# ----------------------------------------------------------------------------- PRISM generator
def prism_model(q: Query) -> str:
    ego_p, ped_p = prior_lists(q)
    accs = q.action_acc()
    r = q.r_walk
    L = []
    w = L.append
    w("// McQueenGuard pedestrian-crossing pMDP instantiated at a runtime decision point")
    w("mdp")
    w(f"const double DT = {DT_MDP};")
    w(f"const double VRES = {VRES};")
    w(f"const double MU = {q.mu!r};")
    w(f"const double ABRK = {q.a_brk!r};")
    w(f"const double PSLIP = {q.p_slip!r};")
    w(f"const double VREF = {q.v_ref!r};")
    w(f"const double PE = {q.pe!r};")
    w(f"const double RVIS = {q.r_vis!r};")
    w(f"const double PGO = {q.p_go!r};")
    w(f"const double CW = {CW};")
    w(f"const double AMAX = {AMAX_R};")
    w(f"const double ANOM = {A_NOM};")
    w(f"const int DET0 = {int(q.det)};")
    w(f"formula col = phase>0 & d>=-5 & d<={D_UNSAFE_HI} & iy>=4 & iy<=11;")
    w("formula threat = det=1 & (m=1 | iy>=3) & iy<=11 & d>=-5;")
    w("formula spd = v*VRES;")
    w(f"formula a0val = a0=0 ? {accs[0]!r} : (a0=1 ? {accs[1]!r} : (a0=2 ? {accs[2]!r} : (a0=3 ? {accs[3]!r} : {accs[4]!r})));")
    w("formula alpha = d=-6 ? 0.0 : (phase=1 ? a0val : (threat ? ABRK : (spd < VREF ? ANOM : 0.0)));")
    w("formula ps = alpha<0 ? PSLIP : 0.0;")
    for k, f in ((0, "alpha"), (1, "alpha*0.6")):
        w(f"formula al{k} = {f};")
        w(f"formula vn{k} = min({V_TOP}, max(0.0, spd + al{k}*DT));")
        w(f"formula dist{k} = (spd + al{k}*DT >= 0) ? (spd*DT + 0.5*al{k}*DT*DT) : (spd*spd/(2*(-al{k})));")
        w(f"formula dn{k} = d - dist{k};")
        w(f"formula fl{k} = floor(dn{k});")
        w(f"formula w{k} = dn{k} - fl{k};")
        w(f"formula df{k} = fl{k} < -5 ? -6 : min(60, fl{k});")
        w(f"formula dc{k} = fl{k}+1 < -5 ? -6 : min(60, fl{k}+1);")
        w(f"formula vf{k} = floor(vn{k}/VRES);")
        w(f"formula wv{k} = vn{k}/VRES - vf{k};")
        w(f"formula vc{k} = min({NV - 1}, vf{k}+1);")
    w("formula dped = max(0.5, d);")
    w(f"formula pf = (1-PE)/(1+pow({E_CONST!r}, -(RVIS - dped)/4));")
    w(f"formula pdet = 1 - pow(1-pf, {FRAMES_PER_STEP});")
    w("formula prog = max(0.0, 1 - (spd > VREF ? spd - VREF : VREF - spd)/VREF);")
    w("")
    w("module ego")
    w("  phase : [0..2];")
    w("  a0 : [0..4];")
    w("  d : [-6..60];")
    w(f"  v : [0..{NV - 1}];")
    br = " + ".join(f"{p!r}:(phase'=1)&(d'={dd})&(v'={vv})" for (dd, vv), p in ego_p)
    w(f"  [step] phase=0 -> {br};")
    w("  [step] phase>0 & (col | d=-6) -> (phase'=2)&(a0'=0);")
    br = []
    for k, ps in ((0, "(1-ps)"), (1, "ps")):
        for dsel, pdw in ((f"df{k}", f"(1-w{k})"), (f"dc{k}", f"w{k}")):
            for vsel, pvw in ((f"vf{k}", f"(1-wv{k})"), (f"vc{k}", f"wv{k}")):
                br.append(f"{ps}*{pdw}*{pvw}:(phase'=2)&(a0'=0)&(d'={dsel})&(v'={vsel})")
    w("  [step] phase>0 & !col & d>-6 -> " + " + ".join(br) + ";")
    w("endmodule")
    w("")
    w("module ped")
    w("  iy : [0..12];")
    w("  m : [0..1];")
    br = " + ".join(f"{p!r}:(iy'={iy})&(m'={mm})" for (iy, mm), p in ped_p)
    w(f"  [step] phase=0 -> {br};")
    w("  [step] phase>0 & (col | iy=12) -> true;")
    w("  [step] phase>0 & !col & iy<12 & m=0 -> PGO:(m'=1)&(iy'=min(12,iy+1)) + (1-PGO):true;")
    if r <= 1:
        w(f"  [step] phase>0 & !col & iy<12 & m=1 -> {r!r}:(iy'=min(12,iy+1)) + {1 - r!r}:true;")
    else:
        w(f"  [step] phase>0 & !col & iy<12 & m=1 -> {2 - r!r}:(iy'=min(12,iy+1)) + {r - 1!r}:(iy'=min(12,iy+2));")
    w("  [step] phase>0 & !col & iy>=4 & iy<=11 & m=1 -> true; // hesitation in the lane (adversarial)")
    w("endmodule")
    w("")
    w("module sensor")
    w("  det : [0..1];")
    w("  [step] phase=0 -> (det'=DET0);")
    w("  [step] phase>0 & det=0 & !col -> pdet:(det'=1) + (1-pdet):true;")
    w("  [step] phase>0 & (det=1 | col) -> true;")
    w("endmodule")
    w("")
    w("init phase=0 & d=0 & v=0 & iy=0 & m=0 & det=0 endinit")
    w("")
    w('rewards "perf"')
    w("  [step] phase>0 & !col : prog + CW*(1 - pow(alpha/AMAX, 2));")
    w("endrewards")
    w('label "col" = col;')
    return "\n".join(L) + "\n"


PRISM_PROPS = ('filter(print, Pmax=? [ F<=17 "col" ], "init")\n'
               'filter(print, Rmax=? [ C<=17 ], "init")\n')
