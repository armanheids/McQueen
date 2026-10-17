"""Learned verifier Gamma' (18-64-64-32-10 MLP), split-conformal error bound and familiarity gate."""
import math
import json
import numpy as np
import torch
import torch.nn as nn
from sklearn.covariance import LedoitWolf

from .config import ALPHA_CONF, FEATURES

N_ACT = 5


class MLP(nn.Module):
    def __init__(self, d_in=18, d_out=2 * N_ACT):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(d_in, 64), nn.ReLU(), nn.Linear(64, 64), nn.ReLU(),
                                 nn.Linear(64, 32), nn.ReLU(), nn.Linear(32, d_out))

    def forward(self, x):
        return self.net(x)


def conformal_eps(resid, alpha):
    """Eq. (25): one-sided split-conformal margin from calibration residuals."""
    n = len(resid)
    k = math.ceil((n + 1) * (1 - alpha))
    if k > n:
        return math.inf
    return max(0.0, float(np.sort(resid)[k - 1]))


def admissible_residual(P, Ph, theta=None):
    """Simultaneous one-sided residual over the actions that could ever be admitted (p_hat < theta).
    Admission of an unsafe action requires P_a > p_hat_a + eps for some a with p_hat_a < theta, hence
    coverage of this residual at level 1-alpha bounds the unsafe-admission probability by alpha."""
    from .config import THETA
    th = THETA if theta is None else theta
    D = np.where(Ph < th, P - Ph, -np.inf)
    r = D.max(1)
    return np.where(np.isfinite(r), r, 0.0)


class LearnedVerifier:
    def __init__(self):
        self.model = None
        self.eps = None

    # ------------------------------------------------------------------ training
    def fit(self, Xtr, Ytr, Xva, Yva, epochs=400, lr=1e-3, batch=128, patience=25, seed=0, log=print):
        torch.manual_seed(seed)
        np.random.seed(seed)
        self.Xtr_ref = Xtr
        self.mu = Xtr.mean(0)
        self.sd = Xtr.std(0) + 1e-6
        self.const = Xtr.std(0) < 1e-6
        self.model = MLP(Xtr.shape[1], Ytr.shape[1])
        opt = torch.optim.Adam(self.model.parameters(), lr=lr)
        xt = torch.tensor((Xtr - self.mu) / self.sd, dtype=torch.float32)
        yt = torch.tensor(Ytr, dtype=torch.float32)
        xv = torch.tensor((Xva - self.mu) / self.sd, dtype=torch.float32)
        yv = torch.tensor(Yva, dtype=torch.float32)
        best, best_state, bad = math.inf, None, 0
        hist = []
        for ep in range(epochs):
            self.model.train()
            perm = torch.randperm(len(xt))
            for i in range(0, len(xt), batch):
                idx = perm[i:i + batch]
                loss = ((self.model(xt[idx]) - yt[idx]) ** 2).mean()
                opt.zero_grad()
                loss.backward()
                opt.step()
            self.model.eval()
            with torch.no_grad():
                vl = float(((self.model(xv) - yv) ** 2).mean())
                tl = float(((self.model(xt) - yt) ** 2).mean())
            hist.append((ep, tl, vl))
            if vl < best - 1e-7:
                best, bad = vl, 0
                best_state = {k: v.clone() for k, v in self.model.state_dict().items()}
            else:
                bad += 1
                if bad >= patience:
                    break
        self.model.load_state_dict(best_state)
        self.model.eval()
        self.history = hist
        log(f"trained {len(hist)} epochs, best val MSE {best:.5f}")
        self._export_numpy()
        return hist

    def _export_numpy(self):
        self.W = [l.weight.detach().numpy().astype(np.float64) for l in self.model.net if isinstance(l, nn.Linear)]
        self.b = [l.bias.detach().numpy().astype(np.float64) for l in self.model.net if isinstance(l, nn.Linear)]

    # ------------------------------------------------------------------ inference
    def raw(self, X):
        h = (np.atleast_2d(X) - self.mu) / self.sd
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            h = h @ W.T + b
            if i < len(self.W) - 1:
                h = np.maximum(h, 0.0)
        return h

    def predict(self, X):
        out = self.raw(X)
        P = np.clip(out[:, :N_ACT], 0.0, 1.0)
        U = out[:, N_ACT:]
        return P, U

    def familiar(self, X):
        X = np.atleast_2d(X)
        Z = ((X - self.mu) / self.sd)[:, ~self.const]
        dz = Z - self.gate_mean
        m2 = np.einsum("ij,jk,ik->i", dz, self.gate_prec, dz)
        inside = np.all((X >= self.env_lo) & (X <= self.env_hi), axis=1)
        return (m2 <= self.gate_thr) & inside

    def predict_one(self, x):
        h = (x - self.mu) / self.sd
        for i in range(len(self.W)):
            h = self.W[i] @ h + self.b[i]
            if i < len(self.W) - 1:
                np.maximum(h, 0.0, out=h)
        z = ((x - self.mu) / self.sd)[~self.const] - self.gate_mean
        fam = float(z @ self.gate_prec @ z) <= self.gate_thr and bool(np.all((x >= self.env_lo) & (x <= self.env_hi)))
        return np.clip(h[:N_ACT], 0.0, 1.0), h[N_ACT:], fam

    # ------------------------------------------------------------------ calibration
    def calibrate(self, Xc, Pc, alpha=ALPHA_CONF, gate_q=0.99):
        Ph, _ = self.predict(Xc)
        self.cal_resid = admissible_residual(Pc, Ph)
        self.eps = conformal_eps(self.cal_resid, alpha)
        self.alpha = alpha
        # familiarity gate: Mahalanobis distance in standardised feature space (Ledoit-Wolf)
        Ztr = ((self.Xtr_ref - self.mu) / self.sd)[:, ~self.const]
        lw = LedoitWolf().fit(Ztr)
        self.gate_mean = lw.location_
        self.gate_prec = lw.precision_
        Zc = ((Xc - self.mu) / self.sd)[:, ~self.const] - self.gate_mean
        m2 = np.einsum("ij,jk,ik->i", Zc, self.gate_prec, Zc)
        self.gate_thr = float(np.quantile(m2, gate_q))
        # training envelope (operational-design-domain check), widened by 5 % of each feature range
        lo, hi = self.Xtr_ref.min(0), self.Xtr_ref.max(0)
        pad = 0.05 * (hi - lo)
        self.env_lo, self.env_hi = lo - pad, hi + pad
        return self.eps

    # ------------------------------------------------------------------ persistence
    def save(self, path):
        torch.save(self.model.state_dict(), path + ".pt")
        np.savez(path + ".npz", mu=self.mu, sd=self.sd, const=self.const, eps=self.eps, alpha=self.alpha,
                 gate_mean=self.gate_mean, gate_prec=self.gate_prec, gate_thr=self.gate_thr,
                 cal_resid=self.cal_resid, env_lo=self.env_lo, env_hi=self.env_hi)

    @classmethod
    def load(cls, path):
        v = cls()
        z = np.load(path + ".npz")
        v.mu, v.sd, v.const = z["mu"], z["sd"], z["const"]
        v.eps, v.alpha = float(z["eps"]), float(z["alpha"])
        v.gate_mean, v.gate_prec, v.gate_thr = z["gate_mean"], z["gate_prec"], float(z["gate_thr"])
        v.cal_resid = z["cal_resid"]
        v.env_lo, v.env_hi = z["env_lo"], z["env_hi"]
        v.model = MLP(len(v.mu), 2 * N_ACT)
        v.model.load_state_dict(torch.load(path + ".pt"))
        v.model.eval()
        v._export_numpy()
        return v

    def export_onnx(self, path):
        class Wrapped(nn.Module):
            def __init__(s, m, mu, sd):
                super().__init__()
                s.m = m
                s.register_buffer("mu", torch.tensor(mu, dtype=torch.float32))
                s.register_buffer("sd", torch.tensor(sd, dtype=torch.float32))

            def forward(s, x):
                return s.m((x - s.mu) / s.sd)
        w = Wrapped(self.model, self.mu, self.sd).eval()
        dummy = torch.zeros(1, len(self.mu))
        torch.onnx.export(w, dummy, path, input_names=["x"], output_names=["y"], opset_version=17,
                          dynamic_axes=None, dynamo=False)
