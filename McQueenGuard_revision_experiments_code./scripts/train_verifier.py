"""Train the learned verifier, calibrate the conformal bound and the familiarity gate.

Split by episode: 70 % train (10 % of it for early stopping), 15 % calibration, 15 % test.
Outputs models/verifier.{pt,npz}, models/verifier.onnx, results/verifier_stats.json
"""
import sys, json
import numpy as np
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from mqg.verifier import LearnedVerifier, conformal_eps, admissible_residual
from mqg.config import ALPHA_CONF, THETA

import glob
files = sys.argv[1:] or sorted(glob.glob("data/d0_*.npz"))
D = [np.load(f) for f in files]
X = np.concatenate([d["X"] for d in D]).astype(np.float64)
P = np.concatenate([d["P"] for d in D])
U = np.concatenate([d["U"] for d in D])
ep = np.concatenate([d["ep"] for d in D])
N_TARGET = 12000
rng = np.random.default_rng(0)
if len(X) > N_TARGET:
    idx = np.sort(rng.choice(len(X), N_TARGET, replace=False))
    X, P, U, ep = X[idx], P[idx], U[idx], ep[idx]
eps_ids = np.unique(ep)
rng.shuffle(eps_ids)
n = len(eps_ids)
tr_ids, cal_ids, te_ids = eps_ids[:int(0.70 * n)], eps_ids[int(0.70 * n):int(0.85 * n)], eps_ids[int(0.85 * n):]
tr, cal, te = np.isin(ep, tr_ids), np.isin(ep, cal_ids), np.isin(ep, te_ids)
Y = np.concatenate([P, U], 1)
itr = np.where(tr)[0]
rng.shuffle(itr)
nv = int(0.1 * len(itr))
va_i, tr_i = itr[:nv], itr[nv:]

v = LearnedVerifier()
v.fit(X[tr_i], Y[tr_i], X[va_i], Y[va_i])
v.Xtr_ref = X[tr]
eps = v.calibrate(X[cal], P[cal], alpha=ALPHA_CONF)
Ph, Uh = v.predict(X[te])
err_p = np.abs(Ph - P[te])
err_u = np.abs(Uh - U[te])
resid_te = admissible_residual(P[te], Ph)
cov = float(np.mean(resid_te <= eps))
fam_te = v.familiar(X[te])
# unsafe-admission rate on test: learned admitted (p_hat+eps<theta for chosen) while exact >= theta
from mqg.controllers import lexicographic_select
adm, viol = 0, 0
for i in range(te.sum()):
    a, ok = lexicographic_select(Ph[i], Uh[i], eps)
    if ok and fam_te[i]:
        adm += 1
        viol += int(P[te][i, a] >= THETA)
stats = dict(
    n_total=int(len(X)), n_train=int(tr.sum()), n_cal=int(cal.sum()), n_test=int(te.sum()),
    n_episodes=int(n), mae_p=float(err_p.mean()), rmse_p=float(np.sqrt((err_p ** 2).mean())),
    mae_u=float(err_u.mean()), rmse_u=float(np.sqrt((err_u ** 2).mean())),
    p95_abs_err_p=float(np.percentile(err_p, 95)), max_abs_err_p=float(err_p.max()),
    mean_abs_resid_cal=float(np.abs(v.cal_resid).mean()),
    eps=float(eps), alpha=ALPHA_CONF, coverage_test=cov,
    familiar_rate_test=float(fam_te.mean()), admitted_test=adm, unsafe_admission_test=viol,
    unsafe_admission_rate_test=float(viol / max(1, adm)), gate_thr=v.gate_thr,
    epochs=len(v.history))
print(json.dumps(stats, indent=1))
v.save("models/verifier")
v.export_onnx("models/verifier.onnx")
json.dump(stats, open("results/verifier_stats.json", "w"), indent=1)
np.savez("data/split_ids.npz", tr=tr_ids, cal=cal_ids, te=te_ids)
