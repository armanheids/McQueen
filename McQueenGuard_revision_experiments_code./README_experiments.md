# McQueenGuard revision experiments (R1-C6, R1-C7, R2-C6, R2-C7)

Closed-loop testbed, baselines and analysis scripts behind the response document
`McQueenGuard_R1C6-7_R2C6-7_response.pdf`.

## Requirements
Python 3.10+, numpy, scipy, pandas, scikit-learn, numba, torch (CPU), onnx, onnxruntime,
stable-baselines3, gymnasium, cvxpy (OSQP), verifai, psutil, matplotlib; Java 17+ and PRISM 4.8.1
(set the PRISM path in `prism_driver/run_server.sh`; compile `MqgPrismServer.java` against `prism.jar`).

## Layout
- `mqg/` testbed: world/perception simulator (`world.py`), episode runner with latency injection (`sim.py`),
  parametric MDP and PRISM model generator + equivalent compiled solver (`pmdp.py`), learned verifier with split-conformal
  margin and familiarity check (`verifier.py`), McQueenGuard and ablation switches (`controllers.py`),
  baselines MPC / PPO / VerifAI runtime assurance (`baselines.py`), result loader (`results.py`).
- `scripts/` experiment and analysis entry points (see order below).
- `models/` trained learned verifier (`verifier.*`, ONNX FP32/INT8), PPO planner, VerifAI monitor.
- `data/` PRISM latency/validation measurements and dataset split; `results/` summaries.

## Reproduce
```
python3 scripts/gen_dataset.py data/d0_0.npz 0 70        # (repeat for 70,140,210,280,350 or use run_chunks.sh)
python3 scripts/train_verifier.py
python3 scripts/prism_profile.py 60                      # PRISM vs compiled solver; latency model
python3 scripts/train_ppo.py 2500000
python3 scripts/verifai_monitor.py 400
python3 scripts/run_paired.py --methods full,noeps,nogate,noexact,nomon,learned_nomon --n 60 --tag abl --label 3
python3 scripts/run_paired.py --methods exact,learned,mpc,rule,verifai,ppo --n 60 --tag ext
python3 scripts/run_paired.py --methods ideal --n 60 --tag ideal
python3 scripts/run_compare.py 10 0 1 embedded 2.5 2.5 "McQueenGuard,MPC,Rule,VerifAI,PPO"
for s in 0.00 0.50 1.00; do python3 scripts/shift_experiment.py gen $s 3000 30; done
python3 scripts/shift_experiment.py analyse
for s in 0.0 0.5 1.0; do python3 scripts/shift_experiment.py closed $s 0 1 5; done
scripts/build_all.sh                                     # benchmarks, analyses, figures, PDF (idle machine)
```
`--worker/--nworkers` split any `run_paired.py` call over several processes. In the submitted results, episodes
e = 0..29 of the exact-only, learned-only, MPC, rule-based, VerifAI and PPO methods come from `run_compare.py`
(`results/compare*_w*.pkl`), and e = 30..59 from `run_paired.py --tag ext --e0 30`.
