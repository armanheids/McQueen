#!/bin/bash
# Final analysis + document build (run on an idle machine after all experiment runs have finished).
set -e
cd /home/claude/mqg_exp
export PYTHONWARNINGS=ignore
python3 scripts/embedded_profile.py > logs/b_embedded.log 2>&1
python3 scripts/mem_measure.py > logs/b_mem.log 2>&1
python3 scripts/latency_breakdown.py > logs/b_lat.log 2>&1
python3 scripts/analyse_compare.py > logs/b_c7.log 2>&1
python3 scripts/analyse_r2.py > logs/b_r2.log 2>&1
python3 scripts/analyse_c6.py stream closed embedded > logs/b_c6.log 2>&1
python3 scripts/make_response.py > logs/b_doc.log 2>&1
cd report
pdflatex -interaction=nonstopmode McQueenGuard_R1C6-7_R2C6-7_response.tex > build_final.log 2>&1 || true
pdflatex -interaction=nonstopmode McQueenGuard_R1C6-7_R2C6-7_response.tex > build_final.log 2>&1 || true
grep -E "^!|undefined" build_final.log || true
echo BUILD_DONE
