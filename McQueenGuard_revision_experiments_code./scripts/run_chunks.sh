#!/bin/bash
# resumable chunked runner: run_chunks.sh <prefix> <first> <n_chunks> <chunk_size> [extra args...]
prefix=$1; first=$2; nch=$3; cs=$4; shift 4
cd /home/claude/mqg_exp
for ((c=0;c<nch;c++)); do
  s=$((first + c*cs))
  out="data/${prefix}_${s}.npz"
  [ -f "$out" ] && continue
  python3 scripts/gen_dataset.py "$out" $s $cs "$@"
done
