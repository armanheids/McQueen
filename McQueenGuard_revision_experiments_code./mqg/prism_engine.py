"""Thin client for the persistent PRISM verification server (warm JVM, PRISM 4.8.1)."""
import os, subprocess, tempfile, time, itertools
import numpy as np
from .pmdp import prism_model, PRISM_PROPS, H_MDP

SERVER = os.path.join(os.path.dirname(__file__), "..", "prism_driver", "run_server.sh")


class PrismEngine:
    def __init__(self, engine="sparse"):
        self.p = subprocess.Popen([SERVER, engine], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self.dir = tempfile.mkdtemp(prefix="mqg_prism_")
        self.props = os.path.join(self.dir, "q.props")
        with open(self.props, "w") as f:
            f.write(PRISM_PROPS)
        self.n = itertools.count()

    def verify(self, q):
        """Returns (p_unsafe[5], U[5], wall_ms, parse_ms, check_ms)."""
        path = os.path.join(self.dir, f"m{next(self.n) % 4}.pm")
        t0 = time.perf_counter()
        with open(path, "w") as f:
            f.write(prism_model(q))
        self.p.stdin.write(f"{path} {self.props}\n")
        self.p.stdin.flush()
        line = self.p.stdout.readline().strip()
        wall = (time.perf_counter() - t0) * 1000
        if not line.startswith("OK"):
            raise RuntimeError(line)
        parts = line.split()
        blocks = " ".join(parts[4:]).split("|")
        res = []
        for b in blocks[:2]:
            arr = np.zeros(5)
            for tok in b.split():
                k, v = tok.split(":")
                arr[int(k)] = float(v)
            res.append(arr)
        vals = np.concatenate(res)
        return vals[:5], vals[5:10] / H_MDP, wall, float(parts[1]), float(parts[2])

    def close(self):
        try:
            self.p.stdin.write("QUIT\n"); self.p.stdin.flush(); self.p.wait(5)
        except Exception:
            self.p.kill()
