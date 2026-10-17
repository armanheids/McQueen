"""Resident memory of a minimal ONNX Runtime inference process (psutil), FP32 and INT8."""
import json, subprocess, sys
code = '''
import onnxruntime as ort, numpy as np, psutil, os, json
p=psutil.Process(os.getpid()); r0=p.memory_info().rss
so=ort.SessionOptions(); so.intra_op_num_threads=1; so.inter_op_num_threads=1
s=ort.InferenceSession("MODEL",so,providers=["CPUExecutionProvider"])
x=np.zeros((1,18),np.float32)
for i in range(5000): s.run(None,{"x":x})
print(json.dumps(dict(after_import=r0/2**20, total=p.memory_info().rss/2**20)))
'''
base = float(subprocess.run([sys.executable, "-c", "import psutil,os;print(psutil.Process(os.getpid()).memory_info().rss/2**20)"], capture_output=True, text=True).stdout)
r = {"python_interpreter": base}
for k, m in (("fp32", "models/verifier.onnx"), ("int8", "models/verifier_int8.onnx")):
    o = json.loads(subprocess.run([sys.executable, "-c", code.replace("MODEL", m)], capture_output=True, text=True).stdout.strip().splitlines()[-1])
    r[f"ort_{k}_process"] = o["total"]
    r[f"ort_{k}_session_increment"] = o["total"] - o["after_import"]
E = json.load(open("results/embedded_profile.json"))
E["memory_mb"].update(r)
E["memory_mb"].pop("ort_int8_increment", None); E["memory_mb"].pop("ort_fp32_increment", None)
json.dump(E, open("results/embedded_profile.json", "w"), indent=1)
print(E["memory_mb"])
