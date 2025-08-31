#!/usr/bin/env python3
import subprocess

# 1. pcd_heatmap.py 실행
print("===== Step 1: Running pcd_heatmap.py =====")
subprocess.run(["python3", "pcd_heatmap.py"])

# 2. slice_pcd.py 실행
print("===== Step 2: Running slice_pcd.py =====")
subprocess.run(["python3", "slice_pcd.py"])

# 3. slice_heatmap.py 실행
print("===== Step 3: Running slice_heatmap.py =====")
subprocess.run(["python3", "slice_heatmap.py"])

print("===== All steps completed =====")
