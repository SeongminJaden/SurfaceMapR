import open3d as o3d
import numpy as np
import pandas as pd

# 1. PCD 파일 읽기
pcd_file = "combined.pcd"  # 변환할 PCD 파일 경로
pcd = o3d.io.read_point_cloud(pcd_file)

# 2. 포인트 배열로 변환
points = np.asarray(pcd.points)  # shape: (N, 3)

# 3. CSV로 저장
csv_file = "combined.csv"
df = pd.DataFrame(points, columns=["x", "y", "z"])
df.to_csv(csv_file, index=False)

print(f"PCD -> CSV 변환 완료: {csv_file}")
