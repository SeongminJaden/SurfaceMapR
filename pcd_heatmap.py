import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# 1. PCD 파일 불러오기
pcd = o3d.io.read_point_cloud("map.pcd")
points = np.asarray(pcd.points)

# 2. 파라미터 설정
resolution = 0.1  # grid cell 크기 (단위: meter)

# 3. XY 그리드에 투영
x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])

x_bins = int((x_max - x_min) / resolution) + 1
y_bins = int((y_max - y_min) / resolution) + 1

# 4. 각 그리드 셀에서 Z값 평균 계산
z_map = np.full((y_bins, x_bins), np.nan)

for pt in points:
    x_idx = int((pt[0] - x_min) / resolution)
    y_idx = int((pt[1] - y_min) / resolution)
    if np.isnan(z_map[y_idx, x_idx]):
        z_map[y_idx, x_idx] = pt[2]
    else:
        z_map[y_idx, x_idx] = (z_map[y_idx, x_idx] + pt[2]) / 2.0  # 평균값 누적

# 5. 시각화
plt.figure(figsize=(10, 8))
plt.imshow(z_map, cmap='jet', origin='lower', extent=[x_min, x_max, y_min, y_max])
plt.colorbar(label='Height (Z)')
plt.title("Z-Axis Heatmap from PCD")
plt.xlabel("X")
plt.ylabel("Y")
plt.tight_layout()
plt.show()
