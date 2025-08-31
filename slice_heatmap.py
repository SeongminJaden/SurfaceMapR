import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# 1. PCD 읽기
pcd = o3d.io.read_point_cloud("./pcd_output/combined_filtered.pcd")
points = np.asarray(pcd.points)
x, y, z = points[:, 0], points[:, 1], points[:, 2]

# 2. 격자 생성 (작게 설정)
grid_size = 0.02  # 작게 할수록 선명
x_bins = np.arange(x.min(), x.max() + grid_size, grid_size)
y_bins = np.arange(y.min(), y.max() + grid_size, grid_size)

# 3. 2D 히스토그램으로 z값 평균 계산
heatmap_sum, _, _ = np.histogram2d(x, y, bins=[x_bins, y_bins], weights=z)
heatmap_count, _, _ = np.histogram2d(x, y, bins=[x_bins, y_bins])
heatmap = np.divide(heatmap_sum, heatmap_count, out=np.zeros_like(heatmap_sum), where=heatmap_count!=0)
heatmap[heatmap_count == 0] = np.nan  # 빈 공간은 NaN 처리

# 4. 절대 z값 기준 0~1 정규화
z_min = np.nanmin(heatmap)
z_max = np.nanmax(heatmap)
heatmap_norm = (heatmap - z_min) / (z_max - z_min)

# 5. 히트맵 시각화 (선명)
plt.figure(figsize=(12, 10))
cmap = plt.get_cmap('jet')  # 파랑->빨강
cmap.set_bad(color='white') # 빈 공간 흰색

im = plt.imshow(
    heatmap_norm.T,
    origin='lower',
    extent=[x.min(), x.max(), y.min(), y.max()],
    cmap=cmap,
    interpolation='nearest'
)
cbar = plt.colorbar(im)
cbar.set_label(f'Height (Z) [m], min={z_min:.2f}, max={z_max:.2f}')  # 단위 표시
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Sharp Top-down Height Heatmap')
plt.tight_layout()
plt.show()

