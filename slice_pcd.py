import open3d as o3d
import numpy as np

# 1. PCD 파일 읽기
pcd = o3d.io.read_point_cloud("./pcd_output/combined.pcd")

# 2. NumPy 배열로 변환
points = np.asarray(pcd.points)

# 3. z값 필터링 (예: z > 1.5m 제거)
z_threshold = 0.4  # m 단위로 설정
filtered_points = points[points[:, 2] <= z_threshold]

# 4. 새로운 PointCloud 객체 생성
pcd_filtered = o3d.geometry.PointCloud()
pcd_filtered.points = o3d.utility.Vector3dVector(filtered_points)

# 5. 새로운 PCD로 저장
o3d.io.write_point_cloud("./pcd_output/combined_filtered.pcd", pcd_filtered)

print(f"원본 포인트: {len(points)}, 필터링 후 포인트: {len(filtered_points)}")
