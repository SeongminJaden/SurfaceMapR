import collada
import numpy as np
from PIL import Image
import yaml

def sample_triangle(v0, v1, v2, n_samples):
    u = np.random.rand(n_samples)
    v = np.random.rand(n_samples)
    mask = u + v > 1
    u[mask] = 1 - u[mask]
    v[mask] = 1 - v[mask]
    w = 1 - (u + v)
    samples = u[:, None]*v0 + v[:, None]*v1 + w[:, None]*v2
    return samples

# ---------------------------
# 설정
# ---------------------------
dae_file = "./gazebo_flat.dae"  # DAE 파일 위치에 맞게 수정
samples_per_triangle = 200       # 삼각형당 샘플 포인트 수
resolution = 0.05                # 1셀당 meter
z_threshold = 3.0                # 이 값 이하를 바닥으로 간주하고 제외

# "원래 크기" (기준 맵 크기) - 두 번째 결과 기준
target_x_range = 24.3  # x축 범위 (-12.4 ~ 11.8)
target_y_range = 22.0  # y축 범위 (-12.2 ~ 9.8)

# ---------------------------
# DAE 로드 및 포인트 샘플링
# ---------------------------
mesh = collada.Collada(dae_file)
points = []

for geom in mesh.geometries:
    for prim in geom.primitives:
        if isinstance(prim, collada.triangleset.TriangleSet):
            vertex_array = prim.vertex

            # ---------------------------
            # 스케일 자동 보정
            # ---------------------------
            # 현재 mesh 크기
            cur_x_min, cur_x_max = vertex_array[:,0].min(), vertex_array[:,0].max()
            cur_y_min, cur_y_max = vertex_array[:,1].min(), vertex_array[:,1].max()
            cur_x_range = cur_x_max - cur_x_min
            cur_y_range = cur_y_max - cur_y_min

            # 스케일 비율 (x, y 중 큰 쪽 기준으로 보정)
            scale_x = target_x_range / cur_x_range
            scale_y = target_y_range / cur_y_range
            scale = min(scale_x, scale_y)

            vertex_array = vertex_array * scale

            # ---------------------------
            # 샘플링
            # ---------------------------
            for tri_indices in prim.vertex_index:
                v0 = vertex_array[tri_indices[0]]
                v1 = vertex_array[tri_indices[1]]
                v2 = vertex_array[tri_indices[2]]
                tri_points = sample_triangle(v0, v1, v2, samples_per_triangle)

                # 바닥 제외
                tri_points = tri_points[tri_points[:,2] > z_threshold]

                if len(tri_points) > 0:
                    points.append(tri_points)

points = np.vstack(points)
xy_points = points[:, :2]

# ---------------------------
# 2D 그리드 맵 생성
# ---------------------------
x_min, x_max = xy_points[:,0].min(), xy_points[:,0].max()
y_min, y_max = xy_points[:,1].min(), xy_points[:,1].max()

width = int(np.ceil((x_max - x_min) / resolution))
height = int(np.ceil((y_max - y_min) / resolution))

# 흑백 맵 초기화 (255=free, 0=occupied)
pgm = np.ones((height, width), dtype=np.uint8) * 255
print(f"맵 범위: x=({x_min:.2f},{x_max:.2f}), y=({y_min:.2f},{y_max:.2f})")

# 포인트를 그리드로 변환
for pt in xy_points:
    x_idx = int((pt[0] - x_min) / resolution)
    y_idx = int((pt[1] - y_min) / resolution)
    y_idx = height - 1 - y_idx  # 좌표계 뒤집기
    pgm[y_idx, x_idx] = 0

# ---------------------------
# PGM 저장
# ---------------------------
pgm_filename = "map_from_dae.pgm"
Image.fromarray(pgm).save(pgm_filename)
print(f"{pgm_filename} 저장 완료!")

# ---------------------------
# YAML 저장
# ---------------------------
yaml_filename = "map_from_dae.yaml"
map_yaml = {
    "image": pgm_filename,
    "resolution": resolution,
    "origin": [float(x_min), float(y_min), 0.0],  # 좌측 하단 (x, y, theta)
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196
}
with open(yaml_filename, "w") as f:
    yaml.dump(map_yaml, f, default_flow_style=False)
print(f"{yaml_filename} 저장 완료!") 

