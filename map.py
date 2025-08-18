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
dae_file = "/home/terranox/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/gazebo_flat/meshes/gazebo_flat.dae"
samples_per_triangle = 100  # 삼각형당 샘플 포인트 수
resolution = 0.05           # 1셀당 meter
z_threshold = 0.5           # 이 값 이하를 바닥으로 간주하고 제외

# ---------------------------
# DAE 로드 및 포인트 샘플링
# ---------------------------
mesh = collada.Collada(dae_file)
points = []

for geom in mesh.geometries:
    for prim in geom.primitives:
        if isinstance(prim, collada.triangleset.TriangleSet):
            vertex_array = prim.vertex
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
