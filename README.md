# TurtleBot3용 Velodyne LiDAR 시뮬레이션

이 저장소는 Gazebo에서 TurtleBot3 로봇에 Velodyne LiDAR를 시뮬레이션하는 설정과 사용 방법을 제공합니다.  
LiDAR 센서 정의를 개선했으며, 포인트 클라우드 수집 및 Z축 히트맵 생성 기능을 포함하고 있습니다.

---

## 기능 / 개선 사항

- TurtleBot3 SDF/URDF 모델에 **Velodyne LiDAR** 추가
- 현실적인 포인트 클라우드 생성을 위해 **수평/수직 해상도** 설정
- `gazebo_ros_velodyne_laser` **ROS2 플러그인** 통합
- 포인트 클라우드를 **주기적으로 수집**하고 `.pcd` 파일로 저장
- 수집된 포인트 클라우드로 자동 **Z축 히트맵** 생성(Open3D + Matplotlib)
- **CPU 기반 시뮬레이션** 지원 (GPU 불필요)
- Dae파일 -> 2D map 변환 map.py
- pcd -> CSV 변환 pcd.py

---

## 요구 사항

- ROS2 Humble
- Gazebo 11 이상
- Python 3
- Python 패키지:
  - `numpy`
  - `open3d`
  - `matplotlib`
  - `rclpy`
  - `sensor_msgs_py`

---

## 설치 방법

1. ROS2 워크스페이스에 저장소 클론:
```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/SeongminJaden/SurfaceMapR
```
2. turtlebot3_gazebo 복사하기(기존 turtlebot3 레포지토리의 "tuetlebot3_gazebo"를 삭제하고 해당 레포지토리에 있는 패키지로 대체합니다.)
3. 워크스페이스 빌드
```bash
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
```
## 사용 방법
1. Gazebo에서 TurtleBot3 + Velodyne 실행
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
2. 포인트 클라우드 수집(Slam, Navigation)
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/


## PCD 기반 히트맵 생성
PointCloud2 Topic을 Z축 히트맵으로 시각화:
```bash
python3 pcd_heatmap.py
```

## PCD -> CSV 생성
pcd파일을 CSV로 변환:
```bash
python3 pcd.py
```

## Dae -> 2d map
Dae 파일을 nav2 2d map으로 변환:
```bash
python3 map.py
```
- 스크립트 내 resolution 파라미터로 그리드 크기 조절 가능
- 히트맵은 XY 평면에서 Z 높이 분포를 나타냄
- 반드시 Navigation 사용 전 실행
- 완료 후 ENTER키를 눌러 종료 -> 히트맵 시각화

## 참고 사항
- 시뮬레이션에서 /velodyne/points 토픽이 올바르게 퍼블리시되는지 확인
- LiDAR 플러그인은 기본적으로 CPU 사용
- ROS2 노드에서 시뮬레이션 시간(use_sim_time:=True) 지원
