#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
import threading
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf_transformations

class RealtimePCDHeatmapNode(Node):
    def __init__(self):
        super().__init__('realtime_pcd_heatmap_node')

        # 시뮬레이션 시간을 사용하도록 설정
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # PointCloud2 토픽 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne/velodyne_lidar/out',  # LIDAR 데이터 토픽
            self.pc_callback,
            10  # 큐 사이즈
        )

        # TF를 사용하기 위한 Buffer와 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.all_points = []  # 누적 포인트를 저장할 리스트
        self.running = True   # 수집 중인지 여부

        # PCD 파일 저장 디렉토리 생성
        self.output_dir = os.path.join(os.getcwd(), "pcd_output")
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"PCD output directory: {self.output_dir}")

        # 엔터 입력을 기다리는 스레드 시작
        threading.Thread(target=self.wait_for_key, daemon=True).start()

    def pc_callback(self, msg: PointCloud2):
        """PointCloud2 메시지를 받아서 odom 좌표계로 변환 후 누적 저장"""
        if not self.running:
            return

        # TF가 준비되었는지 확인
        if not self.tf_buffer.can_transform('odom', 'velodyne_link', msg.header.stamp, timeout=rclpy.duration.Duration(seconds=0.1)):
            self.get_logger().warn("TF not ready, skipping frame")
            return

        try:
            # TF 변환 조회
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'odom',
                'velodyne_link',
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup error: {e}")
            return

        # TransformStamped → 4x4 변환 행렬
        trans_mat = self.transform_to_matrix(trans)

        points_list = []
        # PointCloud2 데이터 반복
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pt = np.array([point[0], point[1], point[2], 1.0])  # 동차좌표
            pt_transformed = trans_mat @ pt  # odom 좌표계로 변환
            points_list.append(pt_transformed[:3])

        if points_list:
            points_np = np.array(points_list)
            self.all_points.append(points_np)  # 누적
            self.get_logger().info(f"Frame received: {points_np.shape[0]} points")

    def transform_to_matrix(self, trans: TransformStamped):
        """TransformStamped를 4x4 변환 행렬로 변환"""
        t = [trans.transform.translation.x,
             trans.transform.translation.y,
             trans.transform.translation.z]
        q = [trans.transform.rotation.x,
             trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w]
        mat = tf_transformations.quaternion_matrix(q)
        mat[:3, 3] = t
        return mat

    def wait_for_key(self):
        """엔터 키 입력을 기다린 후 PCD 저장 및 종료"""
        print("Press ENTER to stop PCD collection and save final heatmap...")
        input()
        self.get_logger().info("Stopping PCD collection...")
        self.running = False
        self.save_final_heatmap()
        rclpy.shutdown()

    def save_final_heatmap(self):
        """누적된 포인트를 PCD와 Z-히트맵으로 저장"""
        if not self.all_points:
            self.get_logger().warn("No points collected.")
            return

        # 모든 프레임 포인트 합치기
        all_points_np = np.vstack(self.all_points)
        combined_pcd = o3d.geometry.PointCloud()
        combined_pcd.points = o3d.utility.Vector3dVector(all_points_np)
        pcd_file = os.path.join(self.output_dir, "combined.pcd")
        o3d.io.write_point_cloud(pcd_file, combined_pcd)
        self.get_logger().info(f"Saved combined PCD: {pcd_file}")

        # =================== 히트맵 생성 ===================
        resolution = 0.1  # 격자 하나의 크기(m). 이걸 바꾸면 히트맵 해상도가 바뀜
        x_min, x_max = np.min(all_points_np[:, 0]), np.max(all_points_np[:, 0])
        y_min, y_max = np.min(all_points_np[:, 1]), np.max(all_points_np[:, 1])
        x_bins = int((x_max - x_min) / resolution) + 1
        y_bins = int((y_max - y_min) / resolution) + 1

        # Z값을 담을 배열 초기화 (nan으로)
        z_map = np.full((y_bins, x_bins), np.nan)
        for pt in all_points_np:
            x_idx = int((pt[0] - x_min) / resolution)
            y_idx = int((pt[1] - y_min) / resolution)
            if np.isnan(z_map[y_idx, x_idx]):
                z_map[y_idx, x_idx] = pt[2]  # 첫 Z값
            else:
                z_map[y_idx, x_idx] = max(z_map[y_idx, x_idx], pt[2])  # Z 최대값

        # =================== 히트맵 시각화 ===================
        plt.figure(figsize=(10, 8))  # 그래프 너비(width)와 높이(height) 조절 (인치 단위)
        plt.imshow(z_map, cmap='jet', origin='lower',
                   extent=[x_min, x_max, y_min, y_max])
        plt.colorbar(label='Height (Z)')  # Z축 단위 표시
        plt.title("Final Z-Axis Heatmap")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.tight_layout()
        heatmap_file = os.path.join(self.output_dir, "final_heatmap.png")
        plt.savefig(heatmap_file)
        plt.show()
        self.get_logger().info(f"Saved final heatmap: {heatmap_file}")

        # ---------------- Z축 단위 변경 ----------------
        # 예를 들어 Z를 cm 단위로 보고 싶으면 아래처럼 하면 됨
        # plt.colorbar(label='Height (Z) [cm]')  # 단위만 바꿔주면 됨
        # z_map_cm = z_map * 100

def main(args=None):
    rclpy.init(args=args)
    node = RealtimePCDHeatmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.running:
            node.save_final_heatmap()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
