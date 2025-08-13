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

class PCDHeatmapGridNode(Node):
    def __init__(self):
        super().__init__('pcd_heatmap_grid_node')

        # PointCloud2 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne/velodyne_lidar/out',  # 토픽 이름
            self.pc_callback,
            10
        )

        self.all_points = []  # 누적 포인트
        self.output_dir = os.path.join(os.getcwd(), "pcd_output")
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"PCD output directory: {self.output_dir}")

        self.running = True

        # 키 입력 감지 스레드
        threading.Thread(target=self.wait_for_key, daemon=True).start()

    def pc_callback(self, msg):
        if not self.running:
            return

        points_list = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        points_np = np.array(points_list)

        if points_np.size == 0:
            return

        self.all_points.append(points_np)
        self.get_logger().info(f"Frame received: {points_np.shape[0]} points")

    def wait_for_key(self):
        print("Press ENTER to stop PCD collection and generate grid heatmap...")
        input()
        self.get_logger().info("Stopping PCD collection...")
        self.running = False
        self.save_and_plot_heatmap()

        rclpy.shutdown()

    def save_and_plot_heatmap(self):
        if not self.all_points:
            self.get_logger().warn("No points collected.")
            return

        # 모든 포인트 합치기
        all_points_np = np.vstack(self.all_points)

        # PCD 저장
        combined_pcd = o3d.geometry.PointCloud()
        combined_pcd.points = o3d.utility.Vector3dVector(all_points_np)
        pcd_file = os.path.join(self.output_dir, "combined.pcd")
        o3d.io.write_point_cloud(pcd_file, combined_pcd)
        self.get_logger().info(f"Saved combined PCD: {pcd_file}")

        # 2D 그리드 기반 Z 히트맵 생성
        resolution = 0.1  # 그리드 크기 (meter)
        x_min, x_max = np.min(all_points_np[:, 0]), np.max(all_points_np[:, 0])
        y_min, y_max = np.min(all_points_np[:, 1]), np.max(all_points_np[:, 1])

        x_bins = int((x_max - x_min) / resolution) + 1
        y_bins = int((y_max - y_min) / resolution) + 1

        z_map = np.full((y_bins, x_bins), np.nan)

        for pt in all_points_np:
            x_idx = int((pt[0] - x_min) / resolution)
            y_idx = int((pt[1] - y_min) / resolution)
            if np.isnan(z_map[y_idx, x_idx]):
                z_map[y_idx, x_idx] = pt[2]
            else:
                z_map[y_idx, x_idx] = (z_map[y_idx, x_idx] + pt[2]) / 2.0

        # 시각화
        plt.figure(figsize=(10, 8))
        plt.imshow(z_map, cmap='jet', origin='lower', extent=[x_min, x_max, y_min, y_max])
        plt.colorbar(label='Height (Z)')
        plt.title("Z-Axis Heatmap from PCD")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.tight_layout()
        heatmap_file = os.path.join(self.output_dir, "combined_heatmap.png")
        plt.savefig(heatmap_file)
        plt.show()
        self.get_logger().info(f"Saved heatmap: {heatmap_file}")


def main(args=None):
    rclpy.init(args=args)
    node = PCDHeatmapGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.running:
            node.save_and_plot_heatmap()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
