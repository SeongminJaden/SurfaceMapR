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
import matplotlib.colors as mcolors

class RealtimePCDHeatmapNode(Node):
    def __init__(self):
        super().__init__('realtime_pcd_heatmap_node')

        # sim time 사용 설정
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # PointCloud2 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne/velodyne_lidar/out',
            self.pc_callback,
            10
        )

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.all_points = []  # 누적 포인트
        self.running = True

        self.output_dir = os.path.join(os.getcwd(), "pcd_output")
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"PCD output directory: {self.output_dir}")

        # 엔터 입력 대기 스레드
        threading.Thread(target=self.wait_for_key, daemon=True).start()

    def pc_callback(self, msg: PointCloud2):
        if not self.running:
            return

        # TF가 준비되었는지 확인
        if not self.tf_buffer.can_transform('odom', 'velodyne_link', msg.header.stamp, timeout=rclpy.duration.Duration(seconds=0.1)):
            self.get_logger().warn("TF not ready, skipping frame")
            return

        try:
            # 메시지 timestamp 기준으로 TF lookup
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'odom',
                'velodyne_link',
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup error: {e}")
            return

        trans_mat = self.transform_to_matrix(trans)

        points_list = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pt = np.array([point[0], point[1], point[2], 1.0])
            pt_transformed = trans_mat @ pt
            points_list.append(pt_transformed[:3])

        if points_list:
            points_np = np.array(points_list)
            self.all_points.append(points_np)
            self.get_logger().info(f"Frame received: {points_np.shape[0]} points")

    def transform_to_matrix(self, trans: TransformStamped):
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
        print("Press ENTER to stop PCD collection and save final heatmap...")
        input()
        self.get_logger().info("Stopping PCD collection...")
        self.running = False
        self.save_final_heatmap()
        rclpy.shutdown()

    def save_final_heatmap(self):
        if not self.all_points:
            self.get_logger().warn("No points collected.")
            return

        all_points_np = np.vstack(self.all_points)
        combined_pcd = o3d.geometry.PointCloud()
        combined_pcd.points = o3d.utility.Vector3dVector(all_points_np)
        pcd_file = os.path.join(self.output_dir, "combined.pcd")
        o3d.io.write_point_cloud(pcd_file, combined_pcd)
        self.get_logger().info(f"Saved combined PCD: {pcd_file}")

        # 최종 히트맵
        resolution = 0.1
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
                z_map[y_idx, x_idx] = max(z_map[y_idx, x_idx], pt[2])

        # 색상 정규화 (0 ~ 100m)
        norm = mcolors.Normalize(vmin=0, vmax=100)

        plt.ioff()
        plt.figure(figsize=(10, 8))
        plt.imshow(
            z_map,
            cmap='jet',
            origin='lower',
            extent=[x_min, x_max, y_min, y_max],
            norm=norm
        )
        plt.colorbar(label='Height (Z)')
        plt.title("Final Z-Axis Heatmap (0–100m scale)")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.tight_layout()
        heatmap_file = os.path.join(self.output_dir, "final_heatmap.png")
        plt.savefig(heatmap_file)
        plt.show()
        self.get_logger().info(f"Saved final heatmap: {heatmap_file}")

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
