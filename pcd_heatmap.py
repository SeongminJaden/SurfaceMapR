#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import os
import threading
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf_transformations

class RealtimePCDCollectorNode(Node):
    def __init__(self):
        super().__init__('realtime_pcd_collector_node')

        # 시뮬레이션 시간 사용
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # PointCloud2 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne/gazebo_ros_velodyne/out',
            self.pc_callback,
            10
        )

        # TF 준비
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.all_points = []
        self.running = True
        self.stop_requested = False  # 엔터 입력 플래그

        # PCD 저장 디렉토리
        self.output_dir = os.path.join(os.getcwd(), "pcd_output")
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"PCD output directory: {self.output_dir}")

        # 엔터 입력 스레드
        threading.Thread(target=self.wait_for_enter, daemon=True).start()

    def pc_callback(self, msg: PointCloud2):
        if not self.running:
            return

        if not self.tf_buffer.can_transform('odom', 'velodyne_link', msg.header.stamp, timeout=rclpy.duration.Duration(seconds=0.1)):
            return

        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'odom',
                'velodyne_link',
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception:
            return

        trans_mat = self.transform_to_matrix(trans)

        points_list = []
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pt = np.array([x, y, z, 1.0])
            pt_transformed = trans_mat @ pt
            points_list.append(pt_transformed[:3])

        if points_list:
            points_np = np.array(points_list)
            self.all_points.append(points_np)
            # 안전하게 메인 스레드에서만 로그 출력
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

    def wait_for_enter(self):
        input("Press ENTER to stop PCD collection and save combined PCD...")
        print("ENTER pressed, stopping collection...")
        self.stop_requested = True  # 메인 스레드에서 종료 처리

    def save_combined_pcd(self):
        if not self.all_points:
            print("No points collected.")
            return

        all_points_np = np.vstack(self.all_points)

        combined_pcd = o3d.geometry.PointCloud()
        combined_pcd.points = o3d.utility.Vector3dVector(all_points_np)
        pcd_file = os.path.join(self.output_dir, "combined.pcd")
        o3d.io.write_point_cloud(pcd_file, combined_pcd)
        print(f"Saved combined PCD: {pcd_file}")

def main(args=None):
    rclpy.init(args=args)
    node = RealtimePCDCollectorNode()
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)  # spin_once로 체크
        # 엔터 눌리면 PCD 저장
        node.save_combined_pcd()
    except KeyboardInterrupt:
        node.save_combined_pcd()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

