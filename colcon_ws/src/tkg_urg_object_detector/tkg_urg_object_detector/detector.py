import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
from numba import jit

@jit(nopython=True, cache=True)
def cluster_points(ranges, first_scan, angle_min, angle_increment):
    initial_clusters = []
    is_near_first_scan = False
    x_ref = 0.0
    y_ref = 0.0
    target_radius_max = 0.6

    for i in range(len(ranges)):
        # 最大距離は射程範囲に合わせて調整
        if ranges[i] == np.inf or ranges[i] <= 0.1 or ranges[i] >= 10.0:
            continue
        # 最初のスキャンと近い場合には除外（検出距離による）
        if abs(ranges[i] - first_scan[i]) < 0.2:
            is_near_first_scan = True
            angle_ref = angle_min + i * angle_increment
            x_ref = first_scan[i] * math.cos(angle_ref)
            y_ref = first_scan[i] * math.sin(angle_ref)
            continue
        angle = angle_min + i * angle_increment
        x = ranges[i] * math.cos(angle)
        y = ranges[i] * math.sin(angle)
        # 最初のスキャンと近い場合には除外（位置基準による）
        if is_near_first_scan:
            if math.sqrt((x - x_ref) ** 2 + (y - y_ref) ** 2) < 0.05:
                x_ref = x
                y_ref = y
                continue
        # CoREjp 2025 フィールド外に発射しないための特殊条件
        if y > 0 and ranges[i] >= 7.5:
            continue
        is_near_first_scan = False

        is_cluster_detected = False
        for i in range(len(initial_clusters)):
            dist = math.sqrt((x - initial_clusters[i][0]) ** 2 + (y - initial_clusters[i][1]) ** 2)
            if dist < 0.05 + initial_clusters[i][2] :
                is_cluster_detected = True
                current_cluster_x = (x + initial_clusters[i][0] * initial_clusters[i][3]) / (initial_clusters[i][3] + 1)
                current_cluster_y = (y + initial_clusters[i][1] * initial_clusters[i][3]) / (initial_clusters[i][3] + 1)
                current_cluster_radius = initial_clusters[i][2]
                if dist > initial_clusters[i][2]:
                    current_cluster_radius = dist
                initial_clusters[i][0] = current_cluster_x
                initial_clusters[i][1] = current_cluster_y
                initial_clusters[i][2] = current_cluster_radius
                initial_clusters[i][3] = initial_clusters[i][3] + 1
                break
        if not is_cluster_detected:
            initial_clusters.append([x, y, 0.0, 1])

    # すべてのスキャン処理が終わった後に、点の数が4以上のクラスタのみを残す
    initial_clusters = [cluster for cluster in initial_clusters if cluster[3] > 3]

    return initial_clusters


    merged = initial_clusters.copy()
    changed = True

    while changed and len(merged) > 1:
        changed = False
        num = len(merged)
        best_pair = None
        best_distance = float('inf')
        # すべてのペアの中心間距離を計算
        for i in range(num):
            for j in range(i+1, num):
                d = math.sqrt((merged[i][0] - merged[j][0])**2 + (merged[i][1] - merged[j][1])**2)
                if d < best_distance:
                    best_distance = d
                    best_pair = (i, j)
        if best_pair is not None:
            i, j = best_pair
            # マージ後のクラスタのパラメータを計算
            count = merged[i][3] + merged[j][3]
            new_x = (merged[i][0] * merged[i][3] + merged[j][0] * merged[j][3]) / count
            new_y = (merged[i][1] * merged[i][3] + merged[j][1] * merged[j][3]) / count
            # 各クラスタの中心から新しい重心までの距離に、それぞれの半径を加えたものの最大値を新半径とする
            r_i = math.sqrt((merged[i][0] - new_x)**2 + (merged[i][1] - new_y)**2) + merged[i][2]
            r_j = math.sqrt((merged[j][0] - new_x)**2 + (merged[j][1] - new_y)**2) + merged[j][2]
            new_radius = max(r_i, r_j)
            # マージ判定：新たな半径が目標上限以下、またはペアの中心間距離が非常に小さい場合はマージする
            if new_radius <= target_radius_max or best_distance < 0.1:
                merged_cluster = [new_x, new_y, new_radius, count]
                # マージ済みのクラスタをリストから除去し、マージクラスタを追加
                new_merged = []
                for k in range(num):
                    if k not in best_pair:
                        new_merged.append(merged[k])
                new_merged.append(merged_cluster)
                merged = new_merged
                changed = True
    return merged


class ScanProcessor(Node):
    def __init__(self):
        super().__init__('scan_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(PoseArray, 'clusters', 10)
        self.first_scan = None
        self.get_logger().info('ScanProcessor node has been started.')

    def scan_callback(self, msg):
        # Convert ranges to a numpy array
        ranges = np.array(msg.ranges)
        
        # Ignore invalid data (e.g., inf or NaN values)
        ranges = np.where(np.isfinite(ranges), ranges, np.inf)

        # Save the first scan
        if self.first_scan is None:
            self.first_scan = np.copy(ranges)
            self.get_logger().info('First scan recorded.')
            return

        # Cluster points that are not -1.0 and within 0.1m of each other
        clusters = cluster_points(ranges, self.first_scan, np.float64(msg.angle_min), np.float64(msg.angle_increment))

        self.publish_clusters(clusters)

    def publish_clusters(self, clusters):
        pose_array = PoseArray()
        pose_array.header.frame_id = 'laser'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for cluster in clusters:
            if len(cluster) == 0:
                continue

            pose = Pose()
            pose.position.x = cluster[0]
            pose.position.y = cluster[1]
            pose.position.z = 0.0

            pose.orientation.w = 1.0

            pose_array.poses.append(pose)

        self.publisher.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = ScanProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

