import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
from numba import jit

@jit(nopython=True, cache=True)
def cluster_points(ranges, angle_min, angle_increment):
    initial_clusters = []
    current_cluster_size = 0
    x_sum = 0.0
    y_sum = 0.0
    first_x = 0.0
    first_y = 0.0
    current_x = -1.0
    current_y = -1.0

    for i in range(len(ranges)):
        if ranges[i] == np.inf or ranges[i] <= 0.1 or ranges[i] >= 30.0:
            continue
        angle = angle_min + i * angle_increment
        x = ranges[i] * math.cos(angle)
        y = ranges[i] * math.sin(angle)
        if math.sqrt((x - current_x) ** 2 + (y - current_y) ** 2) > 0.05:
            if current_cluster_size >= 3:
                current_cluster_x = x_sum / current_cluster_size
                current_cluster_y = y_sum / current_cluster_size
                current_cluster_radius = math.sqrt((x - first_x) ** 2 + (y - first_y) ** 2) / 2.0
                initial_clusters.append([current_cluster_x, current_cluster_y, current_cluster_radius])
            first_x = x
            first_y = y
            x_sum = x
            y_sum = y
            current_cluster_size = 1
        else:
            x_sum += x
            y_sum += y
            current_cluster_size += 1
        current_x = x
        current_y = y

    clusters = []
    current_i = 0
    for i in range(len(initial_clusters)):
        a_x = initial_clusters[current_i][0]
        a_y = initial_clusters[current_i][1]
        a_r = initial_clusters[current_i][2]
        b_x = initial_clusters[i][0]
        b_y = initial_clusters[i][1]
        b_r = initial_clusters[i][2]
        cluster_size = a_r + b_r + math.sqrt((a_x - b_x) ** 2 + (a_y - b_y) ** 2)
        if cluster_size >= 1.0 or i == len(initial_clusters)-1:
            cluster_x = (a_x + b_x) / 2.0
            cluster_y = (a_y + b_y) / 2.0
            cluster_radius = (a_r + b_r + math.sqrt((a_x - b_x) ** 2 + (a_y - b_y) ** 2)) / 2.0
            if cluster_radius <= 0.5:
                clusters.append([cluster_x, cluster_y, cluster_radius])
            current_i = i

    return clusters

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

        # Replace points within 0.05m of the first scan with -1.0m
        diff = np.abs(ranges - self.first_scan)
        ranges[diff <= 0.2] = np.inf

        # Cluster points that are not -1.0 and within 0.1m of each other
        clusters = cluster_points(ranges, np.float64(msg.angle_min), np.float64(msg.angle_increment))

        # Log the size of each cluster
        self.log_clusters(clusters)
        self.publish_clusters(clusters)

    def log_clusters(self, clusters):
        for i, cluster in enumerate(clusters):
            self.get_logger().info(f'Cluster {i + 1}: x = {cluster[0]}, y = {cluster[1]}, radius = {cluster[2]}')

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

