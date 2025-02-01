import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseArray, Pose
from std_msgs.msg import Float64, Int64, Empty, Bool
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge

from tkg_autorobot_commander.target_class import Target
from tkg_autorobot_commander.extract_object_region import ExtractObject

class Auto(Node):

    def __init__(self):
        super().__init__('auto')
        # 砲塔の旋回中心からLidarまでの位置
        self.POSE_OFFSET_X = 0.3
        self.POSE_OFFSET_Y = 0.0

        self.AUTO_MODE_INIT = 0
        self.AUTO_MODE_TRACKING = 1
        self.AUTO_MODE_ATTACK = 2
        self.auto_mode = self.AUTO_MODE_INIT

        self.attack_counter = 0

        self.next_id = 0

        self.hammer_pub = self.create_publisher(Empty, "/hammer", 10)
        self.mazemaze_pub = self.create_publisher(Empty, "/mazemaze", 10)
        self.roller_pub = self.create_publisher(Float64, "/roller", 10)
        self.control_pub = self.create_publisher(Bool, "/control", 10)
        self.yaw_pub = self.create_publisher(Float64, "/yaw", 10)
        self.pitch_pub = self.create_publisher(Float64, "/pitch", 10)

        self.object_image_pub = self.create_publisher(Image, "/object_image", 10)

        self.camera_mat = None
        self.dist_coeffs = None
        self.extractor = None
        self.image = None
        self.bridge = CvBridge()
        self.camera_image_sub = self.create_subscription(Image,'/camera/camera/color/image_raw',self.image_callback,10)
        self.camera_info_sub = self.create_subscription(CameraInfo,'/camera/camera/color/camera_info',self.camera_info_callback,10)

        self.poses = []
        self.target_list = []
        self.selected_target_id = -1
        self.selected_target_index = 0
        self.current_target_pose = Pose()
        self.timer_processing = False
        self.clusters_sub = self.create_subscription(PoseArray,'/clusters',self.clusters_callback,10)

        self.turret_data = Vector3()
        self.turret_sub = self.create_subscription(Vector3,'/current_turret_pose',self.turret_callback,10)

        #self.task_timer =  self.create_timer(0.1, self.timer_callback)

    def turret_callback(self, msg):
        self.turret_data = msg

    def image_callback(self, msg):
        try:
            # ROS2のImageメッセージをOpenCVの画像に変換
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def camera_info_callback(self, msg):
        self.camera_mat = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def clusters_callback(self, msg):
        if self.timer_processing == True:
            return
        self.timer_processing = True

        self.poses = msg.poses
        thread1 = threading.Thread(target=self.timer_callback)
        thread1.start()

    def timer_callback(self):
        current_yaw = self.turret_data.z
        current_pitch = self.turret_data.y

        (current_time, _) = self.get_clock().now().seconds_nanoseconds()
        self.target_list = [target for target in self.target_list if (current_time - target.checked_time) < 3]
        for target in self.target_list:
            if not target.state == Target.UNKNOWN:
                if (current_time - target.state_checked_time)  >= 30:
                    target.state = Target.UNKNOWN

        for pose in self.poses:
            pose.position.x += self.POSE_OFFSET_X
            pose.position.y += self.POSE_OFFSET_Y
            is_matched = False
            for target in self.target_list:
                if math.sqrt((pose.position.x - target.x) ** 2 + (pose.position.y - target.y) ** 2) < 0.5:
                    target.x = pose.position.x
                    target.y = pose.position.y
                    target.checked_time = current_time
                    
                    is_matched = True
                    break
            if is_matched == False:
                self.target_list.append(Target(self.next_id, pose.position.x, pose.position.y, current_time))
                self.next_id += 1
                if self.next_id >= 65534:
                    self.next_id = 0

        # 対象が選択されていないとき、新しい対象を選択
        if self.selected_target_id == -1:
            min_distance = 1000.0
            for target in self.target_list:
                if target.state == Target.UNKNOWN or target.state == Target.ENEMY:
                    distance = math.sqrt(target.x ** 2 + target.y ** 2)
                    if distance < min_distance:
                        min_distance = distance
                        self.selected_target_id = target.id
        self.get_logger().info(f"{len(self.target_list)}: {self.target_list}")

        # 対象が選択されているとき、現在の対象のインデックス番号を取得
        if not self.selected_target_id == -1:
            is_find = False
            for target_index in range(len(self.target_list)):
                if self.target_list[target_index].id == self.selected_target_id:
                    self.selected_target_index = target_index
                    is_find = True
                    break
            if is_find == False:
                self.selected_target_id = -1

        if self.auto_mode == self.AUTO_MODE_INIT:
            if self.camera_mat is not None and self.dist_coeffs is not None:
                self.extractor = ExtractObject(self.camera_mat, self.dist_coeffs)

                self.auto_mode = self.AUTO_MODE_TRACKING
    
        if self.auto_mode == self.AUTO_MODE_TRACKING:
            self.get_logger().info(f"TRACKING_MODE:")
            if not self.selected_target_id == -1:
                target_yaw = math.atan2(self.target_list[self.selected_target_index].y, self.target_list[self.selected_target_index].x)
                target_pitch = 0.0
                self.yaw_pub.publish(Float64(data=float(target_yaw)))
                self.pitch_pub.publish(Float64(data=float(target_pitch)))
                if abs(target_yaw - current_yaw) <= math.radians(5.0) and abs(target_pitch - current_pitch) <= math.radians(5.0):
                    if self.image is not None:
                        extract_image, result_image = self.extractor.extract(self.image, self.target_list[self.selected_target_index].x, self.target_list[self.selected_target_index].y, current_pitch, current_yaw, (256, 128))
                        ros_image = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
                        self.object_image_pub.publish(ros_image)
                        # TODO : image processing
                        (target_pitch, target_yaw, self.target_list[self.selected_target_index].state) = (target_pitch, target_yaw, Target.UNKNOWN)
                        self.target_list[self.selected_target_index].state_checked_time = current_time
                        if self.target_list[self.selected_target_index].state == Target.ENEMY:
                            self.auto_mode = self.AUTO_MODE_ATTACK
                        if self.target_list[self.selected_target_index].state == Target.OTHER:
                            self.selected_target_id = -1
                self.get_logger().info(f"{self.target_list[self.selected_target_index].x},{self.target_list[self.selected_target_index].y},{target_yaw}, {target_pitch}")

        if self.auto_mode == self.AUTO_MODE_ATTACK:
            self.get_logger().info(f"ATTACK_MODE:")
            if not self.selected_target_id == -1:
                extract_image, result_image = self.extractor.extract(self.image, self.target_list[self.selected_target_index].x, self.target_list[self.selected_target_index].y, current_pitch, current_yaw, (256, 128))
                ros_image = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
                self.object_image_pub.publish(ros_image)
                # TODO: image processing and calclate target_pitch
                (target_pitch, target_yaw, self.target_list[self.selected_target_index].state) = (target_pitch, target_yaw, Target.ENEMY)
                self.target_list[self.selected_target_index].state_checked_time = current_time
                if self.target_list[self.selected_target_index].state == Target.ENEMY:
                    self.yaw_pub.publish(Float64(data=float(target_yaw)))
                    self.pitch_pub.publish(Float64(data=float(target_pitch)))
                    if abs(target_yaw - current_yaw) <= math.radians(5.0) and abs(target_pitch - current_pitch) <= math.radians(5.0):
                        #self.roller_pub.publish(Float64(data=float(4000)))
                        self.hammer_pub.publish(Empty())
                        self.mazemaze_pub.publish(Empty())
                        self.attack_counter += 1
                        if self.attack_counter >= 3:
                            self.selected_target_id = -1
                            self.attack_counter = 0
                            self.auto_mode = self.AUTO_MODE_TRACKING
                else:
                    self.selected_target_id = -1
                    self.attack_counter = 0
                    self.auto_mode = self.AUTO_MODE_TRACKING
            else:
                self.selected_target_id = -1
                self.attack_counter = 0
                self.auto_mode = self.AUTO_MODE_TRACKING


        self.timer_processing = False

def main():
    rclpy.init()
    node = Auto()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
