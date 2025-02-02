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
from tkg_autorobot_commander.damage_panel_detection import DamagePanelDetection
from tkg_autorobot_commander.calc_launch_angle import calc_best_launch_angle

class Auto(Node):

    def __init__(self):
        super().__init__('auto')
        # 砲塔の旋回中心からLidarまでの位置
        self.POSE_OFFSET_X = 0.3
        self.POSE_OFFSET_Y = 0.0
        self.TARGET_RPM = 6000

        self.AUTO_MODE_INIT = 0
        self.AUTO_MODE_TRACKING = 1
        self.AUTO_MODE_ATTACK = 2
        self.auto_mode = self.AUTO_MODE_INIT

        self.recog_fail_counter = 0
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
        #TODO: チームによる設定をパラメータで設定できるようにする
        # 敵がRED
        #self.detector = DamagePanelDetection(15, 6, 255, 20, 8, 10)
        #敵がBLUE
        self.detector = DamagePanelDetection(90, 6, 255, 20, 8, 10)
        self.image = None
        self.bridge = CvBridge()
        self.camera_image_sub = self.create_subscription(Image,'/camera/camera/color/image_raw',self.image_callback,10)
        self.camera_info_sub = self.create_subscription(CameraInfo,'/camera/camera/color/camera_info',self.camera_info_callback,10)

        self.motor0_rpm = 0
        self.motor1_rpm = 0
        self.subscription0 = self.create_subscription(Float64, '/can_node/c620_0/rpm', self.motor0_callback, 10)
        self.subscription1 = self.create_subscription(Float64, '/can_node/c620_1/rpm', self.motor1_callback, 10)

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

    def motor0_callback(self, msg):
        self.motor0_rpm = msg.data

    def motor1_callback(self, msg):
        self.motor1_rpm = msg.data

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
            if self.extractor is None and self.camera_mat is not None and self.dist_coeffs is not None:
                self.extractor = ExtractObject(self.camera_mat, self.dist_coeffs)

            if self.extractor is not None and self.image is not None:
                self.auto_mode = self.AUTO_MODE_TRACKING
    
        if self.auto_mode == self.AUTO_MODE_TRACKING:
            # テストコード
            if False: #not self.selected_target_id == -1:
                extract_image, result_image = self.extractor.extract(self.image, self.target_list[self.selected_target_index].x, self.target_list[self.selected_target_index].y, current_pitch, current_yaw, (256, 128))
                (detect_point_x, detect_point_y), color_extract_result = self.detector.detect(extract_image)
                if detect_point_x >= 0 and detect_point_y >= 0:
                    (result_x, result_y) = self.extractor.convert_croped_point_to_image_point(detect_point_x, detect_point_y)
                    damage_panel_position = self.extractor.convert_image_point_to_target_point(result_x, result_y, math.sqrt(self.target_list[self.selected_target_index].x ** 2 + self.target_list[self.selected_target_index].y ** 2))
                    target_yaw = math.atan2(damage_panel_position[1], damage_panel_position[0])
                    target_pitch = calc_best_launch_angle(4000, math.sqrt(damage_panel_position[0] ** 2 + damage_panel_position[1] ** 2), damage_panel_position[2])
                    self.get_logger().info(f"damage_panel_position: {damage_panel_position}")
                    self.get_logger().info(f"target_yaw: {target_yaw}, target_pitch: {target_pitch}")
                    cv2.drawMarker(result_image, (int(result_x), int(result_y)), (0, 255, 0), thickness=3)
                ros_image = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
                self.object_image_pub.publish(ros_image)
            self.get_logger().info(f"TRACKING_MODE:")
            self.roller_pub.publish(Float64(data=float(0.0)))
            if not self.selected_target_id == -1:
                target_yaw = math.atan2(self.target_list[self.selected_target_index].y, self.target_list[self.selected_target_index].x)
                target_pitch = 0.0
                self.yaw_pub.publish(Float64(data=float(target_yaw)))
                self.pitch_pub.publish(Float64(data=float(target_pitch)))
                if abs(target_yaw - current_yaw) <= math.radians(5.0) and abs(target_pitch - current_pitch) <= math.radians(5.0):
                    extract_image, result_image = self.extractor.extract(self.image, self.target_list[self.selected_target_index].x, self.target_list[self.selected_target_index].y, current_pitch, current_yaw, (256, 128))
                    (detect_point_x, detect_point_y), color_extract_result = self.detector.detect(extract_image)
                    if detect_point_x >= 0 and detect_point_y >= 0: # 敵のダメージパネルを検出できた
                        (result_x, result_y) = self.extractor.convert_croped_point_to_image_point(detect_point_x, detect_point_y)
                        cv2.drawMarker(result_image, (int(result_x), int(result_y)), (0, 255, 0), thickness=3)

                        self.target_list[self.selected_target_index].state = Target.ENEMY
                        self.recog_fail_counter = 0
                        self.auto_mode = self.AUTO_MODE_ATTACK
                    else:
                        self.recog_fail_counter += 1
                        if self.recog_fail_counter > 10:
                            # TODO: デバッグが済んだら、状態をOTHERに設定するようにする
                            self.target_list[self.selected_target_index].state = Target.UNKNOWN #Target.OTHER
                            self.selected_target_id = -1
                    ros_image = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
                    self.object_image_pub.publish(ros_image)

                    self.target_list[self.selected_target_index].state_checked_time = current_time
                self.get_logger().info(f"{self.target_list[self.selected_target_index].x},{self.target_list[self.selected_target_index].y},{target_yaw}, {target_pitch}")
            else:
                self.recog_fail_counter = 0

        if self.auto_mode == self.AUTO_MODE_ATTACK:
            self.get_logger().info(f"ATTACK_MODE:")
            # TODO: デバッグが済んだら、ローラを回転させる
            #self.roller_pub.publish(Float64(data=float(self.TARGET_RPM)))
            current_rpm = (abs(self.motor0_rpm) + abs(self.motor1_rpm)) / 2.0
            if not self.selected_target_id == -1:
                extract_image, result_image = self.extractor.extract(self.image, self.target_list[self.selected_target_index].x, self.target_list[self.selected_target_index].y, current_pitch, current_yaw, (256, 128))
                (detect_point_x, detect_point_y), color_extract_result = self.detector.detect(extract_image)
                if detect_point_x >= 0 and detect_point_y >= 0: # 敵のダメージパネルを検出できた
                    (result_x, result_y) = self.extractor.convert_croped_point_to_image_point(detect_point_x, detect_point_y)
                    damage_panel_position = self.extractor.convert_image_point_to_target_point(result_x, result_y, math.sqrt(self.target_list[self.selected_target_index].x ** 2 + self.target_list[self.selected_target_index].y ** 2))
                    self.get_logger().info(f"{damage_panel_position}")
                    cv2.drawMarker(result_image, (int(result_x), int(result_y)), (0, 255, 0), thickness=3)

                    self.target_list[self.selected_target_index].state = Target.ENEMY
                    target_yaw = math.atan2(damage_panel_position[1], damage_panel_position[0])
                    target_pitch = calc_best_launch_angle(current_rpm, math.sqrt(damage_panel_position[0] ** 2 + damage_panel_position[1] ** 2), damage_panel_position[2])
                    self.yaw_pub.publish(Float64(data=float(target_yaw)))
                    if target_pitch > math.radians(-8):
                        self.pitch_pub.publish(Float64(data=float(target_pitch)))
                    if abs(target_yaw - current_yaw) <= math.radians(5.0) and abs(target_pitch - current_pitch) <= math.radians(5.0) and abs(current_rpm - self.TARGET_RPM) <= 100:
                        self.hammer_pub.publish(Empty())
                        self.mazemaze_pub.publish(Empty())
                        self.attack_counter += 1
                        if self.attack_counter >= 3:
                            self.selected_target_id = -1
                            self.attack_counter = 0
                            self.auto_mode = self.AUTO_MODE_TRACKING
                else:
                    self.target_list[self.selected_target_index].state = Target.UNKNOWN
                    self.selected_target_id = -1
                    self.attack_counter = 0
                    self.auto_mode = self.AUTO_MODE_TRACKING
                ros_image = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
                self.object_image_pub.publish(ros_image)

                self.target_list[self.selected_target_index].state_checked_time = current_time
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
