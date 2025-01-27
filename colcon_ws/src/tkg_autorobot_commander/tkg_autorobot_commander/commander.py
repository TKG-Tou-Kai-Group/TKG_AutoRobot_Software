import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseArray, Pose
from std_msgs.msg import Float64, Int64, Empty, Bool

from tkg_autorobot_commander.target_class import Target

class Auto(Node):

    def __init__(self):
        super().__init__('auto')

        self.AUTO_MODE_TRACKING = 0
        self.AUTO_MODE_ATTACK = 1
        self.auto_mode = self.AUTO_MODE_TRACKING

        self.hammer_pub = self.create_publisher(Empty, "/hammer", 10)
        self.mazemaze_pub = self.create_publisher(Empty, "/mazemaze", 10)
        self.roller_pub = self.create_publisher(Float64, "/roller", 10)
        self.control_pub = self.create_publisher(Bool, "/control", 10)
        self.yaw_pub = self.create_publisher(Float64, "/yaw", 10)
        self.pitch_pub = self.create_publisher(Float64, "/pitch", 10)

        self.poses = []
        self.target_list = []
        self.current_target_pose = Pose()
        self.turret_sub = self.create_subscription(Vector3,'/current_turret_pose',self.turret_callback,10)
        self.clusters_sub = self.create_subscription(PoseArray,'/clusters',self.clusters_callback,10)

        self.task_timer =  self.create_timer(2, self.timer_callback)
        #self.task_timer.reset()
        #self.task_timer.cancel()

        self.turret_data = Vector3()

        self.attack_counter = 0
        self.timer_processing = False
        self.cluster_processing = False

    def turret_callback(self, msg):
        self.turret_data = msg

    def clusters_callback(self, msg):
        if self.timer_processing == True:
            return
        self.poses = msg.poses

    def timer_callback(self):
        if self.timer_processing == True:
            return
        self.timer_processing = True

        current_yaw = self.turret_data.z
        current_pitch = self.turret_data.y

        current_time = self.get_clock().now().seconds()
        self.target_list = [target for target in self.target_list if target.checked_time - current_time < 5]
        for target in self.target_list:
            if not target.state == target.UNKNOWN:
                if target.state_checked_time - current_time >= 10:
                    target.state = target.UNKNOWN

        for pose in self.poses:
            is_matched = False
            for target in self.target_list:
                if math.sqrt((pose.x - target.x) ** 2 + (pose.y - target.y) ** 2) < 0.5:
                    target.x = pose.x
                    target.y = pose.y
                    target.checked_time = self.get_clock().now().seconds()
                    
                    is_matched = True
                    break
            if is_matched == False:
                self.target_list.append(Target(pose.x, pose.y, self.get_clock().now().seconds()))

        min_distance = 1000.0
        selected_target = None
        for target in self.target_list:
            if target.state == target.ENEMY:
                distance = math.sqrt(target.x ** 2 + target.y ** 2)
                if distance < min_distance:
                    min_distance = distance
                    selected_target = target
        if selected_target == None:
            self.timer_processing = False
            return
        self.current_target_pose.x = selected_target.x
        self.current_target_pose.y = selected_target.y
        target_yaw = math.atan2(self.current_target_pose.y, self.current_target_pose.x)
        target_pitch = 0.0

        if self.auto_mode == self.AUTO_MODE_TRACKING:
            self.yaw_pub.publish(Float64(data=float(target_yaw)))
            self.pitch_pub.publish(Float64(data=float(target_pitch)))
            if abs(target_yaw - current_yaw) <= math.radians(5.0) and abs(target_pitch - current_pitch) <= math.radians(5.0):
                # TODO : image processing
                if selected_target.state = selected_target.ENEMY:
                    self.auto_mode = self.AUTO_MODE_ATTACK

        if self.auto_mode == self.AUTO_MODE_ATTACK:
            # TODO: image processing and calclate target_pitch
            self.yaw_pub.publish(Float64(data=float(target_yaw)))
            self.pitch_pub.publish(Float64(data=float(target_pitch)))
            if abs(target_yaw - current_yaw) <= math.radians(5.0) and abs(target_pitch - current_pitch) <= math.radians(5.0):
                #self.roller_pub.publish(Float64(data=float(4000)))
                self.hammer_pub.publish(Empty())
                self.mazemaze_pub.publish(Empty())
                self.attack_counter += 1
                if self.attack_counter >= 3:
                    self.attack_counter = 0
                    self.auto_mode = self.AUTO_MODE_TRACKING

        self.timer_processing = True

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
