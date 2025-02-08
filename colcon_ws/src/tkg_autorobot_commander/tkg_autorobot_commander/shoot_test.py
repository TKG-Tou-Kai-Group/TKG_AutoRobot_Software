import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Empty, Bool
from sensor_msgs.msg import LaserScan
import numpy as np

from tkg_autorobot_commander.calc_launch_angle import calc_best_launch_angle

class ShootTest(Node):
    def __init__(self):
        super().__init__('shoot_test')

        self.TARGET_RPM = 6000

        self.AUTO_MODE_INIT = 0
        self.AUTO_MODE_SHOOT = 1
        self.AUTO_MODE_WAIT = 2
        self.AUTO_MODE_STOP = 3
        self.auto_mode = self.AUTO_MODE_INIT

        self.wait_counter = 0

        self.hammer_pub = self.create_publisher(Empty, "/hammer", 10)
        self.mazemaze_pub = self.create_publisher(Empty, "/mazemaze", 10)
        self.roller_pub = self.create_publisher(Float64, "/roller", 10)
        self.control_pub = self.create_publisher(Bool, "/control", 10)
        self.yaw_pub = self.create_publisher(Float64, "/yaw", 10)
        self.pitch_pub = self.create_publisher(Float64, "/pitch", 10)

        self.motor0_rpm = 0
        self.motor1_rpm = 0
        self.subscription0 = self.create_subscription(Float64, '/can_node/c620_0/rpm', self.motor0_callback, 10)
        self.subscription1 = self.create_subscription(Float64, '/can_node/c620_1/rpm', self.motor1_callback, 10)

        self.target_distance = None
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.turret_data = Vector3()
        self.turret_sub = self.create_subscription(Vector3,'/current_turret_pose',self.turret_callback,10)

        self.timer_processing = False
        self.timer = self.create_timer(0.01, self.timer_callback)

    def turret_callback(self, msg):
        self.turret_data = msg

    def motor0_callback(self, msg):
        self.motor0_rpm = msg.data

    def motor1_callback(self, msg):
        self.motor1_rpm = msg.data

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        front_index = int(abs(msg.angle_min / msg.angle_increment))
        self.target_distance = ranges[front_index]
        if self.target_distance == np.inf or self.target_distance <= 0.1 or self.target_distance >= 10.0:
            self.target_distance = -1.0

    def timer_callback(self):
        if self.timer_processing == True:
            return
        self.timer_processing = True

        current_yaw = self.turret_data.z
        current_pitch = self.turret_data.y
        current_rpm = (abs(self.motor0_rpm) + abs(self.motor1_rpm)) / 2.0

        if self.auto_mode == self.AUTO_MODE_INIT:
            self.get_logger().info(f"INIT_MODE:")
            self.control_pub.publish(Bool(data=True))
            if self.target_distance is not None:
                if self.target_distance < 0:
                    self.auto_mode = self.AUTO_MODE_STOP
                else:
                    self.roller_pub.publish(Float64(data=float(self.TARGET_RPM)))
                    self.auto_mode = self.AUTO_MODE_SHOOT

        elif self.auto_mode == self.AUTO_MODE_SHOOT:
            self.get_logger().info(f"SHOOT_MODE:")
            target_yaw = 0.0
            target_pitch = calc_best_launch_angle(4000, self.target_distance, -0.03)
            self.get_logger().info(f"target_yaw: {target_yaw}, target_pitch: {target_pitch}")
            self.yaw_pub.publish(Float64(data=float(target_yaw)))
            if target_pitch > math.radians(-8):
                self.pitch_pub.publish(Float64(data=float(target_pitch)))
            else:
                self.auto_mode = self.AUTO_MODE_STOP
            yaw_angle_tolerance = math.radians(0.5) #math.atan2(0.05, self.target_distance)
            pitch_angle_tolerance = math.radians(0.5) #math.atan2(0.05, self.target_distance)
            print(abs(target_yaw - current_yaw), abs(target_pitch - current_pitch), pitch_angle_tolerance)
            if abs(target_yaw - current_yaw) <= yaw_angle_tolerance and abs(target_pitch - current_pitch) <= pitch_angle_tolerance and abs(current_rpm - self.TARGET_RPM) <= 100:
                self.hammer_pub.publish(Empty())
                self.mazemaze_pub.publish(Empty())
                self.auto_mode = self.AUTO_MODE_WAIT
        
        # 2秒待つ状態
        elif self.auto_mode == self.AUTO_MODE_WAIT:
            self.get_logger().info(f"WAIT_MODE:")
            self.wait_counter += 1
            if self.wait_counter >=200:
                self.auto_mode = self.AUTO_MODE_STOP

        # ローラーを減速する状態（減速終了時に例外を出して止める）
        elif self.auto_mode == self.AUTO_MODE_STOP:
            self.get_logger().info(f"STOP_MODE:")
            self.control_pub.publish(Bool(data=False))
            self.roller_pub.publish(Float64(data=0.0))
            if abs(current_rpm) <= 100:
                raise KeyboardInterrupt

        self.timer_processing = False

def main():
    rclpy.init()
    node = ShootTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
