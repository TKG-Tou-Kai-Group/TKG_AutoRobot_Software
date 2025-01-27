import os
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int64, Bool
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Turret(Node):

    def __init__(self):
        super().__init__('turret')
        self.pitch_calibrated = False
        self.yaw_right_reached = False
        self.yaw_left_reached = False
        self.calibrated_data_written = False

        self.yaw_data = 0.0
        self.ratio_yaw = 2.0
        self.max_yaw = 0.0
        self.min_yaw = 0.0
        self.yaw_pub = self.create_publisher(Int64, '/can_node/gm6020_1/target_volt', 10)
        self.subscription_motor1 = self.create_subscription(Float64, '/can_node/gm6020_1/degree', self.motor1_callback, 10)

        self.pitch_data = 0.0
        self.ratio_pitch = -3.0
        self.subscription_motor0 = self.create_subscription(Float64, '/can_node/gm6020_0/degree', self.motor0_callback, 10)

        self.mesurement_pitch_diff = -95.0;
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.subscription_imu = self.create_subscription(Imu, '/camera/camera/imu', self.callback_imu, qos_profile)

        self.yaw_right = False
        self.yaw_left = False
        self.subscription_yaw_right = self.create_subscription(Bool, '/gpio_node/in0', self.yaw_right_callback, 10)
        self.subscription_yaw_left = self.create_subscription(Bool, '/gpio_node/in1', self.yaw_left_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.limit_output = 0.1

    def output_yaw(self, value):
        self.yaw_pub.publish(Int64(data=int(value)))

    def motor0_callback(self, msg):
        self.pitch_data = msg.data #self.pitch_angle_converter(math.radians(msg.data))

    def motor1_callback(self, msg):
        self.yaw_data = msg.data #self.yaw_angle_converter(math.radians(msg.data))
        if self.yaw_data > self.max_yaw:
            self.max_yaw = self.yaw_data
        if self.yaw_data < self.min_yaw:
            self.min_yaw = self.yaw_data

    def callback_imu(self, msg):
        self.mesurement_pitch_diff = math.degrees(math.atan2(msg.linear_acceleration.z, -msg.linear_acceleration.y)) * self.ratio_pitch

    def yaw_left_callback(self, msg):
        self.yaw_left = msg.data

    def yaw_right_callback(self, msg):
        self.yaw_right = msg.data

    def timer_callback(self):
        if self.pitch_data == 0.0 or self.mesurement_pitch_diff <= -90.0:
            self.get_logger().info("data waiting...")
            return
        elif not self.pitch_calibrated:
            self.get_logger().info("pitch calibrating")
            self.calibrated_pitch_data = self.pitch_data - self.mesurement_pitch_diff
            self.pitch_calibrated = True
            return
        elif not self.yaw_right_reached:
            self.get_logger().info("moving right...")
            self.output_yaw(-4000)
            if self.yaw_right:
                self.yaw_right_reached = True
            return
        elif not self.yaw_left_reached:
            self.get_logger().info("moving left...")
            self.output_yaw(4000)
            if self.yaw_left:
                self.yaw_left_reached = True
            return
        elif not self.calibrated_data_written:
            self.get_logger().info("data writing")
            self.output_yaw(0)
            calibrated_yaw_data = (self.max_yaw + self.min_yaw)/2.0
            self.calibrated_data_written = True
            self.file_path = os.path.join(os.getcwd(), 'calibrated_data.csv')
            data_line = f"{self.calibrated_pitch_data},{calibrated_yaw_data}\n"
            with open(self.file_path, 'w') as file:
                file.write(data_line)
            return
        else:
            raise KeyboardInterrupt
            return

def main():
    rclpy.init()
    node = Turret()

    executor = rclpy.executors.SingleThreadedExecutor()
    node.executor = executor
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
