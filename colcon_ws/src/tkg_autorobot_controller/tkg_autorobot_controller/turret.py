import os
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int64, Bool
from geometry_msgs.msg import Vector3

class Turret(Node):

    def __init__(self):
        super().__init__('turret')
        file_path = os.path.join(os.getcwd(), 'calibrated_data.csv')
        with open(file_path, 'r') as f:
            line = f.readline()
            data = line.split(',')
            if len(data) >= 2:
                self.offset_pitch = math.radians(float(data[0]))
                if self.offset_pitch < math.radians(120-30) or self.offset_pitch > math.radians(120+30):
                    self.offset_pitch = math.radians(120)
                self.offset_yaw = math.radians(float(data[1]))
                if self.offset_yaw < math.radians(175-20) or self.offset_yaw > math.radians(175+20):
                    self.offset_yaw = math.radians(175)
            else:
                raise FileNotFoundError


        self.control = False
        self.control_sub = self.create_subscription(Bool, '/control', self.control_callback, 10)
        
        self.yaw_data = 0.0
        self.ratio_yaw = 2.0
        self.yaw_angle_converter = AngleConverter(self.ratio_yaw, self.offset_yaw)
        self.pid_yaw = PID(10000.0, 0.0, 1000.0)
        self.pid_yaw.target(0)
        self.yaw_pub = self.create_publisher(Int64, '/can_node/gm6020_1/target_volt', 10)
        self.subscription_yaw = self.create_subscription(Float64, '/yaw', self.callback_yaw, 10)
        self.subscription_motor1 = self.create_subscription(Float64, '/can_node/gm6020_1/degree', self.motor1_callback, 10)

        self.pitch_data = 0.0
        self.ratio_pitch = -3.0
        self.pitch_angle_converter = AngleConverter(self.ratio_pitch, self.offset_pitch)
        self.pid_pitch = PID(-20000.0, 0.0, -1000.0)
        self.pid_pitch.target(0)
        self.pitch_pub = self.create_publisher(Int64, '/can_node/gm6020_0/target_volt', 10)
        self.subscription_pitch = self.create_subscription(Float64, '/pitch',self.callback_pitch, 10)
        self.subscription_motor0 = self.create_subscription(Float64, '/can_node/gm6020_0/degree', self.motor0_callback, 10)
        self.yaw_right = False
        self.yaw_left = False
        self.pitch_down = False
        self.pitch_up = False
        self.subscription_yaw_right = self.create_subscription(Bool, '/gpio_node/in0', self.yaw_right_callback, 10)
        self.subscription_yaw_left = self.create_subscription(Bool, '/gpio_node/in1', self.yaw_left_callback, 10)
        self.subscription_pitch_down = self.create_subscription(Bool, '/gpio_node/in2', self.pitch_down_callback, 10)
        self.subscription_pitch_up = self.create_subscription(Bool, '/gpio_node/in3', self.pitch_up_callback, 10)

        self.turret_pose = self.create_publisher(Vector3, '/current_turret_pose', 10)

        self.stamp = self.get_clock().now()
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.limit_output = 0.1

    def output_pitch(self, value):
        self.pitch_pub.publish(Int64(data=int(value)))

    def output_yaw(self, value):
        self.yaw_pub.publish(Int64(data=int(value)))

    def motor0_callback(self, msg):
        self.pitch_data = self.pitch_angle_converter(math.radians(msg.data))

    def motor1_callback(self, msg):
        self.yaw_data = self.yaw_angle_converter(math.radians(msg.data))
        
    def callback_yaw(self, msg):
        self.pid_yaw.target(max([math.radians(-80), min([msg.data, math.radians(80)])]))

    def callback_pitch(self, msg):
        self.pid_pitch.target(max([math.radians(-8), min([msg.data, math.radians(28)])]))

    def control_callback(self, msg):
        self.control = msg.data

    def pitch_up_callback(self, msg):
        self.pitch_up = msg.data

    def pitch_down_callback(self, msg):
        self.pitch_down = msg.data

    def yaw_left_callback(self, msg):
        self.yaw_left = msg.data

    def yaw_right_callback(self, msg):
        self.yaw_right = msg.data

    def timer_callback(self):
        yaw_output = self.pid_yaw.update(self.yaw_data)
        pitch_output = self.pid_pitch.update(self.pitch_data)
        
        cycle_check_ok = ((self.get_clock().now()-self.stamp).nanoseconds <= 20000000)
        if not cycle_check_ok:
            print('cyclecheck failed')
        self.stamp = self.get_clock().now()

        abs_limit = lambda value, limit: math.copysign(min([abs(value), abs(limit)]), value)        
        yaw_output = abs_limit(yaw_output, self.limit_output) if (self.yaw_left or self.yaw_right) else yaw_output
        pitch_output = abs_limit(pitch_output, self.limit_output) if (self.pitch_up or self.pitch_down) else pitch_output

        if not (self.control and cycle_check_ok):
            yaw_output = 0
            pitch_output = 0

        self.output_yaw(yaw_output)
        self.output_pitch(pitch_output)

        pitch = self.pitch_data
        yaw = self.yaw_data
        self.turret_pose.publish(Vector3(x=0.0, y=pitch, z=yaw))


class AngleConverter:
    def __init__(self, ratio,offset):
        self._ratio = ratio
        self._offset = offset

    def angle_normalize(self, value):
        return math.atan2(math.sin(value),math.cos(value))

    def __call__(self, value):
        return self.angle_normalize(value-self._offset)/self._ratio


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._target = 0
        self._output = 0
        self.value_i = 0
        self._last_diff = 0

    def target(self, value):
        self._target = value
    
    def update(self, value):
        diff = self._target-value

        value_p = diff*self.kp
        self.value_i += diff*self.ki
        value_d = (diff-self._last_diff)*self.kd

        self._output = value_p + self.value_i + value_d

        self._last_diff = diff
        return self._output

    def output(self):
        return self._output


def main():
    rclpy.init()
    node = Turret()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, FileNotFoundError):
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
