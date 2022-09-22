import rclpy
from rclpy.node import Node
import time
from tf2_ros.transform_broadcaster import TransformBroadcaster

# message
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# jetson
import RPi.GPIO as GPIO
import serial

# odometry関連
from odom_simulator_func import OdomSimlatorFunc


class NedoMobileMover(Node):

    def __init__(self):
        super().__init__('NedoMobileMover')

        # 設定
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('thorvo_pwm_pin', 15)
        self.declare_parameter('steering_pwm_pin', 18)
        self.declare_parameter('widen_pwm_pin', 13)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        thorvo_pwm_pin = self.get_parameter('thorvo_pwm_pin').get_parameter_value().integer_value
        steering_pwm_pin = self.get_parameter('steering_pwm_pin').get_parameter_value().integer_value
        widen_pwm_pin = self.get_parameter('widen_pwm_pin').get_parameter_value().integer_value
        self.MM_tread = 0.525
        
        # PWM設定
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(thorvo_pwm_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(steering_pwm_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(widen_pwm_pin, GPIO.OUT, initial=GPIO.HIGH)
        self.thorvo = GPIO.PWM(thorvo_pwm_pin, 50)
        self.steering = GPIO.PWM(steering_pwm_pin, 50)
        self.widen = GPIO.PWM(widen_pwm_pin, 50)

        # Serial設定
        self.baudrate = 9600
        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)

        # thread 起動

        self.CmdvelSub = self.create_subscription(Twist, '/cmd_vel', self.sub_cmdvel_callback, 10)
        self.time = time.time()

        # Odometryの設定
        self.OdometryPub_ = self.create_publisher(Odometry, '/odom', 10)
        self._tf_Odompublisher = TransformBroadcaster(self)
        self.odomSimFunc = OdomSimlatorFunc()

        # serialの読み込み
        while True:
            r_fb_flg, r_pls, l_fb_flg, l_pls = self.ser.readline().split("/")

            # 変更の必要あり
            right_wheel_speed = 1
            right_direction = -1
            left_wheel_speed = 1
            left_direction = 1

            left_speed = right_wheel_speed * right_direction
            right_speed = left_wheel_speed * left_direction

            deltat = time.time() - self.time
            self.time = time.time()

            Vf = (left_speed + right_speed) / 2.0
            Wz = (right_speed - left_speed) / self.MM_tread

            # publish odom data
            odom, tf_odom = self.odomSimFunc.update_odom(Vf, Wz, deltat)
            # calc cmdvel
            odom.header.stamp = self.get_clock().now().to_msg()
            self.OdometryPub_.publish(odom)
            tf_odom.header.stamp = self.get_clock().now().to_msg()
            self._tf_Odompublisher.sendTransform(tf_odom)

    def __del__(self):
        # PWM設定の開放
        self.thorvo.stop()
        self.steering.stop()
        self.widen.stop()
        GPIO.cleanup()

    #     # serialの開放
        self.ser.close()

    def sub_cmdvel_callback(self, msg):

        # thorvo設定
        # -1<msg.linear.x<1で想定
        self.thorvo.start(int(msg.linear.x * 100))

        # steering設定
        # -1<msg.angular.z<1で想定
        self.steering.start(int(msg.angular.z * 100))
        pass


def main(args=None):
    rclpy.init(args=args)

    nedo_mobile_mover = NedoMobileMover()

    rclpy.spin(nedo_mobile_mover)
    nedo_mobile_mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()