import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import time

class FollowHuman(Node):

    def __init__(self):
        super().__init__('follow_human')
        self.subscription = self.create_subscription(
            Point,
            '/person_point',
            self.listener_callback,
            10)
        self.hand_subscription = self.create_subscription(
            Point,
            '/hand_topic',
            self.hand_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.002)  # 0.005 for 1280x720 frame or x2 for /2frame
        self.declare_parameter("forward_chase_speed", 0.27)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("min_dist_thresh", 1.3)
        self.declare_parameter("filter_value", 0.8)

        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.min_dist_thresh = self.get_parameter('min_dist_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value

        timer_period = 0.07  # seconds 0.09 is 10% faster than 0.1s
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000
        self.frame_center_x = 640  # 640 x 480 = 320 (for 1280x720 = 640)
        self.center_threshold_high = 750  # (370 270 for 640x480 & 750 530 for 1280x720)
        self.center_threshold_low = 530
        self.last_gesture = None  # closed hand gesture id = 1 (allowed to navigate)

    # def timer_callback(self):
    #     msg = Twist()
    #     if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
    #         self.get_logger().info('Target: {}'.format(self.target_val))
    #         self.get_logger().info('Distance: {}'.format(self.target_dist))
    #         print(self.target_dist)
    #         if (self.target_dist > self.min_dist_thresh):
    #             msg.linear.x = self.forward_chase_speed
    #             msg.angular.z = -self.angular_chase_multiplier*self.target_val
    #         else:
    #             self.get_logger().info('Target lost')
    #             msg.angular.z = self.search_angular_speed
    #         self.publisher_.publish(msg)

    def timer_callback(self):
        msg = Twist()
        if self.last_gesture != 0:
            if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
                # self.get_logger().info('Target: {}'.format(self.target_val))
                # self.get_logger().info('Distance: {}'.format(self.target_dist))
                # print(self.target_dist)
                offset = self.target_val - 640  # Dist from center of the frame 640 x 480 = 320 (for 1280x720 = 640)
                normalized_offset = (offset / abs(offset))
                if (self.target_dist > self.min_dist_thresh):
                    msg.linear.x = self.forward_chase_speed
                    if ((self.target_val > 750) or (self.target_val < 530)):  # Threshold to be considered centered (370 270 for 640x480 & 750 530 for 1280x720)
                        msg.angular.z = self.angular_chase_multiplier * (-normalized_offset) * abs(offset) * 0.85
                        # Give a normal ang vel val ｜ -1=R_TURN.1=L_TURN ｜ angular speed proportional to how far away it is from the center
                else:
                    self.get_logger().info('Target lost')
                    msg.angular.z = self.search_angular_speed
            else:
                # stop the robot
                msg.linear.x = 0.0
                msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        f = self.filter_value
        # print(msg.x)
        self.target_val = self.target_val * f + msg.x * (1 - f)
        self.target_dist = self.target_dist * f + msg.z * (1 - f)
        self.lastrcvtime = time.time()
        # self.get_logger().info('Received: {} {}'.format(msg.x, msg.y))

    def hand_callback(self, msg):
        self.last_gesture = msg.data  # Update the last seen gesture
        # self.get_logger().info(f'Received hand gesture: {msg}')

def main(args=None):
    rclpy.init(args=args)
    follow_human = FollowHuman()
    rclpy.spin(follow_human)
    follow_human.destroy_node()
    rclpy.shutdown()
