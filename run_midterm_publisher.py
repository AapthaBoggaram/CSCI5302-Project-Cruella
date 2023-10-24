#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_ = self.create_publisher(Twist, '/cmdvel_to_servo_pkg/servo_msg', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        #self.subscription = self.create_subscription(
        #    LaserScan,
        #    '/scan',
        #    self.listener_callback,
        #    10)
        #self.subscription  # prevent unused variable warning

    #def listener_callback(self, msg):
        #\ msg = LaserScan()
        #self.get_logger().info('I heard 180: "%s", "%s", "%s", "%s"' % (msg.ranges[0], msg.ranges[360], msg.ranges[540], msg.ranges[716]))
        # self.get_logger().info('I heard 360 : "%s"' % msg.ranges[360])
        # self.get_logger().info('I heard 540: "%s"' % msg.ranges[540])
        # self.get_logger().info('I heard 719: "%s"' % msg.ranges[719])
        #if(msg.data[some_index]<0.5):
        #    msg_new = Twist()
            

    def timer_callback(self):
        msg = Twist()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    #minimal_subscriber = MinimalSubscriber()
    minimal_publisher = MinimalPublisher()
    #rclpy.spin(minimal_subscriber)
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    #minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

