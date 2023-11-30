#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.i = 0
        
    def car_go(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1

    def car_steer_left(self):
        for i in range(0,100):
            msg = Twist()
            msg.linear.x = 2.0
            msg.angular.z = 1.0*(100/(i+1))
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg)
        time.sleep(1)
        for i in range(0,100):
            msg = Twist()
            msg.linear.x = 2.0
            msg.angular.z = -1.0*(100/(i+1))
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg)
        time.sleep(1)
        self.i += 1
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1
        time.sleep(5)


    def car_steer_right(self):
        for i in range(0,100):
            msg = Twist()
            msg.linear.x = 2.0
            msg.angular.z = -1.0*(100/(i+1))
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg)
        time.sleep(1)
        for i in range(0,100):
            msg = Twist()
            msg.linear.x = 2.0
            msg.angular.z = 1.0*(100/(i+1))
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg)
        time.sleep(1)
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1
        time.sleep(5)

    def car_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1
        cont = input("Paused: Enter to continue...")
 

def main(args=None):
    rclpy.init(args=args) 
    minimal_publisher = MinimalPublisher()   
    rclpy.spin(minimal_publisher)   
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
