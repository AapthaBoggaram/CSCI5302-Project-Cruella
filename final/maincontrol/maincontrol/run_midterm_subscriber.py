#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from midterm.run_midterm_publisher import MinimalPublisher
import keyboard as keyboard


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0
        self.subscription = self.create_subscription(LaserScan,'/scan',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.carpub = MinimalPublisher()

    #def car_go(self):
     #   msg = Twist()
      #  msg.linear.x = 2
       # msg.angular.z = 0
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1

    #def car_stop(self):
     #   msg = Twist()
      #  msg.linear.x = 0
       # msg.angular.z = 0
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1
        #cont = input("Paused: Enter to continue...")

    def listener_callback(self, msg):
        # msg = LaserScan()
        try:
            print(msg.ranges[0], msg.ranges[90],msg.ranges[180],msg.ranges[270])
        except:
            print("MSG FAILED!")
        if(float(msg.ranges[0])<1.0):
            print("CAR STOP")
            self.carpub.car_stop()
            cont = input("Paused: Enter to continue...")

        else:
            print("CAR GO")
            self.carpub.car_go()
        if keyboard.is_pressed('t'):
            self.carpub.car_stop()
            self.carpub.destroy_node()
            minimal_subscriber.destroy_node()
            rclpy.shutdown()
def main(args=None):
    cont = input("Paused: Enter to begin...")
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

