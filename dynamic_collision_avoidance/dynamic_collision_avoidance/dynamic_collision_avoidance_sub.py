#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from dynamic_collision_avoidance.dynamic_collision_avoidance_pub import MinimalPublisher
import keyboard as keyboard

global distance_values

class MinimalSubscriber(Node):

    def __init__(self):
        global distance_values
        super().__init__('minimal_subscriber')
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0
        self.subscription = self.create_subscription(LaserScan,'/scan',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.carpub = MinimalPublisher()

    
    def listener_callback(self, msg):
        # msg = LaserScan()
        global distance_values
        threshold = 1.3

        if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
            self.carpub.car_stop()
            self.carpub.destroy_node()
            rclpy.shutdown()
        '''try:
            self.get_logger().info('HERE!!!!')
            #print(msg.ranges[60])
            #print(msg.ranges[470])
        except:
            print("MSG FAILED!")
        if(float(msg.ranges[0])<1.0):
            print("CAR STOP")
            self.carpub.car_stop()
            cont = input("Paused: Enter to continue...")'''
        length_lidar_indices = len(msg.ranges)
        for i in range(0,int(length_lidar_indices/8)):
            #distance_values[i] = msg.ranges[i]
            if(msg.ranges[i] < threshold):
                print(i)
                self.carpub.car_steer_right()
        for i in range(int(length_lidar_indices*7/8),length_lidar_indices):
            #distance_values[i] = msg.ranges[i]
            try:
                if(msg.ranges[i] < threshold):
                    print(i)
                    self.carpub.car_steer_left()
            except:
                continue
        print("CAR GO")
        self.carpub.car_go()


def main(args=None):
    global distance_values
    distance_values = [0]*540
    cont = input("Paused: Enter to begin...")
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
