#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
import time
from std_msgs.msg import String
import keyboard


class MinimalPublisher(Node):

	def __init__(self):
		super().__init__('minimal_publisher')
		self.publisher_servo = self.create_publisher(ServoCtrlMsg, '/cmdvel_to_servo_pkg/servo_msg', 10)
		self.i = 0
		self.speed_mod = .85
	
	def car_go(self):
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.angle = 0.35
		msg.throttle = self.speed_mod
		self.publisher_servo.publish(msg)
		self.get_logger().info('Publishing: "%s"' % msg)
		self.i += 1

		# self.get_logger().info('Publishing: "%s"' % msg)

	def car_stop(self):
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.angle = 0.0
		msg.throttle = 0.0
		self.publisher_servo.publish(msg)
		self.get_logger().info('Publishing: "%s"' % msg)
		self.i += 1
		cont = input("Paused: Enter to continue...")

		# self.get_logger().info('Publishing: "%s"' % msg)

	def car_steer_left(self):
		self.get_logger().info('I AM INSIDE STEER LEFT!!!!')
		if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
			print('oh no')
			self.car_stop()
			#stop_sign_subscriber.destroy_node()
			rclpy.shutdown()

		for i in range(0,100):

			if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
				print('oh no')
				self.car_stop()
				#stop_sign_subscriber.destroy_node()
				rclpy.shutdown()

			msg = ServoCtrlMsg()
			msg.throttle = self.speed_mod * .9
			msg.angle = 1.0*(100/(i+1))
			self.publisher_servo.publish(msg)
			self.get_logger().info('Publishing: "%s"' % msg)
		time.sleep(1)

		for i in range(0,100):

			if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
				print('oh no')
				self.car_stop()
				#stop_sign_subscriber.destroy_node()
				rclpy.shutdown()

			msg = ServoCtrlMsg()
			msg.throttle = self.speed_mod * .9
			msg.angle = -1.0*(100/(i+1))
			self.publisher_servo.publish(msg)
			self.get_logger().info('Publishing: "%s"' % msg)
		time.sleep(1)
		self.i += 1
		msg = ServoCtrlMsg()
		msg.throttle = self.speed_mod * .75
		msg.angle = 0.0
		self.publisher_servo.publish(msg)
		self.get_logger().info('Publishing: "%s"' % msg)
		self.i += 1
		#time.sleep(5)


	def car_steer_right(self):
		self.get_logger().info('I AM INSIDE STEER RIGHT!!!!')
		if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
			print('oh no')
			self.car_stop()
			#stop_sign_subscriber.destroy_node()
			rclpy.shutdown()

		for i in range(0,100):
			
			if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
				print('oh no')
				self.car_stop()
				#stop_sign_subscriber.destroy_node()
				rclpy.shutdown()
			
			msg = ServoCtrlMsg()
			msg.throttle = self.speed_mod * .9
			msg.angle = -1.0*(100/(i+1))
			self.publisher_servo.publish(msg)
			self.get_logger().info('Publishing: "%s"' % msg)
		time.sleep(1)
		for i in range(0,100):

			if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
				print('oh no')
				self.car_stop()
				#stop_sign_subscriber.destroy_node()
				rclpy.shutdown()

			msg = ServoCtrlMsg()
			msg.throttle = self.speed_mod * .9
			msg.angle = 1.0*(100/(i+1))
			self.publisher_servo.publish(msg)
			self.get_logger().info('Publishing: "%s"' % msg)
		time.sleep(1)
		msg = ServoCtrlMsg()
		msg.throttle = self.speed_mod * .75
		msg.angle = 0.0
		self.publisher_servo.publish(msg)
		self.get_logger().info('Publishing: "%s"' % msg)
		self.i += 1
		# time.sleep(5)
 

def main(args=None):
	rclpy.init(args=args) 
	minimal_publisher = MinimalPublisher()   
	rclpy.spin(minimal_publisher)   
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
