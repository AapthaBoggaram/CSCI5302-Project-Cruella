#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from std_msgs.msg import String
import pygame
import numpy as np



class ControllerNode(Node):

	def __init__(self):
		super().__init__('controller_node')
		self.publisher_servo = self.create_publisher(ServoCtrlMsg, '/cmdvel_to_servo_pkg/servo_msg', 10)
		self.i = 0
		self.speed_mod = -.75
		self.ang_mod = -1
		self.timer = self.create_timer(0.1, self.publish_ctrl_msg)
		pygame.init()
		pygame.joystick.init()
		
		if pygame.joystick.get_count() == 0:
				#self.get_logger.error("No joystick found")
				print('No joystick :(')
				self.car_stop()
				rclpy.shutdown

		self.joystick1 = pygame.joystick.Joystick(0)
		#self.joystick2 = pygame.joystick.Joystick(1)
		self.joystick1.init()
		#self.joystick2.init()
		print(pygame.joystick.get_count())

	def publish_ctrl_msg(self):
		pygame.event.pump()
		throttle = self.joystick1.get_axis(1)
		angle = self.joystick1.get_axis(3)
		msg = ServoCtrlMsg()
		print('Throttle: %f' %throttle)
		print('Angle: %f' %angle)
		#if np.abs(throttle) > 0.02:
		msg.throttle = self.speed_mod*throttle
		msg.angle = self.ang_mod*angle
		#else:
		#	msg.throttle = 0.0
		#	msg.angle = -1.2*angle

		self.publisher_servo.publish(msg)

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

def main(args=None):
	rclpy.init(args=args) 
	minimal_publisher = ControllerNode()   
	rclpy.spin(minimal_publisher)   
	self.car_stop()
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

