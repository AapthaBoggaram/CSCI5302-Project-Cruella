import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import keyboard
import time

class PidController(Node):
	
	def __init__(self):
		super().__init__('pid_controller')
		# publisher
		self.publisher_servo = self.create_publisher(ServoCtrlMsg, '/cmdvel_to_servo_pkg/servo_msg', 10)

		# subscriber
		self.subscriber = self.create_subscription(LaserScan,'/scan', self.msg_callback,10)

		# defined variables
		self.prev_error = 0.0
		self.k_p = 1.25
		self.k_i = 0.000
		self.k_d = 0.0015
		self.dt = .01
		self.integral = 0.0
		self.angle = 0.0
		self.speed_mod = .65


	def msg_callback(self, msg):        

		# emergency stpping condition
		if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
			print('oh no')
			self.car_stop()
			#stop_sign_subscriber.destroy_node()
			rclpy.shutdown()

		# number of lidar readings in a given subscription
		num_of_degrees = len(msg.ranges)

		# forward reading median within a 15 degree cone
		forward_readings = []
		for i in range(int(num_of_degrees * 47/48), int(num_of_degrees * 1/48)):
			forward_readings.append(msg.ranges[i])
		forward_reading_median = np.median(forward_readings)

		# getting left median reading - 60 degrees to 30 degrees left of the 0th angle 
		left_readings = []
		for i in range(num_of_degrees-30, num_of_degrees-90):
			left_readings.append(msg.ranges[i])
		left_reading_median = np.median(left_readings)

		# getting right median reading - 60 degrees to 30 degrees right of the 0th angle
		right_readings = []
		for i in range(30, 90):
			right_readings.append(msg.ranges[i])
		right_reading_median = np.median(right_readings)

		# error calculation
		error = right_reading_median - left_reading_median
		
		# car too close to the wall
		if(forward_reading_median < 0.5):
			# make an u-turn by backing up a little and turning it
			self.car_stop()

			# reverse with slight steering to the left for 0.5 seconds
			start_time = time.time()
			while(time.time() - start_time < 1):
				
				# emergency stpping condition
				if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
					print('oh no')
					self.car_stop()
					rclpy.shutdown()

				self.car_turn_reverse_left()	

			# turn right for 0.5 seconds OR until the forward reading overshoots (but I removed that part of the code)
			start_time = time.time()
			while(time.time() - start_time < 0.5):

				# emergency stpping condition
				if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
					print('oh no')
					self.car_stop()
					rclpy.shutdown()
				
				self.car_turn_right()

		# straight hallway scenario
		elif(error > -80 and error < 80):
			# adjusting angle based on pid
			self.integral = self.integral + error
			derivative = (error - self.prev_error)
			self.angle = (self.k_p * error + self.k_i * self.integral + self.k_d * derivative)
			self.prev_error = error

			# updating the angle in car_go command
			self.car_go()

		
	def car_turn_right(self):
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.throttle = self.speed_mod * .75	# lower speed when turning
		msg.angle = 45.0
		self.publisher_servo.publish(msg)


	def car_turn_left(self):
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.throttle = self.speed_mod * .75	# lower speed when turning
		msg.angle = -45.0
		self.publisher_servo.publish(msg)

	
	def car_turn_reverse_left(self):
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.throttle = -self.speed_mod * .75	# lower speed when turning
		msg.angle = -45.0
		self.publisher_servo.publish(msg)


	def car_turn_reverse_right(self):
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.throttle = -self.speed_mod * .75	# lower speed when turning
		msg.angle = 45.0
		self.publisher_servo.publish(msg)


	def car_go(self):
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.angle = self.angle
		msg.throttle = self.speed_mod
		self.publisher_servo.publish(msg)


	def car_stop(self):
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.angle = 0.0
		msg.throttle = 0.0
		self.publisher_servo.publish(msg)


def main(args=None):

	rclpy.init(args=args) 

	pid_controller = PidController()   
	
	rclpy.spin(pid_controller)   

	pid_controller.destroy_node()

	rclpy.shutdown()


if __name__ == '__main__':
	main()

		# diagonal readings at 45 degrees from left and right
		# right_diagonal_reading = msg.ranges[int(num_of_degrees * 1/8)]
		# left_diagonal_reading = msg.ranges[int(num_of_degrees * 7/8)]

		# # right turn scenario
		# if(right_diagonal_reading > 2.75):
		# 	# keep going straight until the car is in the turning
		# 	while(forward_reading >= 1.5):
		# 		self.car_go()
		# 	# turn right until forward reading overshoots
		# 	while(forward_reading < 80 and left_diagonal_reading >= 2.8 and right_diagonal_reading >= 2.8):
		# 		self.car_turn_right()
		# 	# after turning, make car's angle 0 again.
		# 	self.car_go()

		# # left turn scenario
		# elif(left_diagonal_reading > 1.5):
		# 	# keep going straight until the car is in the turning
		# 	while(forward_reading <= 0.5):
		# 		self.car_go()
		# 	# turn right until forward reading overshoots
		# 	while(forward_reading < 80 and left_diagonal_reading <= 1.5 and right_diagonal_reading <= 1.5):
		# 		self.car_turn_left()
		# 	# after turning, make car's angle 0 again.
		# 	self.car_go()

		# # left and right readings at 90 degrees
		# left_reading = msg.ranges[int(num_of_degrees / 4)]
		# right_reading = msg.ranges[int(num_of_degrees * 3/4)]
