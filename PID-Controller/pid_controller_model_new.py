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
		# cmd_vel publisher
		self.publisher_cmdvel = self.create_publisher(Twist, '/cmd_vel', 10)

		# ServoCtrlMsg publisher
		self.publisher_servo = self.create_publisher(ServoCtrlMsg, '/cmdvel_to_servo_pkg/servo_msg', 10)

		self.subscriber = self.create_subscription(LaserScan,'/scan', self.msg_callback,10)

		# defined variables
		self.ranges = 0.0
		self.prev_error = 0.0
		self.k_p = 1.25
		self.k_i = 0.000
		self.k_d = 0.0015
		self.dt = .01
		self.integral = 0.0
		self.angle = 0.0
		self.iter = 0
		self.steer_vals = []
		self.time_vals = []
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

		# left and right readings at 90 degrees
		left_reading = msg.ranges[int(num_of_degrees / 4)]
		right_reading = msg.ranges[int(num_of_degrees * 3/4)]

		# forward reading at 0 degrees
		forward_reading = msg.ranges[0]

		# diagonal readings at 45 degrees from left and right
		right_diagonal_reading = msg.ranges[int(num_of_degrees * 1/8)]
		left_diagonal_reading = msg.ranges[int(num_of_degrees * 7/8)]

		# getting left median reading - 5/8 to 7/8
		left_readings = []
		for i in range(int(num_of_degrees * 5/8), int(num_of_degrees * 7/8)):
			left_readings.append(msg.ranges[i])
		left_reading_median = np.median(left_reading)

		# getting right median reading - 1/8 to 3/8
		right_readings = []
		for i in range(int(num_of_degrees / 8), int(num_of_degrees * 3/8)):
			right_readings.append(msg.ranges[i])
		right_reading_median = np.median(right_reading)

		# print("left_reading: ", left_reading, " left_reading_median: ", left_reading_median, " right_reading: ", right_reading, " right_reading_median: ", right_reading_median)

		error = right_reading_median - left_reading_median
		# print(error)
		
		# right turn scenario
		if(right_diagonal_reading > 2.75):
			# keep going straight until the car is in the turning
			while(forward_reading >= 1.5):
				self.car_go()
			# turn right until forward reading overshoots
			while(forward_reading < 80 and left_diagonal_reading <= 2.8 and right_diagonal_reading <= 2.8):
				self.car_turn_right()
			# after turning, make car's angle 0 again.
			self.car_go()

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
			
		# straight scenario
		elif(error > -80 and error < 80):
			# adjusting angle based on pid
			self.integral = self.integral + error # * self.dt
			derivative = (error - self.prev_error) # / self.dt
			self.angle = (self.k_p * error + self.k_i * self.integral + self.k_d * derivative)
			self.prev_error = error
			self.iter+=1
			self.time_vals.append(time.time)
			self.steer_vals.append(float(self.angle))
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
