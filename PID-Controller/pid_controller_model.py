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

		# self.subscriber_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

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

	# def odom_callback(self, msg):
		# print("odom msg twist:", msg.twist)

	def msg_callback(self, msg):        

		# emergency stpping condition
		if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
			print('oh no')
			self.car_stop()
			#stop_sign_subscriber.destroy_node()
			rclpy.shutdown()

		# number of lidar readings in a given subscription
		num_of_degrees = len(msg.ranges)

		# left and right readings
		left_reading_old = msg.ranges[int(num_of_degrees / 4)]
		right_reading_old = msg.ranges[int(num_of_degrees * 3/4)]

		# getting left min
		left_reading = float('inf')
		for i in range(int(num_of_degrees / 2)):
			if(msg.ranges[i] < left_reading):
				left_reading = msg.ranges[i]

		# getting right min
		right_reading = float('inf')
		for i in range(int(num_of_degrees / 2), num_of_degrees):
			if(msg.ranges[i] < right_reading):
				right_reading = msg.ranges[i]

		# print("left_reading_old: ", left_reading_old, " left_reading: ", left_reading, " right_reading_old: ", right_reading_old, " right_reading: ", right_reading)

		# adjusting angle based on pid (?)
		error = right_reading - left_reading
		print(error)
		if(error > -80 and error < 80):
			self.integral = self.integral + error # * self.dt
			derivative = (error - self.prev_error) # / self.dt
			self.angle = - (self.k_p * error + self.k_i * self.integral + self.k_d * derivative)
			# print(error, self.angle)
			self.prev_error = error
			self.iter+=1
			self.time_vals.append(time.time)
			self.steer_vals.append(float(self.angle))
			
			# updating the angle in car_go command
			self.car_go()

		# if self.iter == 50:
		#	self.car_stop()
		# 	plt.title("Steering Angle") 
		# 	plt.xlabel("time") 
		# 	plt.ylabel("angle") 
		# 	plt.plot(self.steer_vals) 
		# 	plt.show()
		

	def car_go(self):
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.angle = self.angle
		msg.throttle = self.speed_mod
		self.publisher_servo.publish(msg)

		# self.get_logger().info('Publishing: "%s"' % msg)

	def car_stop(self):
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.angle = 0.0
		msg.throttle = 0.0
		self.publisher_servo.publish(msg)

		# self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):

	rclpy.init(args=args) 

	pid_controller = PidController()   
	
	rclpy.spin(pid_controller)   

	pid_controller.destroy_node()

	rclpy.shutdown()


if __name__ == '__main__':
	main()
