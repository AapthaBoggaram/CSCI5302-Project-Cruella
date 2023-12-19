
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
import keyboard
import time

class PidController(Node):
	
	def __init__(self):
		super().__init__('reverse_pid_controller')
		# publisher
		print('Creating Pub')
		self.publisher_servo = self.create_publisher(ServoCtrlMsg, '/cmdvel_to_servo_pkg/servo_msg', 10)

		# subscriber
		print('Creating Sub')
		self.subscriber = self.create_subscription(LaserScan,'/scan', self.msg_callback,10)

		# defined variables
		self.prev_error = 0.0
		self.k_p = .45 #1.15, 1.35
		self.k_i = 0.00
		self.k_d = 0.0035 #.78
		self.dt = .01
		self.integral = 0.0
		self.angle = 0.0
		self.speed_mod = -.85 #.75
		self.back_speed_mod = .75
		self.angle_mod = -.35
		self.previous_angle = 0.0
		self.right_reading_median = None
		self.left_reading_median = None
		self.forward_reading_median = None


	def msg_callback(self, msg):        

		# emergency stpping condition
		if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
			print('oh no')
			self.car_stop()
			#stop_sign_subscriber.destroy_node()
			rclpy.shutdown()

		# number of lidar readings in a given subscription
		one_deg = len(msg.ranges)/360

		# forward reading median within a 16 degree cone
		forward_readings = []
		reading_range = np.array([177, 178, 179, 180, 181, 182, 183])
		for i in reading_range:
			idx = int(one_deg*i)
			if msg.ranges[idx] < float('inf') and not (np.isnan(msg.ranges[i])):
				forward_readings.append(msg.ranges[idx])
				
		self.forward_reading_median = np.median(forward_readings)
		print('forward reading: %f' %self.forward_reading_median)

		# getting left median reading - 60 degrees to 30 degrees left of the 0th angle 
		#print('Reading Left')
		left_readings = []
		for i in range(int(one_deg*210), int(one_deg*260)): #Prior Values: for i in range(int(one_deg*10), int(one_deg*70)):
			if msg.ranges[i] < float('inf') and not (np.isnan(msg.ranges[i])):
				left_readings.append(msg.ranges[i])
		left_readings.sort()
		for i in range(0,int(len(left_readings)/6)):
			left_readings.pop()
		self.left_reading_median = np.median(left_readings)
		print('left reading: %f' %self.left_reading_median)
		

		# getting right median reading - 60 degrees to 30 degrees right of the 0th angle
		#print('Reading Right')
		right_readings = []
		for i in range(int(one_deg*100), int(one_deg*150)): #Prior Values: for i in range(int(one_deg*290), int(one_deg*350)):
			if (msg.ranges[i] < float('inf')) and not (np.isnan(msg.ranges[i])):
				right_readings.append(msg.ranges[i])
		right_readings.sort()
		for i in range(0,int(len(right_readings)/6)):
			right_readings.pop()
		self.right_reading_median = np.median(right_readings)
		print('right reading: %f' %self.right_reading_median)


		# error calculation
		#l_error = (self.front_left_reading_median + self.left_reading_median)/2
		#r_error = (self.front_right_reading_median + self.right_reading_median)/2
		error = self.right_reading_median - self.left_reading_median
				# straight hallway scenario
		if(error >-80 and error < 80):
			# adjusting angle based on pid
			print('Straight Hallway PID')
			self.integral = self.integral + error * self.dt
			derivative = (error - self.prev_error) / self.dt
			self.angle = (-(self.k_p * error + self.k_i * self.integral + self.k_d * derivative)+self.angle_mod)#/11
			self.prev_error = error
			self.previous_angle = self.angle
			print('Error: %f' %error)
			print('Angle: %f' %self.angle)
			# updating the angle in car_go command
			self.car_go(self.angle)
		
		# car too close to the wall
		if(self.forward_reading_median < .65):
			print('U-Turn')
			# make an u-turn by backing up a little and turning it
			self.car_stop()
		
			# reverse with slight steering to the left for 0.5 seconds
			start_time = time.time()
			while(time.time() - start_time < .9):
				
				# emergency stpping condition
				if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
					print('oh no')
					self.car_stop()
					rclpy.shutdown()
		
				self.car_go_reverse(self.previous_angle) #self.car_turn_reverse_left()	

		#bad hallway sad
		elif self.right_reading_median < 1.5 and self.forward_reading_median > 1.0 and self.left_reading_median > .75:
			print('bad hallway')
			start_time = time.time()
			while time.time() - start_time > 1.5:
				self.car_go(0.0)

		#t_intersection turn case
		elif self.right_reading_median > 3.0 and self.forward_reading_median < 1.5:
			start_time = time.time()
			while(time.time()-start_time < .25):
				print('wait')
			while(time.time()-start_time < .65):
				print('hard right')
				self.car_turn_right()
			while(time.time()-start_time < 1.15):
				print('left correction')
				self.car_go(0.0)
		
	def car_turn_right(self):
		# ServoCtrlMsg publish
		print('right')
		msg = ServoCtrlMsg()
		msg.throttle = self.speed_mod	# lower speed when turning
		msg.angle = 1.0
		self.publisher_servo.publish(msg)


	def car_turn_left(self):
		# ServoCtrlMsg publish
		print('left')
		msg = ServoCtrlMsg()
		msg.throttle = self.speed_mod # lower speed when turning
		msg.angle = -1.0
		self.publisher_servo.publish(msg)

	
	def car_turn_reverse_left(self):
		print('revleft')
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.throttle = self.back_speed_mod # lower speed when turning
		msg.angle = 1.0
		self.publisher_servo.publish(msg)


	def car_turn_reverse_right(self):
		print('revright')
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.throttle = self.back_speed_mod # lower speed when turning
		msg.angle = -1.0
		self.publisher_servo.publish(msg)


	def car_go(self, angle):
		# ServoCtrlMsg publish
		print('Go')
		msg = ServoCtrlMsg()
		msg.angle = -angle
		msg.throttle = self.speed_mod
		self.publisher_servo.publish(msg)

	def car_go_reverse(self, angle):
		print('go reverse')
		msg = ServoCtrlMsg()
		msg.throttle = self.back_speed_mod
		msg.angle = angle
		self.publisher_servo.publish(msg)

	def car_stop(self):
		# ServoCtrlMsg publish
		print('Stop')
		msg = ServoCtrlMsg()
		msg.angle = 0.0
		msg.throttle = 0.0
		self.publisher_servo.publish(msg)


def main(args=None):

	rclpy.init(args=args) 

	reverse_pid_controller = PidController()   
	
	rclpy.spin(reverse_pid_controller)

	reverse_pid_controller.destroy_node()

	rclpy.shutdown()


if __name__ == '__main__':
	main()
