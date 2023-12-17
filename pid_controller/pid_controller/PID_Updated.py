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
		super().__init__('pid_controller')
		# publisher
		print('Creating Pub')
		self.publisher_servo = self.create_publisher(ServoCtrlMsg, '/cmdvel_to_servo_pkg/servo_msg', 10)
		# subscriber
		print('Creating Sub')
		self.subscriber = self.create_subscription(LaserScan,'/scan', self.msg_callback,10)

		# defined variables
		self.prev_error = 0.0
		self.k_p = 1.0
		self.k_i = 0.1
		self.k_d = 0.5
		self.dt = .01
		self.integral = 0.0
		self.angle = 0.0
		self.speed_mod = .75
		self.back_speed_mod = -.85
		self.angle_mod = -.35
		self.right_reading_median = 0.0
		self.left_reading_median = 0.0
		self.forward_reading_median = 0.0
		self.err_min_max = 20

	def update(self, error, dt):
		derivative = (error - self.prev_error) / dt
		self.integral += error * dt
		output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
		self.prev_error = error
		return output

	def msg_callback(self, msg):
		# emergency stopping condition
		if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
			print('oh no')
			self.car_stop()

		# number of lidar readings per degree
		one_deg = len(msg.ranges)/360

		# forward reading median within a -8 to 8 degree cone, Counter-Clockwise
		forward_readings = []
		reading_range = np.array([352, 353, 354, 355, 356, 357, 358, 359, 0, 1, 2, 3, 4, 5, 6, 7])
		for i in reading_range:
			idx = int(one_deg*i)
			if msg.ranges[idx] < float('inf'):
				forward_readings.append(msg.ranges[idx])
		self.forward_reading_median = np.median(forward_readings)
		print('Forward reading median: %f' %self.forward_reading_median)

		# left median reading within 15 to 75 degree cone, Counter-Clockwise
		left_readings = []
		for i in range(int(one_deg*15), int(one_deg*75)): #Prior Values: for i in range(int(one_deg*10), int(one_deg*70)):
			if msg.ranges[i] < float('inf'):
				left_readings.append(msg.ranges[i])
		left_readings.sort()
		for i in range(0,int(len(left_readings)/10)):
			left_readings.pop()
		self.left_reading_median = np.median(left_readings)
		print('Left reading median: %f' %self.left_reading_median)

		# right median reading within 285 to 345 degree cone, Counter-Clockwise
		right_readings = []
		for i in range(int(one_deg*285), int(one_deg*345)):
			if msg.ranges[i] < float('inf'):
				right_readings.append(msg.ranges[i])
		right_readings.sort()
		for i in range(0,int(len(right_readings)/10)):
			right_readings.pop()
		self.right_reading_median = np.median(right_readings)
		print('Right reading median: %f' %self.right_reading_median)

		# error calculation
		error = self.right_reading_median - self.left_reading_median

		# car too close to the wall
		if self.forward_reading_median < 2.0:
			# Stop and move backward when too close to the wall
			print('Go Reverse')
			self.car_stop()
			start_time = time.time()
			while (time.time() - start_time) < 2.0:
				self.car_go_reverse()

		# Right turn probably
		# elif  self.right_reading_median > 5 and self.left_reading_median > 5:
		# 	start_time = time.time()
		# 	while(time.time() - start_time) < .5:
		# 		self.car_stop()
		# 	start_time = time.time()
		# 	while(time.time() - start_time) < 2.25:
		# 		self.car_go()
		# 	start_time = time.time()
		# 	while(time.time() - start_time) < 1.35:
		# 		self.car_turn_right()
		# 	print('right turn')

		# T intersection turn case
		elif self.right_reading_median > 3.5 and self.left_reading_median > 3.5:
			self.car_turn_right()

		# straight hallway scenario
		elif (error > -self.err_min_max) and (error < self.err_min_max):
			# adjusting angle based on pid
			print('Straight Hallway PID')
			print('Error: %f' % error)
			self.angle = -self.Update(error, 0.1)
			print('Angle: %f' %self.angle)
			self.car_go()


	def car_turn_right(self):
		# ServoCtrlMsg publish
		print('right')
		msg = ServoCtrlMsg()
		msg.throttle = self.speed_mod*1.25	# increase lower speed when turning
		msg.angle = -1.0
		self.publisher_servo.publish(msg)

	def car_turn_left(self):
		# ServoCtrlMsg publish
		print('left')
		msg = ServoCtrlMsg()
		msg.throttle = self.speed_mod*1.25 # increase speed when turning
		msg.angle = 1.0
		self.publisher_servo.publish(msg)

	def car_go_reverse(self):
		# ServoCtrlMsg publish
		print('Go Reverse')
		msg = ServoCtrlMsg()
		msg.angle = -self.angle
		msg.throttle = self.back_speed_mod
		self.publisher_servo.publish(msg)

	def car_go(self):
		# ServoCtrlMsg publish
		print('Go')
		msg = ServoCtrlMsg()
		msg.angle = self.angle
		msg.throttle = self.speed_mod
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
	pid_controller = PidController()
	rclpy.spin(pid_controller)
	pid_controller.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()


