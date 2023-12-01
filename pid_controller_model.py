import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import keyboard

class PidController(Node):
	
	def __init__(self):
		super().__init__('pid_controller')
		self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		self.subscriber = self.create_subscription(LaserScan,'/scan',self.msg_callback,10)
		self.ranges = 0.0
		self.prev_error = 0.0
		self.k_p = 0.15
		self.k_i = 0.0001
		self.k_d = 3
		self.dt = 1
		self.integral = 0.0
		self.angle = 0.0
	
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
		left_reading = msg.ranges[int(num_of_degrees / 4)]
		right_reading = msg.ranges[int(num_of_degrees * 3/4)]

		# adjusting angle based on pid (?)
		error = left_reading - right_reading

		if(error > -80 and error < 80):
			self.integral = self.integral + error * self.dt
			derivative = (error - self.prev_error) / self.dt
			self.angle = self.k_p * error + self.k_i * self.integral + self.k_d * derivative
			print(error)
			self.prev_error = error

			# updating the angle in car_go command
			self.car_go()

	
	def car_go(self):
		msg = Twist()
		msg.linear.x = 0.5
		msg.angular.z = self.angle
		self.publisher.publish(msg)
		# self.get_logger().info('Publishing: "%s"' % msg)

	def car_stop(self):
		msg = Twist()
		msg.linear.x = 0.0
		msg.angular.z = 0.0
		self.publisher.publish(msg)
		# self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):

	rclpy.init(args=args) 

	pid_controller = PidController()   
	
	rclpy.spin(pid_controller)   

	pid_controller.destroy_node()

	rclpy.shutdown()


if __name__ == '__main__':
	main()
