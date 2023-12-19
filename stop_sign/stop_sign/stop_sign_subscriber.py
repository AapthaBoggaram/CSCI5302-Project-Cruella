# stop-sign-detection subscriber
import rclpy
from sensor_msgs.msg import Image#, VideoStateSrv
from geometry_msgs.msg import Twist
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from rclpy.node import Node
import cv2
import time
import numpy as np
import keyboard as keyboard
from cv_bridge import CvBridge

class StopSignSubscriber(Node):

	def __init__(self):
		super().__init__('stop_sign_subscriber')
		self.subscription = self.create_subscription(Image, '/display_mjpeg', self.msg_callback, 10)
		print('Sub made')
		self.publisher_servo = self.create_publisher(ServoCtrlMsg, '/cmdvel_to_servo_pkg/servo_msg', 10)
		#self.publisher_deepracer = self.create_publisher(Twist, '/cmd_vel', 10)  # to move car
		print('Pub made')
		self.i = 0
		self.bridge = CvBridge()
		self.speed_mod = .70


	def msg_callback(self, msg):
		print('Callback called(back)')
		stop_sign_detected = self.stop_sign_detection(msg)
		if(stop_sign_detected):
			self.car_stop()
			self.get_logger().info("Stop sign detected! Car will stop for a while now!")
			time.sleep(3)
			self.car_go()
			time.sleep(2.5)
		else:
			# publish no message to subscriber if stop-sign is not detected
			self.car_go()
			self.get_logger().info("No stop sign detected! Car will keep moving!")


		# manual stopping condition for the car
		if keyboard.is_pressed('t') and keyboard.is_pressed('x'):
			print('oh no')
			self.car_stop()
			#stop_sign_subscriber.destroy_node()
			rclpy.shutdown()

	def stop_sign_detection(self, msg):
		print('stop sign detection called')
		print('trying to convert a frame...')
		frame = self.bridge.imgmsg_to_cv2(msg,'bgra8')
		self.get_logger().info("Image found!")
		#cv2.imshow("Image",frame)
		cv2.waitKey(1)
		print('transforming a frame...')
		hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #transforming an rgv image to hsv image
		lower_red = cv2.inRange(hsv_image, (0,50,20), (5,255,255)) #mask 1 for lower red range
		upper_red = cv2.inRange(hsv_image, (175,50,20), (180,255,255))  #mask 2 for upper red range
		print('creating a mask...')
		mask = cv2.bitwise_or(lower_red, upper_red)
		
		contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)    # finding boundaries of mask

		if contours:
			print('contour check')
			# if boundaries of mask exist, find countours with max area
			contour = max(contours, key=cv2.contourArea)
			area = cv2.contourArea(contour)
			print('checking area...')
			if area > 325:  # change this to suit the area of stop sign
				return True # returing true if stop sign is detected
			return False    # false if no stop sign is detected



	def car_go(self):
		# cmd_vel publish
		# msg = Twist()
		# msg.linear.x = 0.5
		# msg.angular.z = self.angle
		# self.publisher_cmdvel.publish(msg)

		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.angle = 0.0
		msg.throttle = self.speed_mod
		self.publisher_servo.publish(msg)

		# self.get_logger().info('Publishing: "%s"' % msg)

	def car_stop(self):
		# cmd_vel publish
		# msg = Twist()
		# msg.linear.x = 0.0
		# msg.angular.z = 0.0
		# self.publisher_cmdvel.publish(msg)
		
		# ServoCtrlMsg publish
		msg = ServoCtrlMsg()
		msg.angle = 0.0
		msg.throttle = 0.0
		self.publisher_servo.publish(msg)

		# self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):

	rclpy.init(args=args)

	stop_sign_subscriber= StopSignSubscriber()

	rclpy.spin(stop_sign_subscriber)

	stop_sign_subscriber.destroy_node()

	rclpy.shutdown()


if __name__ == '__main__':
	main()
