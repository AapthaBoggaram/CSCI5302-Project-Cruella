#!/usr/bin/env python3
# stop-sign-detection publisher
import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import CameraMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
# import rospy

class StopSignPublisher(Node):

    def __init__(self):
        super().__init__('stop_sign_publisher')
        self.publisher_image = self.create_publisher(CameraMsg, '/camera_pkg/video_mjpeg', 10)  # to publish images
        self.publisher_deepracer = self.create_publisher(Twist, '/cmd_vel', 10)  # to move car
        self.cv_bridge = CvBridge()
        self.camera = None
        for source in range(11):
            cap = cv2.VideoCapture(source) 
            if cap is None or not cap.isOpened():
                print('Warning: unable to open video source: ', source)
            else:
                self.camera = cap
                print('Camera found on index: %d' %source)
                break
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_camera_image)
        self.i = 0
              
            

    # def testDevice(source):
    #     cap = cv2.VideoCapture(source) 
    #     if cap is None or not cap.isOpened():
    #         print('Warning: unable to open video source: ', source)
    #         return False
    #     else:
    #         return cap

    def publish_camera_image(self):
        # reading camera input from opencv2
        returned, frame = self.camera.read()
        print("ENTERED FUNCTION!")
        if returned:
            print("AAAAAAAAAAAAAAAAAAAAAAA")
            # Publishing the captured image (if sucessful)
            camera_msg = CameraMsg()
            camera_msg.images = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(camera_msg)
            self.get_logger().info('Publishing Image to the subscriber!')

    def car_go(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.i += 1

    def car_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=None)

    stop_sign_publisher = StopSignPublisher()

    rclpy.spin(stop_sign_publisher)

    stop_sign_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
