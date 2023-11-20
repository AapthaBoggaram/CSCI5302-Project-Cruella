# stop-sign-detection publisher
import rclpy
from rclpy.node import Node
from camera_pkg.msg import CameraMsg
from cv_bridge import CvBridge
# from geometry_msgs.msg import Twist
# import rospy

class StopSignPublisher:

    def __init__(self):
        super().__init__('stop_sign_publisher')
        self.publisher_image = self.create_publisher(CameraMsg, '/camera_pkg/video_mjpeg', 10)
        # self.publisher_deepracer = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cv_bridge = CvBridge()
        self.camera = cv2.VideoCapture(0)

    def publish_camera_image(self):
        # reading camera input
        returned, frame = self.camera.read()

        # if we get a valid image that is captured, then publish it
        if returned:
            # Publish the captured image
            camera_msg = CameraMsg()
            camera_msg.image = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.publisher.publish(camera_msg)


def main(args=None):
    rclpy.init(args=None)

    stop_sign_publisher = StopSignPublisher()

    rclpy.spin(stop_sign_publisher.node)

    stop_sign_publisher.node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
