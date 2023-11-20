# stop-sign-detection subscriber
import rclpy
from rclpy.node import Node
from camera_pkg.msg import CameraMsg
from cv_bridge import CvBridge
import cv2
import numpy as np

class StopSignSubscriber(Node):
    def __init__(self):
        super().__init__('stop_sign_detection_subscriber')
		self.subscription = self.Subscriber(CameraMsg, '/camera_pkg/video_mjpeg', image_callback, 10)
		self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        # print(f"Received stop sign at ({msg.detection_coordinates.x}, {msg.detection_coordinates.y})")
        frame = self.cv_bridge.imgmsg_to_cv2(msg.image, desired_encoding="bgr8")

        # Process the image for stop sign detection
        detected = self.stop_sign_detection(frame)

        if detected:
            print("Stop sign detected!")
            

        cv2.imshow("DeepRacer Camera Feed", frame)
        cv2.waitKey(1)

    def stop_sign_detection(self, frame):
        # Color-based detection (replace with your own method if needed)
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Assuming only one stop sign for simplicity
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            if area > 1000:  # Set a suitable threshold area for your scenario
                return True

        return False

def main(args=None):
    rclpy.init(args=args)
		
    stop_sign_detection_subscriber= StopSignSubscriber()
		
    rclpy.spin(stop_sign_detection_subscriber)
		
	stop_sign_detection_subscriber.destroy_node()
		
    rclpy.shutdown()

if __name__ == '__main__':
    main()
