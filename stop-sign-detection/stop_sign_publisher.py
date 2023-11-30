# stop-sign-detection publisher
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2

class StopSignPublisher(Node):

    def __init__(self):
        super().__init__('stop_sign_publisher')
        self.publisher_ = self.create_publisher(String, '/stop_sign', 10)  # to publish images
        self.publisher_deepracer = self.create_publisher(Twist, '/cmd_vel', 10)  # to move car
        # self.camera = cv2.VideoCapture(1, cv2.CAP_V4L)
        # for source in range(11):
        #     cap = cv2.VideoCapture(source) 
        #     if cap is None or not cap.isOpened():
        #         print('Warning: unable to open video source: ', source)
        #     else:
        #         self.camera = cap
        #         print('Camera found on index: %d' %source)
        #         break
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_detection_output)
        self.i = 0


    def publish_detection_output(self):
        # detecting whether there is a stop sign in frames read
        stop_sign_detected = self.stop_sign_detection()

        if(stop_sign_detected):
            # publish "yes" message to subscriber if stop-sign is detected
            msg = String()
            msg.data = "yes"
            self.publisher_.publish(msg)
            self.get_logger().info("Stop sign detected! Car will stop for a while now!")
        else:
            # publish no message to subscriber if stop-sign is not detected
            msg = String()
            msg.data = "no"
            self.publisher_.publish(msg)
            self.get_logger().info("No stop sign detected! Car will keep moving!")


    def stop_sign_detection(self):
        
        camera = cv2.VideoCapture(1, cv2.CAP_V4L)
        returned, frame = camera.read()
        if returned:
            self.get_logger().info("IMAGE FOUNDDDDDDDDDDDDDDDDDDDDDDDDDDDDD")
            cv2.imshow("image",frame)
            hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #transforming an rgv image to hsv image
            lower_red = np.array([0, 100, 100]) # red color lower bound in hsv space
            upper_red = np.array([10, 255, 255])    # red color upper bound in hsv space
            mask = cv2.inRange(hsv_image, lower_red, upper_red) # creating a mask (pixels in range -> white and outside -> black)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)    # finding boundaries of mask

            if contours:
                # if boundaries of mask exist, find countours with max area
                contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(contour)
                if area > 1000:  # change this to suit the area of stop sign
                    return True # returing true if stop sign is detected
                
                return False    # false if no stop sign is detected
        
        # return false if camera doesn't read anything             
        return False


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