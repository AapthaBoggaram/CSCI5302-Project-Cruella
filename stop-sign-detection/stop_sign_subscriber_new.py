# stop-sign-detection subscriber
import rclpy
from rclpy.node import Node
from stop_sign.stop_sign_publisher import StopSignPublisher
import keyboard as keyboard

class StopSignSubscriber(Node):

    def __init__(self):
        super().__init__('stop_sign_subscriber')
        self.subscription = self.create_subscription(String, '/stop_sign', self.msg_callback, 10)
        self.stopsignPub = StopSignPublisher()


    def msg_callback(self, msg):

        # receiving published message from publisher
        detected = msg.data # it is either "yes" or "no        

        if detected == "yes":
            print("Stop sign detected! Car will stop for a while now!")
            self.stopsignPub.car_stop()
            time.sleep(10)
            self.stopsignPub.car_go()

        elif detected == "no": 
            # if no stop sign detected, move car without stopping
            self.stopsignPub.car_go()

        else:
            # invalid msg received
            print("Error: Invalid message received from publisher!")
        
        # manual stopping condition for the car
        if keyboard.is_pressed('t'):
            self.stopsignPub.car_stop()
            self.stopsignPub.destroy_node()
            stop_sign_detection_subscriber.destroy_node()
            rclpy.shutdown()

        # cv2.imshow("DeepRacer Camera Feed", frame)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
		
    stop_sign_subscriber= StopSignSubscriber()
		
    rclpy.spin(stop_sign_subscriber)
		
	stop_sign_subscriber.destroy_node()
		
    rclpy.shutdown()


if __name__ == '__main__':
    main()