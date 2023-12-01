import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PidController(Node):
    
    def __init__(self):
        super().__init__('pid_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher # to prevent unused variable warning. Remove this later
        self.subscriber = self.create_subscription(LaserScan,'/scan',self.msg_callback,10)
        self.ranges = 0
    
    def msg_callback(self, msg):
        # testing for 540 index. If it doesn't work, test for 360 index
        
        # number of lidar readings in a given subscription
        num_of_degrees = len(msg.ranges)

        # left and right index w.r.t. the number of lidar ranges
        left_index = int(num_of_degrees / 4)
        right_index = int(num_of_degrees * 3/4)

        # left and right readings
        left_reading = msg.ranges[left_index]
        right_reading = msg.ranges[right_index]

        print("left reading: ", left_reading, " right reading: ", right_reading)

        # print(len(msg.ranges))
        # self.ranges = [i for i in msg.ranges if i > msg.range_min and i < msg.range_max]
        # print("shortened range: ", len(self.ranges), " full msg.ranges: ", len(msg.ranges), " range_min: ", msg.range_min, " range_max: ", msg.range_max)
        


def main(args=None):

    rclpy.init(args=args) 

    pid_controller = PidController()   
    
    rclpy.spin(pid_controller)   

    pid_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
