import rclpy 
import time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger

class MoveTableBack(Node):

    def __init__(self):
        super().__init__('approach_controller')

        # define a subsription for odom
        self.odom_subscription = self.create_subscription(Odometry, '/diffbot_base_controller/odom', self.odom_callback, 10, 
                                            callback_group=ReentrantCallbackGroup())

        # define a publisher for cmd
        self._pub_cmd_vel = self.create_publisher(Twist, 'diffbot_base_controller/cmd_vel_unstamped', 10)

        # define the service
        self.srv = self.create_service(Trigger, 'move_table_back_srv', self.move_table)

        self.get_logger().info('Move Table Service Online...')
    
    def move_table(self, request, response):
        twist_msg = Twist()
        start_time = time.time()
        elapsed_time = 0.0
        while elapsed_time < 13:
            elapsed_time = time.time() - start_time
            twist_msg.linear.x = -0.20
            twist_msg.angular.z = -0.01
            self._pub_cmd_vel.publish(twist_msg)
        response.success = True
        response.message = 'Ready'
        return response

    def odom_callback(self, msg):
        # get robots positions
        self._position = msg.pose.pose.position

def main(args=None):
    rclpy.init(args=args)

    move_table_back = MoveTableBack()

    executor = MultiThreadedExecutor()

    rclpy.spin(move_table_back, executor=executor)

    move_table_back.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
