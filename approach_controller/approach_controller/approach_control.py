import rclpy 
from rclpy.node import Node
from detection_interfaces.srv import DetectTableLegs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from detection_interfaces.action import ApproachTable
from rclpy.action import ActionServer
from geometry_msgs.msg import Point, Twist
import math
import numpy as np
from scipy.spatial.transform import Rotation

class ApproachController(Node):

    # go to point vars
    # robot state variables
    _position = Point()
    _yaw = 0

    # machine state
    _state = 'idle'

    # goal
    _des_pos = Point()

    # parameters
    _yaw_precision = math.pi / 90 # +/- 2 degree allowed
    _dist_precision = 0.05

    def __init__(self):
        super().__init__('approach_controller')

        # define a subscription for laser scan
        #self.laserscan_subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

        # define a subsription for odom
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # define a publisher for cmd
        self._pub_cmd_vel = self.create_publisher(Twist, 'diffbot_base_controller/cmd_vel_unstamped', 10)

        # create an action service
        self._action_server = ActionServer(self, ApproachTable, 'approach_table_as', self.goal_callback)
    
    def goal_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # helper variables
        success = True

        # define desired position and errors
        self._des_pos = goal_handle.request.goal_position
        desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
        err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
        err_yaw = desired_yaw - self._yaw

        # perform task
        while err_pos > self._dist_precision and success:
            # update vars
            desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
            err_yaw = desired_yaw - self._yaw
            err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
            # print vars status
            self.get_logger().info("Current Yaw: %s" % str(self._yaw))
            self.get_logger().info("Desired Yaw: %s" % str(desired_yaw))
            self.get_logger().info("Error Yaw: %s" % str(err_yaw))
            self.get_logger().info("Error Pos: %s" % str(err_pos))

            # logic goes here
            if goal_handle.is_cancel_requested:
                # cancelled
                self.get_logger().info("The goal has been cancelled/preempted")
                goal_handle.cancelled()
                success = False
            elif math.fabs(err_yaw) > self._yaw_precision:
                # fix yaw
                self.get_logger().info("fix yaw")
                self._state = 'fix yaw'
                twist_msg = Twist()
                twist_msg.angular.z = 0.3 if err_yaw > 0 else -0.3
                self._pub_cmd_vel.publish(twist_msg)
            else:
                # go to point
                self.get_logger().info("going to point: %s" % str(goal_handle.request.goal_name))
                self._state = goal_handle.request.goal_name
                twist_msg = Twist()
                twist_msg.linear.x = 0.20
                twist_msg.angular.z = 0.00
                # twist_msg.angular.z = 0.1 if err_yaw > 0 else -0.1
                self._pub_cmd_vel.publish(twist_msg)

            # use this to send feedback
            feedback_msg = ApproachTable.Feedback()
            feedback_msg.state = goal_handle.request.goal_name
            feedback_msg.current_position = self._position

            # update external variables
            rclpy.spin_once(self)
            
        # stop
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self._pub_cmd_vel.publish(twist_msg)

        # return success
        if success:
            # use this to indicate that the goal succeed
            goal_handle.succeed()

            result = ApproachTable.Result()
            result.success = True
        else:
            result = ApproachTable.Result()
            result.success = False
            
        
        
        

        return result

    def calculate_yaw(self, q_x, q_y, q_z, q_w):
        # Crear el objeto Rotation a partir de los componentes x, y, z, w
        rotation = Rotation.from_quat([q_x, q_y, q_z, q_w])

        # Convertir la rotación a un ángulo de Euler en grados
        euler_angles = rotation.as_euler('zyx', degrees=False)

        # Extraer el ángulo de yaw de los ángulos de Euler
        yaw = euler_angles[0]  # El ángulo de yaw está en el primer elemento

        return yaw

    def odom_callback(self, msg):
        
        # get robots positions
        self._position = msg.pose.pose.position
        
        # get robots orientation in terms of quaternions
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w

        # Calculate robot yaw in degree
        self._yaw = round(self.calculate_yaw(q_x=q_x, q_y=q_y, q_z=q_z, q_w=q_w), 3)
    
    def calculate_path_point_orientation(self, x_point, y_point, init_pos_x, init_pos_y, theta_robot):
        
        # calculate relative coordinates of two points
        x_rel = x_point - init_pos_x
        y_rel = y_point - init_pos_y

        # calculate orientation of point from the robot
        theta_point = math.atan2(y_rel, x_rel) - theta_robot

        # mnake sure the angle is in range -pi to pi
        if theta_point > math.pi:
            theta_point -= 2 * math.pi
        elif theta_point < -math.pi:
            theta_point += 2 * math.pi

        return theta_point

def main(args=None):
    rclpy.init(args=args)

    approach_controller = ApproachController()

    rclpy.spin(approach_controller)

    approach_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()