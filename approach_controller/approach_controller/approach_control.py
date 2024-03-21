import rclpy 
from rclpy.node import Node
from detection_interfaces.srv import DetectTableLegs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from detection_interfaces.action import ApproachTable
from rclpy.action import ActionServer
import math
import numpy as np

class ApproachController(Node):

    def __init__(self):
        super().__init__('trash_table_detection')

        # define a subscription for laser scan
        #self.laserscan_subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

        # define a subsription for odom
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # create an action service
        self._action_server = self.ActionServer(self, ApproachTable, 'approach_table_as', self.execute_callback)
        

    def calculate_yaw(self, q_x, q_y, q_z, q_w):
        yaw = 2 * np.arctan2(np.sqrt(q_x**2 + q_y**2 + q_z**2), q_w)
        return yaw

    def odom_callback(self, msg):
            
        # get robots positions
        self.robot_position_x = msg.pose.pose.position.x
        self.robot_position_y = msg.pose.pose.position.y
        
        # get robots orientation in terms of quaternions
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w

        # Calculate robot yaw in degree
        self.robot_yaw = round(self.calculate_yaw(q_x=q_x, q_y=q_y, q_z=q_z, q_w=q_w), 3)
        print(self.robot_yaw)
    
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

        print(theta_point)
        return theta_point

def main(args=None):
    rclpy.init(args=args)

    approach_controller = ApproachController()

    rclpy.spin(approach_controller)

    approach_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()