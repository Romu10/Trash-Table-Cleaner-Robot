#! /usr/bin/env python3

import time
import math
from copy import deepcopy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point32, Polygon, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty
from rclpy.duration import Duration
import rclpy
from detection_interfaces.srv import DetectTableLegs, TablePosition
from detection_interfaces.action import ApproachTable
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

##################### Trash table positions for picking in Map ##############
#                     X            Y           Z            W               #            
table_trash_positions = {                                                   #
    "position_1": [-0.9323,    -2.2357,   -0.6959,   0.7181],               #
    "position_2": [-1.0839,    -1.0478,   -0.9999,   0.0014],               #
    "position_3": [ 3.8661,    -1.4054,    0.0000,   0.8009],               #
    "position_4": [ 0.0000,     0.0000,    0.0000,   0.0000]}               #
#############################################################################   

################## Shipping destination for dropoff trash tables ############
shipping_destinations = {                                                   #              
    "backroom_1": [ 4.8152, -0.4654,  0.0130,  0.9999],                     #  Door Position 1
    "backroom_2": [ 6.6530, -0.4890,  0.0006,  1.0000],                     #  Door Position 2
    "backroom_3": [ 9.6016, -0.5445, -0.0008,  1.0000]}                     #  Table Destination
#############################################################################

######################     Robot initial positions     ######################
robot_init_position = {                                                     #              
    "start_position": [0.00, 0.00, 0.000, 0.99]}                  #
#############################################################################

class Nav2TaskManager(Node):

    def __init__(self):
        super().__init__('nav2_task_manager')

        # create a service client
        self.detect_table_srv_client = self.create_client(DetectTableLegs, 'find_table_srv')

        # create a topic publisher for change global and local footprint
        self.global_footprint = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.local_footprint = self.create_publisher(Polygon, '/local_costmap/footprint', 10)

        # wait until service gets online
        while not self.detect_table_srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Table Detection service not available, waiting again...')
        
        # define the variable to send the request
        self.req = DetectTableLegs.Request()
    
    def change_footprint(self, table_length):
        polygon_msg = Polygon()

        point1 = Point32()
        point1.x = table_length/2
        point1.y = -table_length/2

        point2 = Point32()
        point2.x = table_length/2
        point2.y = table_length/2

        point3 = Point32()
        point3.x = -table_length/2
        point3.y = table_length/2

        point4 = Point32()
        point4.x = -table_length/2
        point4.y = -table_length/2

        polygon_msg.points = [point1, point2, point3, point4]

        self.global_footprint.publish(polygon_msg)
        self.local_footprint.publish(polygon_msg)

    def send_detection_request(self):
        future = self.detect_table_srv_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def setRobotInitPosition(self, navigator, location, position, frame = 'map'):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = position[0]
        initial_pose.pose.position.y = position[1]
        initial_pose.pose.orientation.z = position[2]
        initial_pose.pose.orientation.w = position[3]
        print('Set initial position at ' + location+'.')
        navigator.setInitialPose(initial_pose)

    def goToPosition(self, navigator, location, position, frame = 'map'):
        table_item_pose = PoseStamped()
        table_item_pose.header.frame_id = frame
        table_item_pose.header.stamp = navigator.get_clock().now().to_msg()
        table_item_pose.pose.position.x = position[0]
        table_item_pose.pose.position.y = position[1]
        table_item_pose.pose.orientation.z = position[2]
        table_item_pose.pose.orientation.w = position[3]
        print('Looking for trash table picking at ' + location + '.')
        navigator.goToPose(table_item_pose)

    def arrivalTime(self, navigator, request_table_location):
        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival at ' + request_table_location +
                    ' for drop off: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

    def getTaskResult(self, navigator, request_table_location):
        result = navigator.getResult()
        task_result = False
        if result == TaskResult.SUCCEEDED:
            task_result = True
            print('Arrived successfully to ' + request_table_location + ' .')

        elif result == TaskResult.CANCELED:
            task_result = False
            print('Task at ' + request_table_location + ' was canceled. Returning to staging point...')\

            # Return to starter position
            self.goToPosition(navigator, 'Start Position', robot_init_position['start_position'])

        elif result == TaskResult.FAILED:
            task_result = False
            print('Task at ' + request_table_location + ' failed!')
            exit(-1)

        return task_result

class ApproachController(Node):

    def __init__(self):
        super().__init__('approach_controller_client')


        # define a service
        self.move_table_back_srv_client = self.create_client(Trigger, 'move_table_back_srv')

        # create a service client
        self.get_table_pos_tf = self.create_client(TablePosition, 'get_position_tf')

        # create a service client
        self.find_table_srv_client = self.create_client(DetectTableLegs, 'find_table_srv')
        
        # wait until service gets online
        while not self.get_table_pos_tf.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Transform service not available, waiting again...')

        # wait until service gets online
        while not self.move_table_back_srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Move Table Back service not available, waiting again...')

        # wait until service gets online
        while not self.find_table_srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Find Table service not available, waiting again...')
        
        # define the variable to send the request
        self.move_table_req = Trigger.Request()

        # define the var to send the request for tf position
        self.tf_pos_req = TablePosition.Request()

        # define the variable to send the request
        self.find_table_req = DetectTableLegs.Request()
    
    def send_table_move_request(self):
        future = self.move_table_back_srv_client.call_async(self.move_table_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_approach_request(self):
        self.tf_pos_req.data = True
        future = self.get_table_pos_tf.call_async(self.tf_pos_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_pos_tf_request(self):
        future = self.find_table_srv_client.call_async(self.find_table_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():

    rclpy.init()
    navigator = BasicNavigator()
    manager = Nav2TaskManager()
    controller = ApproachController()

    # Set your demo's initial pose
    manager.setRobotInitPosition(navigator, 'Start Position', robot_init_position['start_position'])
    
    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # Define trash table final destination
    request_destination = 'backroom_3'

    # Define the initial variable
    request_table_location = 'position_'

    # Define flag for when no table found
    table_not_found_in_room = False

    # Define flag for when table is lifted
    table_lifted = False

    # Iterate over a sequence from 1 to 3
    i = 1
    while i < 4:
        # Define the goal position 
        request_table_location = 'position_' + str(i)

        # Go to position 
        manager.goToPosition(navigator, request_table_location, table_trash_positions[request_table_location])
 
        # Arrival Time 
        manager.arrivalTime(navigator, request_table_location)

        # Get Task Result 
        result = manager.getTaskResult(navigator, request_table_location)

        # Do something if result depending on result status
        if result: 

            # wait until robot is complete stop
            time.sleep(2)

            # request for table verifacation
            table_detection = manager.send_detection_request()

            print(table_detection.success)         

            if not table_detection.success:
                
                print("Table Not Found")
                print("Looking in Next Position")
                time.sleep(2)
                i = i + 1
                if i == 4:
                    table_not_found_in_room = True
                continue

            # verify and inform if table was detected
            if table_detection.success:
                print('Table Found!')

                # if table found then move underneath table center
                in_center_position = controller.send_approach_request()
                
                # wait for the result
                while not in_center_position.success:
                    rclpy.spin_once(controller)
                
            else:
                print('Table Not Found in this Place')
                continue

                    
            # if robot is under table center. 
            if in_center_position.success:
                print('Robot is in table center position.')
                time.sleep(5)

                # change robot footprint, just for square tables
                manager.change_footprint(table_length=0.65)
                table_status = controller.send_table_move_request()
                while not table_status.success:
                    rclpy.spin_once(controller)
                # when the table is lifted clear the costmaps 
                navigator.clearAllCostmaps()
            
                table_lifted = True
                
                break

        if table_not_found_in_room:

            print('No Table Found, Going to HOME position!')
            time.sleep(5)

            # Define the home goal position 
            robot_home_position = 'start_position'

            # Go to position 
            manager.goToPosition(navigator, robot_home_position, robot_init_position[robot_home_position])
    
            # Arrival Time 
            manager.arrivalTime(navigator, robot_home_position)

            # Get Task Result 
            result = manager.getTaskResult(navigator, robot_home_position)


    if table_lifted: 
        # Behavior with table lifted

        # Go to position 
        manager.goToPosition(navigator, request_destination, shipping_destinations[request_destination])

        # Arrival Time 
        manager.arrivalTime(navigator, request_destination)

        # Get Task Result 
        result = manager.getTaskResult(navigator, request_destination)

                
    while not navigator.isTaskComplete():
        pass

    exit(0)
    
if __name__ == '__main__':
    main()