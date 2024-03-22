#! /usr/bin/env python3

import time
from copy import deepcopy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from detection_interfaces.srv import DetectTableLegs
from detection_interfaces.action import ApproachTable
from rclpy.action import ActionClient

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

##################### Trash table positions for picking in Map ##############
#                     X            Y           Z            W               #            
table_trash_positions = {                                                   #
    "position_1": [0.954607,    0.364404,   0.709042,   0.705167],          #
    "position_2": [-0.0491602, -0.689815,  -0.999793,   0.0203558],         #
    "position_3": [-1.46219,   -2.08501,   -0.705164,   0.709044],          #
    "position_4": [3.92892,    -1.435,     -0.100632,   0.994924],          #
    "position_5": [4.92647,     0.19721,    0.691586,   0.722294],          #
    "position_6": [1.1534,     -2.76343,   -0.694667,   0.719332]}          #
#############################################################################                                                                         

################## Shipping destination for dropoff trash tables ############
shipping_destinations = {                                                   #              
    "backroom_1": [7.21814, -2.1297, -0.999085, 0.0427718],                 #
    "backroom_2": [8.44286, -2.18891, 0.00634964, 0.99998],}                #
#############################################################################

######################     Robot initial positions     ######################
robot_init_position = {                                                     #              
    "start_position": [0.00, 0.00, 0.00, 0.99]}                             #
#############################################################################

class Nav2TaskManager(Node):

    def __init__(self):
        super().__init__('nav2_task_manager')

        # create a service client
        self.service_client = self.create_client(DetectTableLegs, 'find_table_srv')

        # wait until service gets online
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Table Detection service not available, waiting again...')
        
        # define the variable to send the request
        self.req = DetectTableLegs.Request()

        # define the action service
        self.action_client = ActionClient(self, ApproachTable, 'approach_table_as')

        # wait until action service gets online
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            print('Approach Controller action not available, waiting again...')
        
    def send_approach_request(self, waypoint_coordinate_x, waypoint_coordinate_y, waypoint_name):
        # define the goal msg
        waypoint_goal = ApproachTable.Goal()

        # pass received coordinates if table is detected
        waypoint_goal.goal_position.x = waypoint_coordinate_x
        waypoint_goal.goal_position.y = waypoint_coordinate_y

        # pass waypoint name either
        waypoint_goal.goal_name = waypoint_name
        
        self.action_client.wait_for_server()

        return self.action_client.send_goal_async(waypoint_goal)

    def send_detection_request(self):
        self.future = self.service_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
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

def main():

    rclpy.init()
    navigator = BasicNavigator()
    manager = Nav2TaskManager()

    # Set your demo's initial pose
    manager.setRobotInitPosition(navigator, 'Start Position', robot_init_position['start_position'])
    
    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # Define trash table final destination
    request_destination = 'backroom_2'

    # Define the initial variable
    request_table_location = 'position_'

    # Iterate over a sequence from 1 to 6
    i = 1
    while i < 6:
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
            print('Looking for Table in this Place\n')
            time.sleep(10)
            table_detection = manager.send_detection_request()
            if table_detection.success:
                print('Table detected\n')
                print('Waypoints are:\n')
                print('Table Center Point x: ', table_detection.table_center_point.x)
                print('Table Center Point y: ', table_detection.table_center_point.y)
                print('Table Middle Point x: ', table_detection.table_middle_point.x)
                print('Table Middle Point y: ', table_detection.table_middle_point.y)
                print('Approach Distance Point x: ', table_detection.approach_distance_point.x)
                print('Approach Distance Point y: ', table_detection.approach_distance_point.y)
                
                # if table detected, go to the approach point
                reach_approach_point = manager.send_approach_request(waypoint_coordinate_x=table_detection.approach_distance_point.x,
                                                                        waypoint_coordinate_y= table_detection.approach_distance_point.y,
                                                                            waypoint_name='Approach Point')
                while not reach_approach_point:
                    rclpy.spin_until_future_complete(manager, reach_approach_point)
                
                # if reached approach point, go to middle point 
                # if reach_approach_point:
                #    print('Approach Reached')
                #    reach_middle_point = manager.send_approach_request(waypoint_coordinate_x= table_detection.table_middle_point.x,
                #                                                            waypoint_coordinate_y= table_detection.table_middle_point.y,
                #                                                                waypoint_name='Middle Point')
                # if reached middle point, go to center point
                #    if reach_middle_point:
                #        reach_center_point = manager.send_approach_request(waypoint_coordinate_x= table_detection.table_center_point.x,
                #                                                            waypoint_coordinate_y= table_detection.table_center_point.y,
                #                                                                waypoint_name='Center Point')
                # break

                # if table detected and failed while approaching to table, restart the process                                                        
                # else:
                #    print('Process failed while approaching to table, repeting process.')
                
            else: 
                print('tale not detected')
                # update i in order to go to the next position
                i += 1

    while not navigator.isTaskComplete():
        pass

    exit(0)
    
if __name__ == '__main__':
    main()