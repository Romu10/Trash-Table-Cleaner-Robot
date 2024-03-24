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
from action_msgs.msg import GoalStatus

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

class ApproachController(Node):

    def __init__(self):
        super().__init__('approach_controller_client')

        # define the action service
        self.action_client = ActionClient(self, ApproachTable, 'approach_table_as')

        # wait until action service gets online
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            print('Approach Controller action not available, waiting again...')

        # action flag
        self.action_success = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.get_logger().info('Goal succeeded !')
                self.action_success = True
            else:
                self.get_logger().info('Goal failed !')
                self.action_success = False

        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

    def send_approach_request(self, waypoint_coordinate_x, waypoint_coordinate_y, waypoint_name):
        
        # wait until action service is online
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
       
        # define the goal msg
        waypoint_goal = ApproachTable.Goal()

        # pass received coordinates if table is detected
        waypoint_goal.goal_position.x = waypoint_coordinate_x
        waypoint_goal.goal_position.y = waypoint_coordinate_y

        # pass waypoint name either
        waypoint_goal.goal_name = waypoint_name
        
        # inform that the goal was sent
        self.get_logger().info('Sending goal request...')

        # send goal
        self.send_goal_future = self.action_client.send_goal_async(waypoint_goal)

        # get the response callback
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def get_action_result(self):
        return self.action_success

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
                controller.send_approach_request(waypoint_coordinate_x=table_detection.approach_distance_point.x,
                                                    waypoint_coordinate_y= table_detection.approach_distance_point.y,
                                                        waypoint_name='Approach Point')
                first_pos = False
                # wait until the robot reach the approach point
                while not first_pos:
                    first_pos = controller.get_action_result()
                    rclpy.spin_once(controller)
                
                # if robot reached the approach point, then go to the middle point
                if first_pos:
                    controller.send_approach_request(waypoint_coordinate_x=table_detection.table_middle_point.x,
                                                    waypoint_coordinate_y= table_detection.table_middle_point.y,
                                                        waypoint_name='Middle Point')
                    second_pos = False
                    # wait until robot reach the middle point
                    while not second_pos:
                        second_pos = controller.get_action_result()
                        rclpy.spin_once(controller)
            
        break        
                # if reached middle point, go to center point
                #    if reach_middle_point:
                #        reach_center_point = manager.send_approach_request(waypoint_coordinate_x= table_detection.table_center_point.x,
                #                                                            waypoint_coordinate_y= table_detection.table_center_point.y,
                #                                                                waypoint_name='Center Point')
                # break

                # if table detected and failed while approaching to table, restart the process                                                        
                # else:
                #    print('Process failed while approaching to table, repeting process.')
                
            # else: 
            #    print('tale not detected')
                # update i in order to go to the next position
            #    i += 1

    while not navigator.isTaskComplete():
        pass

    exit(0)
    
if __name__ == '__main__':
    main()