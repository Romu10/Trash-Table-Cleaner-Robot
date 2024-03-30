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
from detection_interfaces.srv import DetectTableLegs
from detection_interfaces.action import ApproachTable
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

##################### Trash table positions for picking in Map ##############
#                     X            Y           Z            W               #            
table_trash_positions = {                                                   #
    "position_1": [1.10607,    -0.164404,   0.759042,   0.755167],           #
    "position_2": [-0.0491602, -0.689815,  -0.999793,   0.0203558],         #
    "position_3": [-1.46219,   -2.08501,   -0.705164,   0.709044],          #
    "position_4": [3.72892,    -1.550005,     -0.100632,   0.994924],          #
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

        # create a topic publisher for call the robots elevator up
        self.lift_table_pub = self.create_publisher(String,'elevator_up', 10)

        # create a topic publisher for call the robots elevator down
        self.drop_table_pub = self.create_publisher(String,'elevator_down', 10)

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

    def lift_table(self):
        table_up = String()
        table_up.data = ''
        self.lift_table_pub.publish(table_up)

    def drop_table(self):
        table_down = String()
        table_down.data = '' 
        self.drop_table_pub.publish(table_down)

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

        # define the action service
        self.action_client = ActionClient(self, ApproachTable, 'approach_table_as')

        # define a service
        self.move_table_back_srv_client = self.create_client(Trigger, 'move_table_back_srv')

        # wait until action service gets online
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            print('Approach Controller action not available, waiting again...')
        
        # wait until service gets online
        while not self.move_table_back_srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Move Table Back service not available, waiting again...')
        
        # define the variable to send the request
        self.move_table_req = Trigger.Request()
    
    def send_table_move_request(self):
        future = self.move_table_back_srv_client.call_async(self.move_table_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

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

    def send_approach_request(self, waypoint_coordinate_x, waypoint_coordinate_y, waypoint_name, desired_yaw, calculate_yaw):
        
        # action flag
        self.action_success = False

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

        # pass waypoint yaw
        waypoint_goal.desired_yaw = desired_yaw

        # pass calculate yaw indication
        waypoint_goal.calculate_yaw = calculate_yaw
        
        # inform that the goal was sent
        self.get_logger().info('Sending goal request...')

        # send goal
        self.send_goal_future = self.action_client.send_goal_async(waypoint_goal)

        # get the response callback
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def get_action_result(self):
        return self.action_success

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # define a subsription for odom
        self.odom_subscription = self.create_subscription(Odometry, '/diffbot_base_controller/odom', self.odom_callback, 10)

        # run flag
        self.read = False

    def get_position(self):
        position = None
        read = None
        if self.read:
            position = self._position
            read = True
        return position, read

    def calculate_yaw(self, q_x, q_y, q_z, q_w):
        # Calcular ángulo de yaw
        yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
        
        # Ajustar el ángulo para que esté en el rango [-pi, pi]
        if yaw > math.pi:
            yaw -= 2 * math.pi
        elif yaw < -math.pi:
            yaw += 2 * math.pi
        
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
        self.yaw = round(self.calculate_yaw(q_x=q_x, q_y=q_y, q_z=q_z, q_w=q_w), 3)

        self.read = True

def main():

    rclpy.init()
    navigator = BasicNavigator()
    manager = Nav2TaskManager()
    controller = ApproachController()
    robot = RobotController()

    # Set your demo's initial pose
    manager.setRobotInitPosition(navigator, 'Start Position', robot_init_position['start_position'])
    
    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # Define trash table final destination
    request_destination = 'backroom_2'

    # Define the initial variable
    request_table_location = 'position_'

    robot_position = Point()
    odom_read = False

    # Define some variables
    approach_point = Point()
    pre_approach_point = Point()
    middle_point = Point()
    center_point = Point()

    # Iterate over a sequence from 1 to 6
    i = 4
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
            # wait until robot is complete stop
            time.sleep(15)
            
            # request for table verifacation
            table_detection = manager.send_detection_request()
            
            # define a flag var
            table_detected = False

            # define time vars 
            prev_time = None
            elapsed_time = 0
            start_time = time.time()

            while not table_detected and elapsed_time < 20:
                # update table_detected var
                table_detected = table_detection.success
                rclpy.spin_once(manager)
                
                # Update elapsed time and previous time
                prev_time = elapsed_time
                elapsed_time = time.time() - start_time

            # verify and inform if table was detected
            if table_detection.success:
                print('Table Found!')
            else:
                print('Table Not Found in this Place')
                continue

            # first get current position
            while not odom_read:
                robot_position, odom_read = robot.get_position()
                rclpy.spin_once(robot)

            # calculate each point position from current position

            #calculate position for approach_distance point 
            approach_point.x = robot_position.x + table_detection.approach_distance_point.y
            approach_point.y = robot_position.y + table_detection.approach_distance_point.x

            # calculate yaw between current position and approach_distance point
            desired_yaw_1 = math.atan2(approach_point.y - robot_position.y, approach_point.x - robot_position.x)

            # calculate position for table center point
            center_point.x = robot_position.x + table_detection.table_center_point.y 
            center_point.y = robot_position.y + table_detection.table_center_point.x + 0.25

            # calulate yaw betwwen approach point and center point 
            desired_yaw_2 = math.atan2(center_point.y - approach_point.y, center_point.x - approach_point.x)

            # print point generated
            print('Waypoints are:')
            print('Approach Distance Point x: ', approach_point.x)
            print('Approach Distance Point y: ', approach_point.y)
            print('Table Center Point x: ', center_point.x)
            print('Table Center Point y: ', center_point.y)

            # if table detected, go to the approach point
            controller.send_approach_request(waypoint_coordinate_x=approach_point.x,
                                            waypoint_coordinate_y=approach_point.y,
                                            waypoint_name='Approach Point',
                                            desired_yaw=desired_yaw_1,
                                            calculate_yaw=False)
            first_pos = False
            # wait until the robot reach the approach point
            while not first_pos:
                first_pos = controller.get_action_result()
                rclpy.spin_once(controller)
            
            # if robot reached the approach point, then go to the pre approach point
            if first_pos:
                rclpy.spin_once(robot)
                robot_position, odom_read = robot.get_position()
                controller.send_approach_request(waypoint_coordinate_x=center_point.x,
                                                waypoint_coordinate_y=center_point.y,
                                                waypoint_name='Center Point',
                                                desired_yaw=desired_yaw_2,
                                                calculate_yaw=True)
                second_pos = False
                # wait until robot reach the pre approach point
                while not second_pos:
                    second_pos = controller.get_action_result()
                    rclpy.spin_once(controller)
            
                    
            # if all waypoints where reached, then lift the table. 
            if first_pos and second_pos:
                print('Robot is in table center position.')
                time.sleep(5)
                manager.lift_table()
                # change robot footprint, just for square tables
                manager.change_footprint(table_length=0.65)
                table_status = controller.send_table_move_request()
                while not table_status.success:
                    rclpy.spin_once(controller)
                # when the table is lifted clear the costmaps 
                navigator.clearAllCostmaps()
                break

        if not result:
            # Define the home goal position 
            robot_home_position = 'start_position'

            # Go to position 
            manager.goToPosition(navigator, robot_home_position, robot_init_position[robot_home_position])
    
            # Arrival Time 
            manager.arrivalTime(navigator, robot_home_position)

            # Get Task Result 
            result = manager.getTaskResult(navigator, robot_home_position)


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