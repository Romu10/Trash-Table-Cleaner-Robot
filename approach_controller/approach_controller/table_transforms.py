import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point, Twist
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool
from detection_interfaces.srv import TablePosition
from nav_msgs.msg import Odometry
from rclpy.time import Time
import math
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class TableTransformPublisher(Node):

    # parameters
    _yaw_precision = 0.03   # +/- 2 degree allowed
    _dist_precision = 0.15
    _dist_from_ctp = 0.10

    def __init__(self):
        super().__init__('trash_table_transform_publisher')

        # define the tf broadcaster to publish tf
        #self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # define the service
        self.srv = self.create_service(TablePosition, 'get_position_tf', self.get_position_from_tf, 
                                                        callback_group=ReentrantCallbackGroup())

        # define a publisher for cmd
        self._pub_cmd_vel = self.create_publisher(Twist, 'cleaner_2/cmd_vel', 10)

        # define a publisher for the elevator
        self._pub_elevator = self.create_publisher(String, 'set_elevator', 10)

        # define tf listener 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # initialize some needed vars
        self.publish_flag = False
        self.published_flag = False

        # transforms
        self.leg_1_transform = Point()
        self.center_point_transform = Point()
        self.approach_point_transform = Point()
        self.robot_position = Point()
        self.odom_map = Point()

        # save table positions from robot_base_link
        self.center_point = Point()
        self.approach_point = Point()

        self.robot_tf_yaw = 0.00

        # create flag for start broadcasting tfs 
        self.start_broadcasting = False

        # create flag to read transforms once
        self.read = False
        self.process = False

        # 
        self.success_tf_apr = False
        self.success_tf_ctp = False
        self.success_aprch = False
        self.needed_dist_apr = 1.0
        self.needed_dist_ctp = 1.0
        self.stop_adj_yaw = False

        # Call on_timer function every second
        self.timer = self.create_timer(0.05, self.on_timer, callback_group=ReentrantCallbackGroup())
        
        # Make sure the elevator start down
        cmd_elevator = String()
        cmd_elevator.data = 'down'
        self._pub_elevator.publish(cmd_elevator)
        self.get_logger().warning('Elevator Down')
        time.sleep(5)

        self.get_logger().info('Transform Position Service Online...')
    
    def on_timer(self):

        if self.start_broadcasting:

            prev_time = None
            elapsed_time = 0
            start_time = time.time()

            # keep calculating the robot orientation everytime
            self.robot_position = self.get_transform(target_frame='map', source_frame='robot_base_link')

            # calculate robot orientation
            self.robot_tf_yaw = self.calculate_yaw( q_x= self.robot_position.transform.rotation.x,
                                                    q_y= self.robot_position.transform.rotation.y, 
                                                    q_z= self.robot_position.transform.rotation.z,
                                                    q_w= self.robot_position.transform.rotation.w)
            
            # get the transform position 
            while not self.read:
                self.center_point_transform = self.get_transform(target_frame='map', source_frame='table_center')
                self.approach_point_transform = self.get_transform(target_frame='map', source_frame='approach_distance')
                time.sleep(0.1)
                if self.center_point_transform and self.approach_point_transform:
                    # indicate that the transfrom is running well
                    self.read = False
                    break

            # save center point position from base_link
            self.center_point.x = self.center_point_transform.transform.translation.x
            self.center_point.y = self.center_point_transform.transform.translation.y

            # save approach point position from base_link
            self.approach_point.x = self.approach_point_transform.transform.translation.x 
            self.approach_point.y = self.approach_point_transform.transform.translation.y

            # Calculate control commands using PID
            if prev_time is not None:  
                dt = elapsed_time - prev_time
            else:
                dt = 0.000001

            # operate the approach
            if not self.success_tf_apr: 
                self.needed_dist_apr, err_yaw, self.success_tf_apr = self.approach_proccess(point_x= self.approach_point.x, 
                                                                                            point_y= self.approach_point.y, 
                                                                                            precision= self._dist_precision,
                                                                                            position= "Approach Point")

            if self.success_tf_apr and self.center_point_transform and not self.success_tf_ctp: 
                self.needed_dist_ctp, err_yaw, self.success_tf_ctp = self.approach_proccess(point_x= self.center_point.x, 
                                                                                            point_y= self.center_point.y, 
                                                                                            precision= self._dist_from_ctp,
                                                                                            position= "Center Point")
            
            if self.success_tf_apr and self.success_tf_ctp:
                self.get_logger().warning("Goal Reached GOOD")
                self.success_aprch = True

            if math.fabs(err_yaw) > self._yaw_precision and not self.stop_adj_yaw:
                twist_msg = Twist()
                twist_msg.angular.z = 0.0050 if err_yaw > 0 else -0.0050
                self._pub_cmd_vel.publish(twist_msg)
                #self.get_logger().info("Need Yaw")
            else:
                twist_msg = Twist()
                twist_msg.linear.x = 0.05
                self._pub_cmd_vel.publish(twist_msg)
                #self.get_logger().info("Need Distance")

            if self.success_aprch: 
                self.read = True
                
                # make sure the robot completly stopped
                twist_msg = Twist()
                twist_msg.linear.x = 0.00
                twist_msg.angular.z = 0.00
                self._pub_cmd_vel.publish(twist_msg)
                
                # Once the robot is in position lift the table
                cmd_elevator = String()
                cmd_elevator.data = 'up'
                self._pub_elevator.publish(cmd_elevator)
                self.get_logger().warning('Elevator Down')
                time.sleep(5)
                self.get_logger().warning('OPERATION SUCCESS')

                # End the service
                self.start_broadcasting = False
                self.process = True


            # Update elapsed time and previous time
            prev_time = elapsed_time
            elapsed_time = time.time() - start_time

            self.get_logger().warning('Going to Transform')
            time.sleep(0.2)  

    def approach_proccess(self, point_x, point_y, precision, position):
        
        # calculate orientation (positive right | negative left)
        yaw_1 = math.atan2(point_y - self.robot_position.transform.translation.y, point_x - self.robot_position.transform.translation.x) 

        # calculte error yaw
        err_yaw = yaw_1 - self.robot_tf_yaw

        # indicate position
        self.get_logger().info(position)

        # print the orientation     
        #self.get_logger().info("TF Yaw: %f" % yaw_1)
        #self.get_logger().info("Robot Yaw: %f" % self.robot_tf_yaw)
        self.get_logger().info("Error Yaw: %f" % err_yaw)

        # calculate the forward distance needed
        #self.get_logger().info("TF X : %f" % point_x)
        #self.get_logger().info("TF : %f" % point_y)
        needed_dist = math.sqrt(pow(point_y - self.robot_position.transform.translation.y, 2) + pow(point_x - self.robot_position.transform.translation.x, 2))
        self.get_logger().info("Distance: %f" % needed_dist)

        if needed_dist < precision:
            success = True
            #self.get_logger().info("Distances Reached")
            time.sleep(1)
        else:
            success = False

        if position == "Center Point" and needed_dist < 0.75 and math.fabs(err_yaw) < 0.02:
            self.get_logger().info("STOP Reading TF")
            self.read = True
            self.stop_adj_yaw = True
            twist_msg = Twist()
            twist_msg.angular.z = 0.00
            self._pub_cmd_vel.publish(twist_msg)

        return needed_dist, err_yaw, success


    def get_transform(self, target_frame, source_frame):
        when = self.get_clock().now() - rclpy.time.Duration(seconds=1.0)
        now = self.get_clock().now()
        zero_time = Time(seconds=0, nanoseconds=0)
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Could not obtain the transform: %s' % str(e))
            return None

        
    def get_position_from_tf(self, request, response):
        
        # get the indication of start broadcasting from service
        self.start_broadcasting = request.data
        if self.start_broadcasting: 
            self.get_logger().warning('Start Getting Transform')
            self.read = False
        else:
            self.get_logger().warning('Stop Getting Transform')

        while not self.process:
            self.get_logger().warning('Waiting')
            time.sleep(0.1)

        # return the result of the error
        response.success = self.process
        if response.success:
            response.message= 'Process Running Well'
            response.center_point.x = self.center_point.x 
            response.center_point.y = self.center_point.y 
            self.get_logger().warning('Approach Success')
        else:
            response.message= 'Process NOT Running Well'

        return response

    def calculate_yaw(self, q_x, q_y, q_z, q_w):
        # Calcular ángulo de yaw
        yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
        
        # Ajustar el ángulo para que esté en el rango [-pi, pi]
        if yaw > math.pi:
            yaw -= 2 * math.pi
        elif yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw

def main(args=None):
    rclpy.init(args=args)

    table_transform_publisher = TableTransformPublisher()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(table_transform_publisher, executor=executor)

    table_transform_publisher.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()