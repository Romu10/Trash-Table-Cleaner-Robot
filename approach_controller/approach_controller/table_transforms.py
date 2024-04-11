import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point, Twist
from std_msgs.msg import Float32
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
    _dist_precision = 0.05

    def __init__(self):
        super().__init__('trash_table_transform_publisher')

        # define the tf broadcaster to publish tf
        #self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # define the service
        self.srv = self.create_service(TablePosition, 'get_position_tf', self.get_position_from_tf, 
                                                        callback_group=ReentrantCallbackGroup())

        # define a subsription for odom
        self.odom_subscription = self.create_subscription(Odometry, '/cleaner_2/odom', self.odom_callback, 10, 
                                                            callback_group=ReentrantCallbackGroup())

        # define a publisher for cmd
        self._pub_cmd_vel = self.create_publisher(Twist, 'cleaner_2/cmd_vel', 10)

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

        self._yaw = Float32()

        # save variables to compare error 
        self.verify_center_point = Point()

        # create flag for start broadcasting tfs 
        self.start_broadcasting = False

        # create flag to indicate that the error is small enought
        self.small_error = False

        # create flag to read transforms once
        self.read = False
        self.process = False

        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer, callback_group=ReentrantCallbackGroup())
        
        self.get_logger().info('Transform Position Service Online...')
    
    def on_timer(self):

        if self.start_broadcasting:
            
            # get the transform position 
            while not self.read:
                self.center_point_transform = self.get_transform(target_frame='table_center', source_frame='cleaner_2/base_link')
                self.approach_point_transform = self.get_transform(target_frame='approach_distance', source_frame='cleaner_2/base_link')
                if self.center_point_transform and self.approach_point_transform:
                    # indicate that the transfrom is running well
                    self.process = True
                    #self.read = True
                    break

            # save center point position from base_link
            self.center_point.x = self.center_point_transform.transform.translation.x
            self.center_point.y = self.center_point_transform.transform.translation.y

            # save approach point position from base_link
            self.approach_point.x = self.approach_point_transform.transform.translation.x 
            self.approach_point.y = self.approach_point_transform.transform.translation.y

            # calculate orientation (positive right | negative left)
            yaw_1 = math.atan2(self.approach_point.y - 0, self.approach_point.x - 0) 
            
            # aling table point orientation with robot odom
            if yaw_1 < 0:
                yaw_2 = yaw_1 + math.pi
            else: 
                yaw_2 = yaw_1 - math.pi
            
            # calculate tf yaw respect the robot
            yaw = self._yaw.data + yaw_2

            # calculte error yaw
            err_yaw = math.fabs(yaw) - math.fabs(self._yaw.data)
 
            # print the orientation     
            self.get_logger().info("Pre TF Yaw: %f" % yaw_1)
            self.get_logger().info("Post TF Yaw: %f" % yaw_2)
            self.get_logger().info("Robot TF Yaw: %f" % yaw)
            self.get_logger().info("Robot Yaw: %f" % self._yaw.data)
            self.get_logger().info("Error Yaw: %f" % err_yaw)

            # calculate the forward distance needed
            self.get_logger().info("X : %f" % self.approach_point.x)
            self.get_logger().info("Y : %f" % self.approach_point.y)
            needed_dist = math.sqrt(pow(self.approach_point.y - 0, 2) + pow(self.approach_point.x - 0, 2))
            self.get_logger().info("Distance: %f" % needed_dist)
            
  
            # confirm the distance
            if needed_dist < self._dist_precision:
                self.read = True
                self.start_broadcasting = False

            if math.fabs(err_yaw) > self._yaw_precision:
                twist_msg = Twist()
                twist_msg.angular.z = 0.0087
                twist_msg.angular.z = -0.0087 if err_yaw > 0 else 0.0087
                self._pub_cmd_vel.publish(twist_msg)
                self.get_logger().info("Need Yaw")


            else:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0235
                self._pub_cmd_vel.publish(twist_msg)
                self.get_logger().info("Need Distance")
            
            # indicate process is running
            
            # end cycle
            # self.start_broadcasting = False

            self.get_logger().warning('Going to Transform')
            time.sleep(0.2)


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
            self.get_logger().warning('Start Publishing Transform')
            self.read = False
        else:
            self.get_logger().warning('Stop Publishing Transform')

        while not self.process:
            self.get_logger().warning('Waiting')

        # return the result of the error
        response.success = self.process
        if response.success:
            response.message= 'Process Running Well'
            response.center_point.x = self.center_point.x 
            response.center_point.y = self.center_point.y 
            self.get_logger().warning('Success')
        else:
            response.message= 'Process NOT Running Well'

        return response

    def publish_table_transform(self, frame, source_frame, x_coordinate, y_coordinate):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = source_frame
        tf_msg.child_frame_id = frame
        tf_msg.transform.translation.y = x_coordinate
        tf_msg.transform.translation.x = y_coordinate
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

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
        self._yaw.data = self.calculate_yaw(q_x=q_x, q_y=q_y, q_z=q_z, q_w=q_w)
        #self.get_logger().info("Direct Odometry Yaw: %f" % self._yaw.data)

        

    def rotate_point(self, x, y, yaw):
        theta = math.radians(yaw)        
        x_prime = x * math.cos(theta) - y * math.sin(theta)
        y_prime = x * math.sin(theta) + y * math.cos(theta)
        return x_prime, y_prime

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