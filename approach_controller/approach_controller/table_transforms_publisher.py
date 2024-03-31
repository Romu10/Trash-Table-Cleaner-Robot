import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point
from std_srvs.srv import SetBool
from rclpy.time import Time
import math

class TableTransformPublisher(Node):
    def __init__(self):
        super().__init__('trash_table_transform_publisher')

        # define the tf broadcaster to publish tf
        #self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # define the service
        self.srv = self.create_service(SetBool, 'publish_table_frame_srv', self.publish_table_frame_srv)

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
        self.table_leg_1 = Point()
        self.odom_table_leg_1 = Point()
        self.map_table_leg_1 = Point()
        self.center_point = Point()
        self.odom_center_point = Point()
        self.approach_point = Point()
        self.odom_approach_point = Point()

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
        self.timer = self.create_timer(0.1, self.on_timer)
        
        self.get_logger().info('Table Transform Publisher Service Online...')
    
    def on_timer(self):

        if self.start_broadcasting:
            
            # get the transform position 
            while not self.read:
                self.leg_1_transform = self.get_transform(target_frame='leg_1', source_frame='robot_base_link')
                self.center_point_transform = self.get_transform(target_frame='table_center', source_frame='robot_base_link')
                self.approach_point_transform = self.get_transform(target_frame='approach_distance', source_frame='robot_base_link')
                self.robot_position = self.get_transform(target_frame='robot_base_link', source_frame='odom')
                self.odom_map = self.get_transform(target_frame='map', source_frame='odom')
                if self.leg_1_transform and self.robot_position and self.approach_point_transform and self.robot_position:
                    # indicate that the transfrom is running well
                    self.process = True
                    self.read = True
                    break

            # save table leg_1 position from robot_base_link
            self.table_leg_1.x = self.leg_1_transform.transform.translation.x * -1
            self.table_leg_1.y = self.leg_1_transform.transform.translation.y

            # save center point position from robot_base_link
            self.center_point.x = self.center_point_transform.transform.translation.x * -1
            self.center_point.y = self.center_point_transform.transform.translation.y

            # save approach point position from robot_base_link
            self.approach_point.x = self.approach_point_transform.transform.translation.x * -1 
            self.approach_point.y = self.approach_point_transform.transform.translation.y

            # calculate leg position in the odometry plane
            self.odom_table_leg_1.x = ((self.table_leg_1.x * -1) + self.robot_position.transform.translation.x) * -1
            self.odom_table_leg_1.y = self.table_leg_1.y - (self.robot_position.transform.translation.y) 

            # calculate center position in the odometry plane
            self.odom_center_point.x = ((self.center_point.x * -1) + self.robot_position.transform.translation.x) * -1
            self.odom_center_point.y = self.center_point.y - (self.robot_position.transform.translation.y) 

            # calculate approach positin in the odometry plane 
            self.odom_approach_point.x = ((self.approach_point.x * -1) + self.robot_position.transform.translation.x) * -1
            self.odom_approach_point.y = self.approach_point.y - (self.robot_position.transform.translation.y) 

            # calcula yaw error between odom and map
            rotation = self.odom_map.transform.rotation
            yaw = self.calculate_yaw(q_x=rotation.x, q_y=rotation.y, q_z=rotation.z, q_w=rotation.w)
            yaw = math.degrees(yaw)

            # extract odom to map pos offset
            offset_x = self.odom_map.transform.translation.x
            offset_y = self.odom_map.transform.translation.y

            # rotate in odom to the yaw the frame is rotate
            fixed_table_x, fixed_table_y = self.rotate_point(x=self.odom_table_leg_1.x, y=self.odom_table_leg_1.y, yaw=yaw)
            fixed_center_x, fixed_center_y = self.rotate_point(x=self.odom_center_point.x, y=self.odom_center_point.y, yaw=yaw)
            fixed_approach_x, fixed_approach_y = self.rotate_point(x=self.odom_approach_point.x, y=self.odom_approach_point.y, yaw=yaw)

            # publish first leg  transform
            self.publish_table_transform(frame='table_leg_1',
                                        source_frame='odom',
                                        y_coordinate= fixed_table_x,
                                        x_coordinate= -fixed_table_y)
            
            # publish first leg  transform
            self.publish_table_transform(frame='table_center_point',
                                        source_frame='odom',
                                        y_coordinate= fixed_center_x,
                                        x_coordinate= -fixed_center_y)

            # publish first leg  transform
            self.publish_table_transform(frame='table_approach_point',
                                        source_frame='odom',
                                        y_coordinate= fixed_approach_x,
                                        x_coordinate= -fixed_approach_y)

            self.get_logger().info("Leg X : %f" % self.odom_table_leg_1.x)
            self.get_logger().info("Leg Y : %f" % self.odom_table_leg_1.y)
            self.get_logger().info("Odom Yaw Offset: %f" % yaw)

            # indicate process is running
            self.get_logger().warning('Publishing Transform')


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

        
    def publish_table_frame_srv(self, request, response):
        
        # get the indication of start broadcasting from service
        self.start_broadcasting = request.data
        if self.start_broadcasting: 
            self.get_logger().warning('Start Publishing Transform')
            self.read = False
        else:
            self.get_logger().warning('Stop Publishing Transform')

        # return the result of the error
        response.success = self.process
        if response.success:
            response.message= 'Process Running Well'
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
    
    def rotate_point(self, x, y, yaw):
        theta = math.radians(yaw)        
        x_prime = x * math.cos(theta) - y * math.sin(theta)
        y_prime = x * math.sin(theta) + y * math.cos(theta)
        return x_prime, y_prime

def main(args=None):
    rclpy.init(args=args)

    table_transform_publisher = TableTransformPublisher()

    rclpy.spin(table_transform_publisher)

    table_transform_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()