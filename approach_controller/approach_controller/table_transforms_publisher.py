import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point
from std_srvs.srv import SetBool
from rclpy.time import Time

class TableTransformPublisher(Node):
    def __init__(self):
        super().__init__('trash_table_transform_publisher')

        # define the tf broadcaster to publish tf
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # define the service
        self.srv = self.create_service(SetBool, 'publish_table_frame_srv', self.publish_table_frame_srv)

        # define tf listener 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # initialize some needed vars
        self.publish_flag = False
        self.published_flag = False

        # save table positions from robot_base_link
        self.table_leg_1 = Point()
        self.odom_table_leg_1 = Point()
        self.center_point = Point()
        self.odom_center_point = Point()
        self.approach_point = Point()
        self.odom_approach_point = Point()

        # save variables to compare error 
        self.verify_center_point = Point()
        
        self.get_logger().info('Table Transform Publisher Service Online...')

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
       
        if request.data:
            
            # get the transform position 
            while True:
                leg_1_transform = self.get_transform(target_frame='leg_1', source_frame='robot_base_link')
                center_point_transform = self.get_transform(target_frame='table_center', source_frame='robot_base_link')
                approach_point_transform = self.get_transform(target_frame='approach_distance', source_frame='robot_base_link')
                robot_position = self.get_transform(target_frame='robot_base_link', source_frame='odom')
                if leg_1_transform and robot_position and approach_point_transform and robot_position:
                    break

            # save table leg_1 position from robot_base_link
            self.table_leg_1.x = leg_1_transform.transform.translation.x * -1
            self.table_leg_1.y = leg_1_transform.transform.translation.y

            # save center point position from robot_base_link
            self.center_point.x = center_point_transform.transform.translation.x * -1
            self.center_point.y = center_point_transform.transform.translation.y

            # save approach point position from robot_base_link
            self.approach_point.x = approach_point_transform.transform.translation.x * -1 
            self.approach_point.y = approach_point_transform.transform.translation.y

            # calculate leg position in the odometry plane
            self.odom_table_leg_1.x = ((self.table_leg_1.x * -1) + robot_position.transform.translation.x) * -1
            self.odom_table_leg_1.y = self.table_leg_1.y - (robot_position.transform.translation.y) 

            # calculate center position in the odometry plane
            self.odom_center_point.x = ((self.center_point.x * -1) + robot_position.transform.translation.x) * -1
            self.odom_center_point.y = self.center_point.y - (robot_position.transform.translation.y) 

            # calculate approach positin in the odometry plane 
            self.odom_approach_point.x = ((self.approach_point.x * -1) + robot_position.transform.translation.x) * -1
            self.odom_approach_point.y = self.approach_point.y - (robot_position.transform.translation.y) 

            # evaluate odometry error for publishing the correct position of the table
            #err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))


            # publish first leg static transform
            self.publish_table_transform(frame='table_leg_1',
                                        source_frame='odom',
                                        y_coordinate= self.odom_table_leg_1.x,
                                        x_coordinate= self.odom_table_leg_1.y)
            
            # publish first leg static transform
            self.publish_table_transform(frame='table_center_point',
                                        source_frame='odom',
                                        y_coordinate= self.odom_center_point.x,
                                        x_coordinate= self.odom_center_point.y)

            # publish first leg static transform
            self.publish_table_transform(frame='table_approach_point',
                                        source_frame='odom',
                                        y_coordinate= self.odom_approach_point.x,
                                        x_coordinate= self.odom_approach_point.y)
        
        response.success = True
        self.get_logger().warning('Received Transform')

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


def main(args=None):
    rclpy.init(args=args)

    table_transform_publisher = TableTransformPublisher()

    rclpy.spin(table_transform_publisher)

    table_transform_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()