import rclpy 
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from detection_interfaces.srv import DetectTableLegs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from detection_interfaces.action import ApproachTable
from rclpy.action import ActionServer
from geometry_msgs.msg import Point, Twist
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, kp, ki, kd, min_output, max_output):
        self.kp = kp  # Constante proporcional
        self.ki = ki  # Constante integral
        self.kd = kd  # Constante derivativa
        self.min_output = min_output  # Valor mínimo de salida
        self.max_output = max_output  # Valor máximo de salida
        self.prev_error = 0.0  # Error previo
        self.integral = 0.0  # Término integral

    def calculate(self, error, dt):
        # Término proporcional
        proportional = self.kp * error

        # Término integral
        self.integral += error * dt
        integral = self.ki * self.integral

        # Término derivativo
        derivative = self.kd * (error - self.prev_error) / dt

        # Actualizar el error previo
        self.prev_error = error

        # Calcular la salida del control PID
        output = proportional + integral + derivative

        # Limitar la salida dentro del rango permitido
        output = max(min(output, self.max_output), self.min_output)

        return output

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
    _yaw_precision = 0.02 # +/- 2 degree allowed
    _dist_precision = 0.05
    _robot_radius = 0.30

    def __init__(self):
        super().__init__('approach_controller')

        # define a subscription for laser scan
        #self.laserscan_subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

        # define a subsription for odom
        self.odom_subscription = self.create_subscription(Odometry, '/diffbot_base_controller/odom', self.odom_callback, 10, 
                                            callback_group=ReentrantCallbackGroup())

        # define a publisher for cmd
        self._pub_cmd_vel = self.create_publisher(Twist, 'diffbot_base_controller/cmd_vel_unstamped', 10)

        # create an action service
        self._action_server = ActionServer(self, ApproachTable, 'approach_table_as',  
                                            execute_callback=self. execute_callback,
                                            callback_group=ReentrantCallbackGroup(),
                                            goal_callback=self.goal_callback,
                                            cancel_callback=self.cancel_callback)

        self.get_logger().info('Approach Controller Action Service Online...')

        # Angular Vel PID Control params
        kp_ang = 1.60  # proportional gain
        ki_ang = 0.007  # integral gain
        kd_ang = 1.20  # derivative gain 
        min_output_ang = -0.6  # min output value for vel 
        max_output_ang = 0.6  # max output value for vel

        # Lineal Vel Control PID Control params
        kp_lin = 1.0  # proportional gain
        ki_lin = 0.005  # integral gain
        kd_lin = 0.1  # derivative gain 
        min_output_lin = 0.0  # min output value for vel 
        max_output_lin = 0.5  # max output value for vel

        # Crear el controlador PID para velocidad angular
        self.pid_controller_ang = PIDController(kp_ang, ki_ang, kd_ang, min_output_ang, max_output_ang)
        self.pid_controller_lin = PIDController(kp_lin, ki_lin, kd_lin, min_output_lin, max_output_lin)

        # Inicializar listas para almacenar los datos
        self.desired_yaw_list = []
        self.actual_yaw_list = []
        self.time_list = []

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()
        
    def  execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # helper variables
        success = True

        # define desired position and errors
        self._des_pos = goal_handle.request.goal_position
        self.get_logger().info("Point X position: %f" % self._des_pos.x)
        self.get_logger().info("Point Y position: %f" % self._des_pos.y)

        # Guardar temporalmente las coordenadas del punto de destino
        des_pos_x = self._des_pos.x
        des_pos_y = self._des_pos.y

        # Actualizar las coordenadas del punto de destino con las coordenadas relativas al robot
        self._des_pos.x = self._position.x + des_pos_y
        self._des_pos.y = self._position.y + des_pos_x 

        self.get_logger().info("Current X position: %f" % self._position.x)
        self.get_logger().info("Current Y position: %f" % self._position.y)
        self.get_logger().info("Waypoint X position: %f" % self._des_pos.x)
        self.get_logger().info("Waypoint Y position: %f" % self._des_pos.y)

        desired_yaw = math.atan2(self._des_pos.x - self._position.x, self._des_pos.y - self._position.y)

        self.get_logger().info("Desired Yaw: %f" % desired_yaw)
        self.get_logger().info("Current Yaw: %f" % self._yaw)

        self.desired_yaw_list.append(desired_yaw)
        err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
        err_yaw = desired_yaw - self._yaw
        self.get_logger().info("Error Pos: %f" % err_pos)
        self.get_logger().info("Error Yaw: %f" % err_yaw)
        time.sleep(10)

        rot_vel = 0.0

        prev_time = None
        elapsed_time = 0
        start_time = time.time()
        
        # perform task
        while success:
            # Update variables
            desired_yaw = math.atan2(self._des_pos.x - self._position.x, self._des_pos.y - self._position.y)
            
            err_yaw = desired_yaw - self._yaw
            err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))

            # Check if the position error is within the precision threshold
            if err_pos < self._dist_precision:
                break  # Exit the loop if the position error is within the precision threshold

            # Calculate control commands using PID
            if prev_time is not None:  
                dt = elapsed_time - prev_time
            else:
                dt = 0.00001

            # Print variable status
            #self.get_logger().info("Current Yaw: %s" % str(self._yaw))
            #self.get_logger().info("Desired Yaw: %s" % str(desired_yaw))
            self.get_logger().info("Error Yaw: %s" % str(err_yaw))
            self.get_logger().info("Error Pos: %s" % str(err_pos))
            #self.get_logger().info("X Position: %f" % self._position.x)
            #self.get_logger().info("Y Position: %f" % self._position.y)

            # Logic goes here
            if goal_handle.is_cancel_requested:
                # Cancelled
                self.get_logger().info("The goal has been cancelled/preempted")
                goal_handle.abort()
                success = False
            elif math.fabs(err_yaw) > self._yaw_precision:
                rot_vel = self.pid_controller_ang.calculate(err_yaw, dt)
                # fix yaw
                #self.get_logger().info("fix yaw")
                self._state = 'fix yaw'
                twist_msg = Twist()
                twist_msg.angular.z = rot_vel
                self._pub_cmd_vel.publish(twist_msg)

            else:
                rot_vel = self.pid_controller_ang.calculate(err_yaw, dt)
                lin_vel = self.pid_controller_lin.calculate(err_pos, dt)
                # Go to point
                self.get_logger().info("going to point: %s" % str(goal_handle.request.goal_name))
                self._state = goal_handle.request.goal_name
                twist_msg = Twist()
                twist_msg.linear.x = lin_vel
                twist_msg.angular.z = rot_vel
                self._pub_cmd_vel.publish(twist_msg)

            # Use this to send feedback
            feedback_msg = ApproachTable.Feedback()
            feedback_msg.state = goal_handle.request.goal_name
            feedback_msg.current_position = self._position

            # Update elapsed time and previous time
            prev_time = elapsed_time
            elapsed_time = time.time() - start_time
            
            # update plot variables
            #self.actual_yaw_list.append(self._yaw)
            #self.time_list.append(elapsed_time)
            
            
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
            self.get_logger().info("goal reached")

        else:
            # use this to indicate that the goal failed
            goal_handle.failed()
            result = ApproachTable.Result()
            result.success = False
            self.get_logger().info("goal failed")

        # Graph PID 
        # create a time list with same yaw list lenght 
        # time_list = range(len(self.actual_yaw_list))

        # Desired Yaw for graphs
        # desired_yaw_constant = [desired_yaw] * len(self.actual_yaw_list)

        '''
        plt.plot(self.time_list, desired_yaw_constant, label='Desired Yaw (Constant)')
        plt.plot(self.time_list, self.actual_yaw_list, label='Actual Yaw')
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw')
        plt.title('PID Control Behavior')
        plt.legend()
        plt.show()
        '''
        return result

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
        self._yaw = round(self.calculate_yaw(q_x=q_x, q_y=q_y, q_z=q_z, q_w=q_w), 3)

        #self.get_logger().info("Yaw: %f" % self._yaw)

    
    def calculate_path_point_orientation(self, x_point, y_point, init_pos_x, init_pos_y, theta_robot):
        
        # calculate relative coordinates of two points
        x_rel = x_point - init_pos_x
        y_rel = y_point - init_pos_y

        # calculate orientation of point from the robot
        theta_point = math.atan2(y_rel, x_rel) - theta_robot

        # make sure the angle is in range -pi to pi
        if theta_point > math.pi:
            theta_point -= 2 * math.pi
        elif theta_point < -math.pi:
            theta_point += 2 * math.pi

        return theta_point

def main(args=None):
    rclpy.init(args=args)

    approach_controller = ApproachController()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(approach_controller, executor=executor)

    approach_controller.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()