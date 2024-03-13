import rclpy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class TrashTableDetection(Node):

    def __init__(self):
        super().__init__('trash_table_detection')
        
        # variable to storage the laser filtered data
        self.laser_data = []
        self.list_of_laser_values = []

        # define a ros subscription
        self.subscription = self.create_subscription(LaserScan, 'table_scan_filtered', self.laser_callback, 10)

    # laser callback function
    def laser_callback(self, msg):

        # receive the laser filtered data
        self.laser_data = msg.ranges
        print(self.laser_data)

        # count the number of data received 
        laser_quantity = len(self.laser_data)

        # create the numbers of value for plotting
        laser_received = range(laser_quantity)

        # list of lasers
        self.list_of_laser_values = list(laser_received)

        # Calculate x, y coordinates for each laser range measurement
        angles = np.arange(laser_quantity) * msg.angle_increment + msg.angle_min
        x_coordinates = np.multiply(self.laser_data, np.cos(angles))
        y_coordinates = np.multiply(self.laser_data, np.sin(angles))

        # plot data distribution
        # plt.scatter(self.list_of_laser_values, self.laser_data)

        # Agregar un círculo estático en las coordenadas (0.5, 0.5) con radio 0.1
        # circle = plt.Circle((0.0, -1.0), 0.3, color='r', fill=True)
        # plt.gca().add_patch(circle)

        # plot coordinates data
        plt.scatter(y_coordinates, x_coordinates)
        
        # Set the labels for the axes
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')

        # Set the limits for the axes
        plt.xlim(-3, 3)
        plt.ylim(-1, 5)
        plt.show()

def main(args=None):
    rclpy.init(args=args)

    trash_table_detection = TrashTableDetection()

    rclpy.spin(trash_table_detection)

    trash_table_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()