import rclpy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sklearn.cluster import KMeans
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class TrashTableDetection(Node):

    def __init__(self):
        super().__init__('trash_table_detection')
        
        # variable to storage the laser filtered data
        self.laser_data = []
        self.list_of_laser_values = []

        # define k for clustering
        self.k = 15

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
        self.x_coordinates = np.multiply(self.laser_data, np.cos(angles))
        self.y_coordinates = np.multiply(self.laser_data, np.sin(angles))

        # merge data
        self.data = np.column_stack((self.y_coordinates, self.x_coordinates))

        self.plot_data()
    
    def plot_data(self):
        # Agregar un círculo estático en las coordenadas (0.5, 0.5) con radio 0.1
        # circle = plt.Circle((0.0, -1.0), 0.3, color='r', fill=True)
        # plt.gca().add_patch(circle)

        # merge data
        # self.data = np.column_stack((self.y_coordinates, self.x_coordinates))

        # plot coordinates data
        #plt.scatter(self.data[:,0],self.data[:,1])
        
        # Set the labels for the axes
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')

        # Set the limits for the axes
        plt.xlim(-3, 3)
        plt.ylim(-1, 5)

        self.clustering()
        
    
    def clustering(self):
        kmeans = KMeans(n_clusters=self.k).fit(self.data)
        centroids = kmeans.cluster_centers_
        print(centroids)
        #plt.scatter(self.data[:,0], self.data[:,1], c=kmeans.labels_.astype(float), s=50)
        #plt.scatter(centroids[:,0], centroids[:,1], c='red', marker='*', s=50)
        #plt.show()
        self.calculate_cluster_error()
    
    def calculate_cluster_error(self):
        # Calculate la metrica SSE Sum of Square Error to differents k
        krango = range(1,10)
        sse=[]
        for k in krango:
            kmeans = KMeans(n_clusters=k).fit(self.data)
            sse.append(kmeans.inertia_)
        print(sse)
        plt.xlabel('K')
        plt.ylabel('SSE')
        plt.xlim(0, 10)
        plt.ylim(0, 150)
        plt.plot(krango,sse)
        plt.show()

    

def main(args=None):
    rclpy.init(args=args)

    trash_table_detection = TrashTableDetection()

    rclpy.spin(trash_table_detection)

    trash_table_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()