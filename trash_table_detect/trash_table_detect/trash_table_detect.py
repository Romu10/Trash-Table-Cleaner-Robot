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
        self.k = 10

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

        self.clustering()
        self.calculate_cluster_error()
        self.plot_data()
    
    def plot_data(self):

        fig, axs = plt.subplots(1, 2, figsize=(10, 5))  # Crea una figura con 1 fila y 2 columnas

        # Scatter plot para el primer subplot
        axs[0].scatter(self.data[:,0], self.data[:,1], c=self.kmeans.labels_.astype(float), s=50)
        axs[0].scatter(self.centroids[:,0], self.centroids[:,1], c='red', marker='*', s=50)  # Añade los centroides al mismo subplot
        axs[0].set_title('Scatter Plot con Centroides')  # Título del primer subplota
        axs[0].set_xlabel('X Coordinates')
        axs[0].set_ylabel('Y Coordinates')
        axs[0].set_xlim(-3, 3)
        axs[0].set_ylim(-0.5, 5)

        # Plot the second set of data on the second subplot
        axs[1].plot(self.krango, self.sse)
        axs[1].set_title('Sum of Square Error')
        axs[1].set_xlabel('Error')
        axs[1].set_ylabel('K value')

        # Show the plots
        plt.show()
      
    
    def clustering(self):
        self.kmeans = KMeans(n_clusters=self.k).fit(self.data)
        self.centroids = self.kmeans.cluster_centers_
        print('\nCentroids: \n', self.centroids)

    
    def calculate_cluster_error(self):
        self.krango = range(1,self.k)
        self.sse=[]
        for k in self.krango:
            kmeans = KMeans(n_clusters=k).fit(self.data)
            self.sse.append(kmeans.inertia_)
        print('\nCluster errors for K = %i to K = 0' % self.k)
        print(self.sse)

def main(args=None):
    rclpy.init(args=args)

    trash_table_detection = TrashTableDetection()

    rclpy.spin(trash_table_detection)

    trash_table_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()