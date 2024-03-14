import rclpy
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sklearn.cluster import KMeans
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
import itertools

class TrashTableDetection(Node):

    def __init__(self):
        super().__init__('trash_table_detection')
        
        # variable to storage the laser filtered data
        self.laser_data = []
        self.list_of_laser_values = []

        # define k for clustering
        self.k = 20

        # define a ros subscription
        self.subscription = self.create_subscription(LaserScan, 'table_scan_filtered', self.laser_callback, 10)

    # laser callback function
    def laser_callback(self, msg):

        # receive the laser filtered data
        self.laser_data = msg.ranges
        # print(self.laser_data)

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
        # print('\nLaser Data\n', self.data)
        # print('Data lenght: %i' % len(self.data))

        self.clustering()
        self.predict_cluster()
        self.calculate_cluster_error()
        self.print_data_matrix()
        self.process_data_matrix()
        self.count_cluster_repetitions()
        self.get_smaller_values_from_cluster(15)
        self.extrac_selection_from_process_matrix()
        self.plot_data()
    
    def plot_data(self):

        fig, axs = plt.subplots(2, 2, figsize=(10, 5))  # Crea una figura con 1 fila y 2 columnas

        # Scatter plot para el primer subplot
        axs[0,0].scatter(self.data[:,0], self.data[:,1], c=self.kmeans.labels_.astype(float), s=50)
        axs[0,0].scatter(self.centroids[:,0], self.centroids[:,1], c='red', marker='*', s=50)  
        axs[0,0].set_title('Scatter Plot with Centroides')  
        axs[0,0].set_xlabel('X Coordinates')
        axs[0,0].set_ylabel('Y Coordinates')
        axs[0,0].set_xlim(-3, 3)
        axs[0,0].set_ylim(-0.5, 5)

        # Scatter plot para el primer subplot
        axs[0,1].scatter(self.matrix_without_reps[:,0], self.matrix_without_reps[:,1], s=50)
        axs[0,1].scatter(self.centroids[:,0], self.centroids[:,1], c='red', marker='*', s=50)
        axs[0,1].scatter(self.selected_points[:,0], self.selected_points[:,1], c='green', marker='*', s=50)  
        axs[0,1].set_title('Scatter Plot Filt with Centroides')  
        axs[0,1].set_xlabel('X Coordinates')
        axs[0,1].set_ylabel('Y Coordinates')
        axs[0,1].set_xlim(-3, 3)
        axs[0,1].set_ylim(-0.5, 5)

        # Plot the second set of data on the second subplot
        axs[1,0].plot(self.krango, self.sse)
        axs[1,0].set_title('Sum of Square Error')
        axs[1,0].set_xlabel('Error')
        axs[1,0].set_ylabel('K value')

        # Plot the third set of data on the third subplot
        #axs[1,1].scatter(self.list_of_cluster_values, self.frecuencies, s=50)
        axs[1,1].stem(self.list_of_cluster_values, self.frecuencies)
        axs[1,1].set_title('Group Cluster Repetitions')
        axs[1,1].set_xlabel('Cluster Group')
        axs[1,1].set_ylabel('Cluster Repetition')
        axs[1,1].set_xlim(-1, len(self.list_of_cluster_values)+1)
        axs[1,1].set_ylim(0, 50)

        # Show the plots
        plt.show()
      
    def clustering(self):
        self.kmeans = KMeans(n_clusters=self.k).fit(self.data)
        self.centroids = self.kmeans.cluster_centers_
        print('\nCentroids\n', self.centroids)
        print('Data lenght: %i' % len(self.centroids))
    
    def predict_cluster(self):
        self.clust = self.kmeans.predict(self.data)
        print('\nCluster predicted\n')
        print(self.clust)
        print('Data lenght: %i' % len(self.clust))
    
    def calculate_cluster_error(self):
        self.krango = range(1,self.k)
        self.sse=[]
        for k in self.krango:
            kmeans = KMeans(n_clusters=k).fit(self.data)
            self.sse.append(kmeans.inertia_)
        print('\nCluster errors for K = %i to K = 0\n' % self.k)
        print(self.sse)
    
    def print_data_matrix(self): 
        self.data_with_cluster = np.column_stack((self.y_coordinates, self.x_coordinates, self.clust))
        print('\nLaser Data with Cluster\n')
        print(self.data_with_cluster)

    def count_cluster_repetitions(self):
        self.frecuencies = np.bincount(self.clust)
        self.cluster_values_for_plot = range(len(self.frecuencies))
        self.list_of_cluster_values = list(self.cluster_values_for_plot)
        self.cluster_w_repetitions_num = []

        print('\nCluster repetitions\n', self.frecuencies)
        for numero, frecuencia in enumerate(self.frecuencies):
            if frecuencia > 0 and frecuencia < 30:
                self.cluster_w_repetitions_num.append([numero, frecuencia])

        print('\nMatrix with cluster reps\n', self.cluster_w_repetitions_num)

    def get_smaller_values_from_cluster(self, number_of_values):
        data_matrix_sorted = sorted(self.cluster_w_repetitions_num, key=lambda x: x[1])
        filtered_sorted_values = data_matrix_sorted[:number_of_values]
        shaped_matrix = np.array(filtered_sorted_values)
        self.matrix_filtered_less = shaped_matrix.reshape(-1, 2)
        print('\nFiltered values\n', self.matrix_filtered_less)

    def process_data_matrix(self):
        unique_values = set()
        filtered_matrix = []

        for fila in self.data_with_cluster:
            if fila[2] not in unique_values:
                filtered_matrix.append(fila)
                unique_values.add(fila[2])

        # convert to 3xn matrix
        self.matrix_without_reps = np.array(filtered_matrix)
        print('\nLaser Data Filtered without reps\n', self.matrix_without_reps)
    
    def extrac_selection_from_process_matrix(self):
        # Extraer valores de la primera y segunda columna según los índices de la tercera columna
        self.selected_points = self.matrix_without_reps[self.matrix_filtered_less[:, 0], :2]

        print("\nSelected points\n")
        print(self.selected_points)

def main(args=None):
    rclpy.init(args=args)

    trash_table_detection = TrashTableDetection()

    rclpy.spin(trash_table_detection)

    trash_table_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()