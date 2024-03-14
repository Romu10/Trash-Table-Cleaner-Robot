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
        self.k = 15

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
        self.count_cluster_repetitions()
        self.get_smaller_values_from_cluster(number_of_values=15)
        self.distances = self.calculate_distance_to_zero(coordinates= self.centroids, name_of_coordinates= 'Clusters')
        self.selected_points_with_distances_sorted()
        self.selected_points_with_distances_sorted_filtered(self.sorted_matrix_with_coord_dist, columna=4, valor_maximo=3.0)
        self.filtrarCoordenadas()  #
        self.leg_distances = self.calculate_distance_to_zero(self.array_final,name_of_coordinates= 'Table Leg Distances')
        self.plot_data()
    
    def plot_data(self):

        fig, axs = plt.subplots(2, 3, figsize=(10, 5))  # Crea una figura con 1 fila y 2 columnas
        
        plt.subplots_adjust(wspace=0.4, hspace=0.6)

        axs[0,0].scatter(self.data[:,0], self.data[:,1], c=self.kmeans.labels_.astype(float), s=50)
        axs[0,0].scatter(self.centroids[:,0], self.centroids[:,1], c='red', marker='*', s=50)  
        axs[0,0].set_title('Groups With Centroids')  
        axs[0,0].set_xlabel('X Coordinates')
        axs[0,0].set_ylabel('Y Coordinates')
        axs[0,0].set_xlim(-3, 3)
        axs[0,0].set_ylim(-0.5, 5)

        axs[0,1].scatter(self.datos_filtrados[:,2], self.datos_filtrados[:,3], c='blue', marker='o', s=50)  
        axs[0,1].scatter(self.array_final[:,0], self.array_final[:,1], c='orange', marker='+', s=50)  
        axs[0,1].set_title('Apply filters')  
        axs[0,1].set_xlabel('X Coordinates')
        axs[0,1].set_ylabel('Y Coordinates')
        axs[0,1].set_xlim(-3, 3)
        axs[0,1].set_ylim(-0.5, 5)

        axs[0,2].scatter(self.array_final[:,0], self.array_final[:,1], c='green', marker='s')
        axs[0,2].set_title('Legs Position Found')  
        axs[0,2].set_xlabel('X Coordinates')
        axs[0,2].set_ylabel('Y Coordinates')
        axs[0,2].set_xlim(-3, 3)
        axs[0,2].set_ylim(-0.5, 4)

        axs[1,0].plot(self.krango, self.sse, c='black')
        axs[1,0].set_title('Sum of Square Error')
        axs[1,0].set_xlabel('K Value')
        axs[1,0].set_ylabel('Error')

        axs[1,1].stem(self.list_of_cluster_values, self.frecuencies, linefmt='k-', markerfmt='ro')
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
        # print('\nLaser Data with Cluster\n')
        # print(self.data_with_cluster)

    def count_cluster_repetitions(self):
        self.frecuencies = np.bincount(self.clust)
        self.cluster_values_for_plot = range(len(self.frecuencies))
        self.list_of_cluster_values = list(self.cluster_values_for_plot)
        self.cluster_w_repetitions_num = []

        print('\nCluster repetitions\n', self.frecuencies)
        for numero, frecuencia in enumerate(self.frecuencies):
            if frecuencia > 0 and frecuencia < 40:
                self.cluster_w_repetitions_num.append([numero, frecuencia])

        print('\nMatrix with cluster reps\n', self.cluster_w_repetitions_num)

    def get_smaller_values_from_cluster(self, number_of_values):
        data_matrix_sorted = sorted(self.cluster_w_repetitions_num, key=lambda x: x[1], reverse=True)
        filtered_sorted_values = data_matrix_sorted[:number_of_values]
        shaped_matrix = np.array(filtered_sorted_values)
        self.matrix_filtered_less = shaped_matrix.reshape(-1, 2)
        print('\nFiltered values\n', self.matrix_filtered_less)
    
    def calculate_distance_to_zero(self, coordinates, name_of_coordinates):
        distances = np.linalg.norm(coordinates, axis=1)
        distances = distances.reshape(-1, 1)
        print('\nDistance from zero for selected points: ' + name_of_coordinates + '\n', distances)
        return distances
    
    def selected_points_with_distances_sorted(self):
        matrix = np.column_stack((self.matrix_filtered_less, self.centroids, self.distances))
        # Obtener índices que ordenan la matriz según la tercera columna
        sorted_indices = np.argsort(matrix[:, 4])

        # Reorganizar la matriz según los índices obtenidos
        self.sorted_matrix_with_coord_dist = matrix[sorted_indices]
        print('\n Selected points with distances from 0.0\n')
        print('\n Group\t\t Reps\t X Coordinate\t Y Coordinate\t Distance\n', self.sorted_matrix_with_coord_dist)
    
    def selected_points_with_distances_sorted_filtered(self, matrix, columna, valor_maximo):
        # Filtrar los datos según el valor máximo en la columna especificada
        self.datos_filtrados = matrix[matrix[:, columna] < valor_maximo]
        print('Selected points with distances from 0.0 filtered\n', self.datos_filtrados)
    
    def distancia(self, punto1, punto2):
        return np.linalg.norm(punto1 - punto2)

    def filtrarCoordenadas(self):
        Umbral_de_distancia = 0.55  # Ajusta este valor según sea necesario
        coordenadas = np.column_stack((self.datos_filtrados[:7,2], self.datos_filtrados[:7,3]))
        print('\nCoordenadas pre filtradas \n', coordenadas)
        coordenadas_filtradas = []
        for punto in coordenadas:
            es_nueva_coordenada = True
            for punto_filtrado in coordenadas_filtradas:
                # print(self.distancia(punto, punto_filtrado))
                if self.distancia(punto, punto_filtrado) > 0.35 and self.distancia(punto, punto_filtrado) < 0.60:
                    es_nueva_coordenada = False
                    break
            if es_nueva_coordenada:
                coordenadas_filtradas.append(punto)
        self.array_final = np.vstack(coordenadas_filtradas)

        print('\nCoordenadas filtradas\n', self.array_final)

    


def main(args=None):
    rclpy.init(args=args)

    trash_table_detection = TrashTableDetection()

    rclpy.spin(trash_table_detection)

    trash_table_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()