import rclpy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sklearn.cluster import KMeans
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
from itertools import permutations


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

        ''' STARTING FILTERING THE LASER DATA '''
        
        self.clustering()
        self.predict_cluster()
        self.calculate_cluster_error()
        self.print_data_matrix()
        self.count_cluster_repetitions()
        self.get_smaller_values_from_cluster(number_of_values=15)
        self.distances = self.calculate_distance_to_zero(coordinates= self.centroids, name_of_coordinates= 'Clusters')
        self.selected_points_with_distances_sorted()
        distances_average =np.mean(self.distances)
        print('\nDistance Average: ', distances_average)
        self.selected_points_with_distances_sorted_filtered(self.sorted_matrix_with_coord_dist, columna=4, valor_maximo=distances_average)
        self.filter_coordinates()
        self.legs_coordinates_with_no_reps = self.verify_close_points(self.array_final, threshold=0.3)
        self.leg_distances = self.calculate_distance_to_zero(self.legs_coordinates_with_no_reps, name_of_coordinates= 'Table Leg Distances')
        self.table_square = self.find_square(points=self.legs_coordinates_with_no_reps)

        if len(self.table_square) > 0:
            print('Square Coordinates Posible:', len(self.table_square))
            self.selected_table_square = np.array(self.table_square[0])
            print("Square Coordinate Selected:\n", self.selected_table_square)
            self.leg_middle_point = self.calculate_front_legs_center_point(leg_coordinates=self.legs_coordinates_with_no_reps)

            self.plot_data()
    
    def plot_data(self):

        fig, axs = plt.subplots(2, 3, figsize=(10, 5))  # Crea una figura con 1 fila y 2 columnas
        
        plt.subplots_adjust(wspace=0.4, hspace=0.6)
        fig.patch.set_facecolor('black')
        

        axs[0,0].scatter(self.data[:,0], self.data[:,1], c=self.kmeans.labels_.astype(float), s=50, label='Data Groups')
        axs[0,0].scatter(self.centroids[:,0], self.centroids[:,1], c='red', marker='*', s=50, label='Groups Centroids')  
        axs[0,0].legend()
        axs[0,0].set_title('Groups With Centroids', color='white')  
        axs[0,0].set_xlabel('X Coordinates', color='white')
        axs[0,0].set_ylabel('Y Coordinates', color='white')
        axs[0,0].set_xlim(-3, 3)
        axs[0,0].set_ylim(-0.5, 5)
        axs[0, 0].set_facecolor('black')
        axs[0, 0].grid(False)
        axs[0, 0].locator_params(axis='x', nbins=10)  
        axs[0, 0].locator_params(axis='y', nbins=10)  
        axs[0,0].spines['bottom'].set_color('white')  # Eje x
        axs[0,0].spines['left'].set_color('white')
        axs[0,0].tick_params(axis='x', colors='white', labelcolor='white') 
        axs[0,0].tick_params(axis='y', colors='white', labelcolor='white') 

        axs[0,1].scatter(self.datos_filtrados[:,2], self.datos_filtrados[:,3], c='blue', marker='o', s=50, label='Centroids from Cluster')  
        axs[0,1].scatter(self.array_final[:,0], self.array_final[:,1], c='red', marker='+', s=50, label='Filters Applied') 
        axs[0,1].legend()
        axs[0,1].set_title('Apply filters', color='white')  
        axs[0,1].set_xlabel('X Coordinates', color='white')
        axs[0,1].set_ylabel('Y Coordinates', color='white')
        axs[0,1].set_xlim(-3, 3)
        axs[0,1].set_ylim(-0.5, 5)
        axs[0,1].set_facecolor('black')
        axs[0,1].grid(False)
        axs[0,1].locator_params(axis='x', nbins=10)  
        axs[0,1].locator_params(axis='y', nbins=10)  
        axs[0,1].spines['bottom'].set_color('white')  # Eje x
        axs[0,1].spines['left'].set_color('white')
        axs[0,1].tick_params(axis='x', colors='white', labelcolor='white') 
        axs[0,1].tick_params(axis='y', colors='white', labelcolor='white') 
        # Agregar notas o anotaciones
        axs[0,1].text(-3, 1, 'Nota', fontsize=12)

        axs[0,2].scatter(self.legs_coordinates_with_no_reps[:,0], self.legs_coordinates_with_no_reps[:,1], c='green', marker='s',label='Table Legs')
        axs[0,2].scatter(self.leg_middle_point[0], self.leg_middle_point[1], c='red', marker='.', label='Front Legs Mid Point')
        axs[0,2].legend()
        axs[0,2].set_title('Legs Position Found', color='white')  
        axs[0,2].set_xlabel('X Coordinates', color='white')
        axs[0,2].set_ylabel('Y Coordinates', color='white')
        axs[0,2].set_xlim(-3, 3)
        axs[0,2].set_ylim(-0.5, 5)
        axs[0,2].set_facecolor('black')
        axs[0,2].grid(False)
        axs[0,2].locator_params(axis='x', nbins=10)  
        axs[0,2].locator_params(axis='y', nbins=10) 
        axs[0,2].spines['bottom'].set_color('white')  # Eje x
        axs[0,2].spines['left'].set_color('white')
        axs[0,2].tick_params(axis='x', colors='white', labelcolor='white') 
        axs[0,2].tick_params(axis='y', colors='white', labelcolor='white') 

        axs[1,0].plot(self.krango, self.sse, c='blue')
        axs[1,0].set_title('Sum of Square Error', color='white')
        axs[1,0].set_xlabel('K Value', color='white')
        axs[1,0].set_ylabel('Error', color='white')
        axs[1,0].set_facecolor('black')
        axs[1,0].grid(False)
        axs[1,0].spines['bottom'].set_color('white')  # Eje x
        axs[1,0].spines['left'].set_color('white')
        axs[1,0].tick_params(axis='x', colors='white', labelcolor='white') 
        axs[1,0].tick_params(axis='y', colors='white', labelcolor='white') 

        axs[1,1].stem(self.list_of_cluster_values, self.frecuencies, linefmt='b', markerfmt='ro')
        axs[1,1].set_title('Group Cluster Repetitions', color='white')
        axs[1,1].set_xlabel('Cluster Group', color='white')
        axs[1,1].set_ylabel('Cluster Repetition', color='white')
        axs[1,1].set_xlim(-1, len(self.list_of_cluster_values)+1)
        axs[1,1].set_ylim(0, 50)
        axs[1,1].set_facecolor('black')
        axs[1,1].grid(False)
        axs[1,1].spines['bottom'].set_color('white')  # Eje x
        axs[1,1].spines['left'].set_color('white')
        axs[1,1].tick_params(axis='x', colors='white', labelcolor='white') 
        axs[1,1].tick_params(axis='y', colors='white', labelcolor='white') 

        axs[1,2].set_title('Underneath Table Path', color='white')
        axs[1,2].scatter(self.selected_table_square[:,0], self.selected_table_square[:,1], c='red', marker='s',label='Table Legs Verificated')
        axs[1,2].legend()
        axs[1,2].set_xlim(-3, 3)
        axs[1,2].set_ylim(-0.5, 5)
        axs[1,2].set_xlabel('X Coordinates', color='white')
        axs[1,2].set_ylabel('Y Coordinates', color='white')
        axs[1,2].set_facecolor('black')
        axs[1,2].grid(False)
        axs[1,2].locator_params(axis='x', nbins=10)  
        axs[1,2].locator_params(axis='y', nbins=10) 
        axs[1,2].spines['bottom'].set_color('white')  # Eje x
        axs[1,2].spines['left'].set_color('white')
        axs[1,2].tick_params(axis='x', colors='white', labelcolor='white') 
        axs[1,2].tick_params(axis='y', colors='white', labelcolor='white') 

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
    
    # Need rework
    def selected_points_with_distances_sorted(self):
        matrix = np.column_stack((self.matrix_filtered_less, self.centroids, self.distances))
        # Obtener índices que ordenan la matriz según la tercera columna
        sorted_indices = np.argsort(matrix[:, 4])

        # Reorganizar la matriz según los índices obtenidos
        self.sorted_matrix_with_coord_dist = matrix[sorted_indices]
        print('\n Selected points with distances from 0.0\n')
        print('\n Group\t\t Reps\t X Coordinate\t Y Coordinate\t Distance\n', self.sorted_matrix_with_coord_dist)
    
    # Need rework
    def selected_points_with_distances_sorted_filtered(self, matrix, columna, valor_maximo):
        # Filtrar los datos según el valor máximo en la columna especificada
        self.datos_filtrados = matrix[matrix[:, columna] < valor_maximo]
        print('Selected points with distances from 0.0 filtered\n', self.datos_filtrados)
    
    # Need rework
    def euclidean_distances(self, point1, point2):
        return np.linalg.norm(point1 - point2)

    # Need rework
    def filter_coordinates(self):
        coordinates = np.column_stack((self.datos_filtrados[:,2], self.datos_filtrados[:,3]))
        print('\ncoordinates pre filtradas \n', coordinates)
        filtered_coordinates = []
        for point in coordinates:
            new_coordinate = True
            minor_distances = 0
            for filtered_point in filtered_coordinates:
                dist = self.euclidean_distances(point, filtered_point)
                if dist > 0.70:
                    minor_distances += 1
                if dist < 0.62:
                    minor_distances += 1
            if minor_distances > 2:
                new_coordinate = False
            if new_coordinate:
                filtered_coordinates.append(point)
        self.array_final = np.vstack(filtered_coordinates)
        print('\ncoordinates filtradas\n', self.array_final)

    def verify_close_points(self, coordinates, threshold):
        new_points = []
        close_points = set()
        for i in range(len(coordinates)):
            valid_point = True
            for j in range(i + 1, len(coordinates)):
                distance = self.euclidean_distances(coordinates[i], coordinates[j])
                if distance < threshold:
                    # print(f"Points {i} and {j} are too close.")
                    close_points.add(i)
                    close_points.add(j)
                    valid_point = False
            if valid_point:
                new_points.append(coordinates[i])
        new_points = np.vstack(new_points)
        print('\ncoordinates filtradas sin reps\n', new_points)
        return np.array(new_points)

    def find_square(self, points):
        possible_squares = []

        # Generate all possible permutations of points
        point_permutations = permutations(points, 4)  # We use only permutations of length 4

        # Iterate over the permutations
        for i, perm in enumerate(point_permutations, 1):
            combination = perm
                
            p1, p2, p3, p4 = combination

            # Calculate the distances of the sides
            sides = [
                self.euclidean_distances(p1, p2),
                self.euclidean_distances(p2, p3),
                self.euclidean_distances(p3, p4),
                self.euclidean_distances(p4, p1)
            ]

            # Calculate the diagonals
            diagonals = [
                self.euclidean_distances(p1, p3),
                self.euclidean_distances(p2, p4)
            ]

            # Check if the distances meet the criteria
            if all(0.60 <= distance <= 0.75 for distance in sides) and \
            all(0.90 <= diagonal <= 1.20 for diagonal in diagonals):
                possible_squares.append(combination)

        return possible_squares
    

    def calculate_front_legs_center_point(self, leg_coordinates):
        middle_point = [(leg_coordinates[0,0] + leg_coordinates[1,0]) / 2, (leg_coordinates[0,1] + leg_coordinates[1,1]) / 2]
        print('\nCalculated Middle Point: ', middle_point)
        return middle_point


def main(args=None):
    rclpy.init(args=args)

    trash_table_detection = TrashTableDetection()

    rclpy.spin(trash_table_detection)

    trash_table_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()