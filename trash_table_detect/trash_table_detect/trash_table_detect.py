import rclpy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sklearn.cluster import KMeans
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from itertools import permutations
import math


class TrashTableDetection(Node):

    def __init__(self):
        super().__init__('trash_table_detection')
        
        # variable to storage the laser filtered data
        self.laser_data = []
        self.list_of_laser_values = []

        # define k number of groups for clustering
        self.k = 15

        # define a ros subscription
        self.subscription = self.create_subscription(LaserScan, 'table_scan_filtered', self.laser_callback, 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

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
        # start data clustering process
        self.clustering(number_of_groups=self.k, display=False)

        # predict the clusters for each group
        self.predict_cluster(display=False)

        # calculate the error for each value of k
        self.calculate_cluster_error(number_of_groups=self.k, display=False)

        # create a matrix where each point shows its cluster group
        self.print_data_matrix(display=False)
        
        # count how many points are in each group
        self.count_cluster_repetitions(display=False)
        
        # sort the point cluster matrix (filter)
        self.get_smaller_values_from_cluster(number_of_values=15, display=False)

        # calculate the distance from origin (robot position) for each centroids obtained
        self.distances = self.calculate_distance_to_zero(coordinates=self.centroids, name_of_coordinates='Clusters', display=False)
        
        # reorder the cluster matrix but with each distance from origin and sorted (filter)
        self.selected_points_with_distances_sorted(display=False)

        # calculate an average of the distances of all reading points 
        distances_average =np.mean(self.distances)
        
        # select just the points with distance from origin lower than the average calculated before
        self.selected_points_with_distances_sorted_filtered(self.sorted_matrix_with_coord_dist, column=4, max_value=distances_average, display=False)

        # eliminate points that are more far than the specified values (filter)
        self.filter_coordinates(max_legs_distance=0.72, min_leg_distance=0.62, display=False)

        # eliminate points that are too close and get the result
        self.legs_coordinates_with_no_reps = self.verify_close_points(self.array_final, threshold=0.3, display=False)

        # calculate the distance from origin to the remain points filtered   
        self.leg_distances = self.calculate_distance_to_zero(coordinates=self.legs_coordinates_with_no_reps, name_of_coordinates= 'Table Leg Distances', display=False)
        
        # permutate and search for the exactly combination of square sides and diagonals
        self.table_square = self.find_square(points=self.legs_coordinates_with_no_reps, max_legs_side_distance=0.75, min_leg_side_distance=0.60, max_diagonal_distance=1.20, min_diagonal_distance=0.90)

        if len(self.table_square) > 0:
            # print('Square Coordinates Posible:', len(self.table_square))

            # select the first result from the posible combinations results
            self.selected_table_square = np.array(self.table_square[0])
            # print("Square Coordinate Selected:\n", self.selected_table_square)

            # calculate the distancia from origin to each result legs point
            verified_table_distance_from_zero= self.calculate_distance_to_zero(coordinates=self.selected_table_square, name_of_coordinates='Verified Table Leg Distance', display=False)
            
            # sort the legs point for identify the front and back table legs
            sorted_verified_coordinates = np.column_stack((self.selected_table_square, verified_table_distance_from_zero))
            
            # sort the obtained matrix 
            sorted_indices = np.argsort(sorted_verified_coordinates[:, -1])
            sorted_table_legs_with_distance = sorted_verified_coordinates[sorted_indices]

            # Imprimir la matriz ordenada
            # print("Square Coordinate Selected Distance Sorted:\n", sorted_table_legs_with_distance)

            # calculate all the points required for define an underneath the table path
            self.leg_middle_point = self.calculate_front_legs_center_point(leg_coordinates=sorted_table_legs_with_distance, display=False)
            self.table_center_point = self.calculate_table_center_point(leg_coordinates=sorted_table_legs_with_distance, display=False)
            self.approach_point = self.calculate_approach_point(leg_middle_point=self.leg_middle_point, table_center_point=self.table_center_point, approach_distance=0.5, display=False)
            self.approach_path = self.create_approach_path(approach_point=self.approach_point, leg_middle_point=self.leg_middle_point, table_center_point=self.table_center_point, display=False)
            
            # Publish Table Legs Transform
            self.publish_table_transform(frame='leg_1', x_coordinate=sorted_table_legs_with_distance[0,0], y_coordinate=sorted_table_legs_with_distance[0,1])
            self.publish_table_transform(frame='leg_2', x_coordinate=sorted_table_legs_with_distance[1,0], y_coordinate=sorted_table_legs_with_distance[1,1])
            self.publish_table_transform(frame='leg_3', x_coordinate=sorted_table_legs_with_distance[2,0], y_coordinate=sorted_table_legs_with_distance[2,1])
            self.publish_table_transform(frame='leg_4', x_coordinate=sorted_table_legs_with_distance[3,0], y_coordinate=sorted_table_legs_with_distance[3,1])

            # Publish Table Approach Path
            self.publish_table_transform(frame='table_center', x_coordinate=self.table_center_point[0], y_coordinate=self.table_center_point[1])
            self.publish_table_transform(frame='table_middle', x_coordinate=self.leg_middle_point[0], y_coordinate=self.leg_middle_point[1])
            self.publish_table_transform(frame='approach_distance', x_coordinate=self.approach_point[0], y_coordinate=self.approach_point[1])

            # plot graph to visualize data
            #self.plot_data()

            # inform table found 
            print('Trash Table Detected')

        else:

            # inform table not found
            print('Trash Table NOT Detected')

    
    def plot_data(self):

        fig, axs = plt.subplots(2, 3, figsize=(10, 5)) 
        
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
        axs[0,1].spines['bottom'].set_color('white')  
        axs[0,1].spines['left'].set_color('white')
        axs[0,1].tick_params(axis='x', colors='white', labelcolor='white') 
        axs[0,1].tick_params(axis='y', colors='white', labelcolor='white') 
        # Agregar notas o anotaciones
        axs[0,1].text(-3, 1, 'Nota', fontsize=12)

        axs[0,2].scatter(self.legs_coordinates_with_no_reps[:,0], self.legs_coordinates_with_no_reps[:,1], c='red', marker='s',label='Table Legs')
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
        axs[0,2].spines['bottom'].set_color('white')  
        axs[0,2].spines['left'].set_color('white')
        axs[0,2].tick_params(axis='x', colors='white', labelcolor='white') 
        axs[0,2].tick_params(axis='y', colors='white', labelcolor='white') 

        axs[1,0].scatter(self.krango, self.sse, c='red', marker='o', label='K Value')
        axs[1,0].plot(self.krango, self.sse, c='blue')
        axs[1,0].legend()
        axs[1,0].set_title('Sum of Square Error', color='white')
        axs[1,0].set_xlabel('K Value', color='white')
        axs[1,0].set_ylabel('Error', color='white')
        axs[1,0].set_facecolor('black')
        axs[1,0].grid(False)
        axs[1,0].locator_params(axis='x', nbins=15) 
        axs[1,0].spines['bottom'].set_color('white')  
        axs[1,0].spines['left'].set_color('white')
        axs[1,0].tick_params(axis='x', colors='white', labelcolor='white') 
        axs[1,0].tick_params(axis='y', colors='white', labelcolor='white') 

        axs[1,1].stem(self.list_of_cluster_values, self.frecuencies, linefmt='b', markerfmt='ro')
        axs[1,1].set_title('Group Cluster Repetitions', color='white')
        axs[1,1].set_xlabel('Cluster Group', color='white')
        axs[1,1].set_ylabel('Cluster Repetition', color='white')
        axs[1,1].set_xlim(-1, len(self.list_of_cluster_values))
        axs[1,1].set_ylim(0, 50)
        axs[1,1].set_facecolor('black')
        axs[1,1].grid(False)
        axs[1,0].locator_params(axis='x', nbins=len(self.list_of_cluster_values)) 
        axs[1,1].spines['bottom'].set_color('white')  
        axs[1,1].spines['left'].set_color('white')
        axs[1,1].tick_params(axis='x', colors='white', labelcolor='white') 
        axs[1,1].tick_params(axis='y', colors='white', labelcolor='white') 

        axs[1,2].set_title('Underneath Table Path', color='white')
        axs[1,2].scatter(self.selected_table_square[:,0], self.selected_table_square[:,1], c='green', marker='s')
        axs[1,2].scatter(self.table_center_point[0], self.table_center_point[1], c='red', marker='o', label='Table Center Point')
        axs[1,2].scatter(self.leg_middle_point[0], self.leg_middle_point[1], c='orange', marker='o', label='Front Legs Mid Point')
        axs[1,2].scatter(self.approach_point[0], self.approach_point[1], c='yellow', marker='o', label='Approach Point')
        axs[1,2].scatter(0, 0, c='purple', marker='^', s=75)
        axs[1,2].plot(self.approach_path[:,0], self.approach_path[:,1], c='blue')
        axs[1,2].legend()
        axs[1,2].set_xlim(-3, 3)
        axs[1,2].set_ylim(-0.5, 5)
        axs[1,2].set_xlabel('X Coordinates', color='white')
        axs[1,2].set_ylabel('Y Coordinates', color='white')
        axs[1,2].set_facecolor('black')
        axs[1,2].grid(False)
        axs[1,2].locator_params(axis='x', nbins=10)  
        axs[1,2].locator_params(axis='y', nbins=10) 
        axs[1,2].spines['bottom'].set_color('white')  
        axs[1,2].spines['left'].set_color('white')
        axs[1,2].tick_params(axis='x', colors='white', labelcolor='white') 
        axs[1,2].tick_params(axis='y', colors='white', labelcolor='white') 

        # Show the plots
        plt.show()
      
    def clustering(self, number_of_groups, display):
        self.kmeans = KMeans(n_clusters=number_of_groups).fit(self.data)
        self.centroids = self.kmeans.cluster_centers_
        if display:
            print('\nCentroids\n', self.centroids)
            print('Data lenght: %i' % len(self.centroids))
    
    def predict_cluster(self, display):
        self.clust = self.kmeans.predict(self.data)
        if display:
            print('\nCluster predicted\n')
            print(self.clust)
            print('Data lenght: %i' % len(self.clust))
    
    def calculate_cluster_error(self, number_of_groups, display):
        self.krango = range(1,number_of_groups)
        self.sse=[]
        for k in self.krango:
            kmeans = KMeans(n_clusters=k).fit(self.data)
            self.sse.append(kmeans.inertia_)
        if display:
            print('\nCluster errors for K = %i to K = 0\n' % number_of_groups)
            print(self.sse)
    
    def print_data_matrix(self, display): 
        self.data_with_cluster = np.column_stack((self.y_coordinates, self.x_coordinates, self.clust))
        if display:
            print('\nLaser Data with Cluster\n')
            print(self.data_with_cluster)

    def count_cluster_repetitions(self, display):
        self.frecuencies = np.bincount(self.clust)
        self.cluster_values_for_plot = range(len(self.frecuencies))
        self.list_of_cluster_values = list(self.cluster_values_for_plot)
        self.cluster_w_repetitions_num = []
        
        for numero, frecuencia in enumerate(self.frecuencies):
            if frecuencia > 0 and frecuencia < 40:
                self.cluster_w_repetitions_num.append([numero, frecuencia])
        
        if display:
            print('\nCluster repetitions\n', self.frecuencies)
            print('\nMatrix with cluster reps\n', self.cluster_w_repetitions_num)

    def get_smaller_values_from_cluster(self, number_of_values, display):
        data_matrix_sorted = sorted(self.cluster_w_repetitions_num, key=lambda x: x[1], reverse=True)
        filtered_sorted_values = data_matrix_sorted[:number_of_values]
        shaped_matrix = np.array(filtered_sorted_values)
        self.matrix_filtered_less = shaped_matrix.reshape(-1, 2)
        if display:
            print('\nFiltered values\n', self.matrix_filtered_less)
    
    def calculate_distance_to_zero(self, coordinates, name_of_coordinates, display):
        distances = np.linalg.norm(coordinates, axis=1)
        distances = distances.reshape(-1, 1)
        if display:
            print('\nDistance from zero for selected points: ' + name_of_coordinates + '\n', distances)
        return distances
    
    def selected_points_with_distances_sorted(self, display):
        
        if display:
            print("Dimensiones de la primera matriz:", self.matrix_filtered_less.shape)
            print("Dimensiones de la segunda matriz:", self.centroids.shape)
            print("Dimensiones de la tercera matriz:", self.distances.shape)
        #matrix = np.column_stack((self.matrix_filtered_less, self.centroids, self.distances))

        num_rows_matrix_filtered_less = self.matrix_filtered_less.shape[0]
        centroids_selected = self.centroids[:num_rows_matrix_filtered_less]
        distances_selected = self.distances[:num_rows_matrix_filtered_less]
        matrix = np.column_stack((self.matrix_filtered_less, centroids_selected, distances_selected))
        
        sorted_indices = np.argsort(matrix[:, 4])
        self.sorted_matrix_with_coord_dist = matrix[sorted_indices]
        if display:
            print('\n Selected points with distances from 0.0\n')
            print('\n Group\t\t Reps\t X Coordinate\t Y Coordinate\t Distance\n', self.sorted_matrix_with_coord_dist)
    
    def selected_points_with_distances_sorted_filtered(self, matrix, column, max_value, display):
        self.datos_filtrados = matrix[matrix[:, column] < max_value]
        if display:
            print('Selected points with distances from 0.0 filtered\n', self.datos_filtrados)
    
    def euclidean_distances(self, point1, point2):
        return np.linalg.norm(point1 - point2)

    def filter_coordinates(self, max_legs_distance, min_leg_distance, display):
        coordinates = np.column_stack((self.datos_filtrados[:,2], self.datos_filtrados[:,3]))
        if display:
            print('\ncoordinates pre filtradas \n', coordinates)
        filtered_coordinates = []
        for point in coordinates:
            new_coordinate = True
            minor_distances = 0
            for filtered_point in filtered_coordinates:
                dist = self.euclidean_distances(point, filtered_point)
                if dist > max_legs_distance:
                    minor_distances += 1
                if dist < min_leg_distance:
                    minor_distances += 1
            if minor_distances > 2:
                new_coordinate = False
            if new_coordinate:
                filtered_coordinates.append(point)
        self.array_final = np.vstack(filtered_coordinates)
        if display:
            print('\ncoordinates filtradas\n', self.array_final)

    def verify_close_points(self, coordinates, threshold, display):
        new_points = []
        close_points = set()
        for i in range(len(coordinates)):
            valid_point = True
            for j in range(i + 1, len(coordinates)):
                distance = self.euclidean_distances(coordinates[i], coordinates[j])
                if distance < threshold:
                    close_points.add(i)
                    close_points.add(j)
                    valid_point = False
            if valid_point:
                new_points.append(coordinates[i])
        new_points = np.vstack(new_points)
        if display:
            print('\ncoordinates filtradas sin reps\n', new_points)
        return np.array(new_points)

    def find_square(self, points, max_legs_side_distance, min_leg_side_distance, max_diagonal_distance, min_diagonal_distance):
        possible_squares = []

        # Generate all possible permutations of points
        point_permutations = permutations(points, 4)  # use only permutations of length 4

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
            if all(min_leg_side_distance <= distance <= max_legs_side_distance for distance in sides) and \
            all(min_diagonal_distance <= diagonal <= max_diagonal_distance for diagonal in diagonals):
                possible_squares.append(combination)

        return possible_squares
    

    def calculate_front_legs_center_point(self, leg_coordinates, display):
        middle_point = [(leg_coordinates[0,0] + leg_coordinates[1,0]) / 2, (leg_coordinates[0,1] + leg_coordinates[1,1]) / 2]
        if display:
            print('\nCalculated Middle Point: ', middle_point)
        return middle_point
    
    def calculate_table_center_point(self, leg_coordinates, display):
        sum_x = sum(point[0] for point in leg_coordinates)
        sum_y = sum(point[1] for point in leg_coordinates)
        
        center_x = sum_x / len(leg_coordinates)
        center_y = sum_y / len(leg_coordinates)
        
        square_center = [center_x, center_y]
        if display:
            print('\nCalculated Square Center:', square_center)
        return square_center
    
    import numpy as np

    def calculate_approach_point(self, leg_middle_point, table_center_point, approach_distance, display):
        leg_middle_point = np.array(leg_middle_point)
        table_center_point = np.array(table_center_point)
        vector = table_center_point - leg_middle_point
        direction = vector / np.linalg.norm(vector) 
        new_vector = approach_distance * direction * -1
        approach_point = leg_middle_point + new_vector

        if display:
            print('Approach Point:', approach_point)
        return approach_point

    
    def create_approach_path(self, approach_point, leg_middle_point, table_center_point, display):
        approach_path = np.array([
            [0,0],
            [approach_point[0], approach_point[1]],
            [leg_middle_point[0], leg_middle_point[1]],
            [table_center_point[0], table_center_point[1]]
        ])

        if display:
            print('Approach Path:\n', approach_path)
        return approach_path

    def publish_table_transform(self, frame, x_coordinate, y_coordinate):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'robot_front_laser_base_link'
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

    trash_table_detection = TrashTableDetection()

    rclpy.spin(trash_table_detection)

    trash_table_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()