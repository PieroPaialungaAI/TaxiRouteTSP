from IPython.display import display
import json
import random
import folium
import numpy as np
from scipy.spatial.distance import cdist
import elkai
from itertools import permutations

class TaxiDriverAI:

    def __init__(self, route_box_file = "route_boundaries/route_box.json",
                 map_folder = "map_folder/"):
        self.city_box = self.load_city_box(route_box_file)
        self.locations = []
        self.distance_matrix = None
        self.optimal_tour = None
        self.map_folder = map_folder


    def load_city_box(self,route_box_file):
        with open(route_box_file, "r") as file:
            bounding_box = json.load(file)
        return bounding_box


    def generate_random_points(self, num_points=10):
        points = []
        for _ in range(num_points):
            lat = random.uniform(self.city_box['min_lat'], self.city_box['max_lat'])
            lon = random.uniform(self.city_box['min_lon'], self.city_box['max_lon'])
            points.append((lat, lon))
        self.locations = points
        return points


    def compute_distances(self):
        points_array = np.array(self.locations)
        self.distance_matrix = cdist(points_array, points_array, metric='euclidean')
        return self.distance_matrix

    def find_shortest_path_brute_force(self):
        if self.distance_matrix is None:
            self.compute_distances()

        # Generate all possible permutations of the cities
        num_cities = len(self.locations)
        all_permutations = permutations(range(num_cities))

        shortest_distance = float('inf')
        best_tour = None

        # Evaluate each permutation
        for perm in all_permutations:
            # Calculate the total distance for this tour
            total_distance = sum(
                self.distance_matrix[perm[i], perm[i + 1]] for i in range(num_cities - 1)
            )
            # Add the distance back to the starting point
            total_distance += self.distance_matrix[perm[-1], perm[0]]

            # Update the best tour if this one is shorter
            if total_distance < shortest_distance:
                shortest_distance = total_distance
                best_tour = perm

        self.optimal_tour = np.array(list(best_tour))
        return self.optimal_tour


    def find_shortest_path(self):
        n = len(self.locations)
        points_array = np.array(self.locations)
        self.loc_dict = {i: self.locations[i] for i in range(len(self.locations))}
        cities = elkai.Coordinates2D(self.loc_dict)
        optimal_tour = cities.solve_tsp()
        self.optimal_tour = optimal_tour
        return optimal_tour  # Return the order of cities in the optimized tour

    def display_optimized_route(self, optimized_route=None, save_file='optimized_map.html'):
        if optimized_route is None:
            optimized_route = self.optimal_tour
        first_point = self.locations[optimized_route[0]]
        city_map = folium.Map(location=first_point, zoom_start=12)

        # Plot the optimized route
        for i in range(len(optimized_route)):
            start = self.locations[optimized_route[i]]
            end = self.locations[optimized_route[(i + 1) % len(optimized_route)]]
            folium.PolyLine([start, end], color='blue', weight=2.5, opacity=1).add_to(city_map)

        # Add numbered markers for each point
        for idx, point_index in enumerate(optimized_route):
            point = self.locations[point_index]
            folium.Marker(
                location=point,
                popup=f"Stop {idx + 1}",
                icon=folium.Icon(color='blue', icon='info-sign')
            ).add_to(city_map)

        # Save the map
        city_map.save(self.map_folder + save_file)
        print(f"Optimized map saved as {save_file}.")
        return city_map



    def static_plot(self, save_file='random_points_map.html'):
        if not self.locations:
            print("No locations to plot. Generate random points first.")
            return None

        # Initialize the map centered on the first point
        city_map = folium.Map(location=[self.city_box['min_lat'], self.city_box['min_lon']], zoom_start=12)

        # Add pin-style markers for each point
        for idx, point in enumerate(self.locations):
            folium.Marker(
                location=point,
                popup=f"Point {idx + 1}",
                icon=folium.Icon(color='blue', icon='info-sign')  # Use a pin-like icon
            ).add_to(city_map)

        # Save and display map
        city_map.save(self.map_folder + save_file)
        print(f"Random points map saved as {save_file}.")
        return city_map
    

    def plot_with_connections(self, save_file='random_connections_map.html'):
        """Displays the random points on a map with pin-style markers and random connections."""
        if not self.locations:
            print("No locations to plot. Generate random points first.")
            return None

        # Initialize the map centered on the bounding box
        city_map = folium.Map(location=[self.city_box['min_lat'], self.city_box['min_lon']], zoom_start=12)

        # Add pin-style markers for each point
        for idx, point in enumerate(self.locations):
            folium.Marker(
                location=point,
                popup=f"Point {idx + 1}",
                icon=folium.Icon(color='blue', icon='info-sign')  # Pin-like icon
            ).add_to(city_map)

        # Generate random connections
        num_points = len(self.locations)
        random_order = list(range(num_points))
        random.shuffle(random_order)  # Randomize the order of connections

        # Draw connections as red lines
        for i in range(num_points):
            start = self.locations[random_order[i]]
            end = self.locations[random_order[(i + 1) % num_points]]  # Wrap around to connect the last point to the first
            folium.PolyLine([start, end], color='red', weight=2.5, opacity=1).add_to(city_map)

        # Save and display map
        city_map.save(self.map_folder + save_file)
        print(f"Random points with connections map saved as {save_file}.")
        return city_map
