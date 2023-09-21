import numpy as np
import random
import math
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
# from PIL import Image

# data = np.load(r'C:\Users\puran\Downloads\assignment1_student-1\assignment1_student\collision_checking_polygons.npy',
# allow_pickle=True)

# print(data)


# function to generate a complex polygon we could create a list, and it can choose a random polygon center we could
# have a for loop, and we could use math.random or something of the sort to generate a random number of vertices we
# could use a for loop to generate a random radius for each vertex

def generate_convex_polygon(P, N_min, N_max, r_min, r_max):
    polygons = []

    for _ in range(P):
        x_center, y_center = random.uniform(-100, 100), random.uniform(-100, 100)  # Random center of the polygon
        N = random.randint(N_min, N_max)  # Number of vertices

        points = []
        for _ in range(N): # use _ cause we don't care about the values returned
            alpha_n = math.radians(random.uniform(0, 360))  # Angle in radians
            r = random.uniform(0, r_max - r_min) + r_min
            x = x_center + r * math.cos(alpha_n)
            y = y_center + r * math.sin(alpha_n)
            points.append([x, y])

        hull = ConvexHull(points)
        convex_polygon = [points[i] for i in hull.vertices]
        polygons.append(convex_polygon)

    return polygons


def save_polygons_to_file(polygons, filename):
    np.save(filename, polygons)


def load_polygons_from_file(filename):
    return np.load(filename, allow_pickle=True).tolist()


# add a function to plot the polygons
def plot_polygons(polygons):
    plt.figure(figsize=(10, 10))
    for polygon in polygons:
        polygon.append(polygon[0])  # close the polygon
        xs, ys = zip(*polygon)
        plt.plot(xs, ys)
    plt.show()


def main():

    # Asks for inputs from the user for the inputs highlighted in the assignment
    P = int(input("Enter the total number of polygons in the scene (P): "))
    N_min = int(input("Enter the minimum number of vertices (N_min): "))
    N_max = int(input("Enter the maximum number of vertices (N_max): "))
    r_min = float(input("Enter the minimum radius of the polygon (r_min): "))
    r_max = float(input("Enter the maximum radius of the polygon (r_max): "))
    filename = input("Enter the filename to save/load polygons (without extension): ") + '.npy'

    # save polygons to file. Should refer to def save_polygons_to_file(polygons, filename)
    polygons = generate_convex_polygon(P, N_min, N_max, r_min, r_max)
    save_polygons_to_file(polygons, filename)
    # load polygons from file. Should refer to def load_polygons_from_file(filename)

    loaded_polygons = load_polygons_from_file(filename)
    plot_polygons(loaded_polygons)


if __name__ == "__main__":
    main()
