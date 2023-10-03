
# from PIL import Image

# data = np.load(r'C:\Users\puran\Downloads\assignment1_student-1\assignment1_student\collision_checking_polygons.npy',
# allow_pickle=True)

# print(data)
import numpy as np
import random
import math
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt


def generate_convex_polygon(P, N_min, N_max, r_min, r_max):
    polygons = []

    for _ in range(P):
        x_center, y_center = random.uniform(-100, 100), random.uniform(-100, 100)  # Random center of the polygon
        N = random.randint(N_min, N_max)  # Number of vertices

        points = []
        for _ in range(N):
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
    # Convert list of polygons to a list of np arrays
    np_polygons = [np.array(polygon) for polygon in polygons]
    # Combine all polygons into a single numpy array and save as .npy
    np.save(filename, np.array(np_polygons, dtype=object))


def load_polygons_from_file(filename):
    return np.load(filename, allow_pickle=True)


def plot_polygons(polygons):
    plt.figure(figsize=(8, 8))  # set the size of the figure, 1 unit will be represented by 400 pixels
    plt.axis('equal')  # to keep the aspect ratio of the polygons

    for polygon in polygons:
        polygon = np.concatenate((polygon, [polygon[0]]), axis=0)  # close the polygon
        xs, ys = zip(*polygon)
        plt.plot(xs, ys)

    plt.show()


def main():
    P = int(input("Enter the total number of polygons in the scene (P): "))
    N_min = int(input("Enter the minimum number of vertices (N_min): "))
    N_max = int(input("Enter the maximum number of vertices (N_max): "))
    r_min = float(input("Enter the minimum radius of the polygon (r_min): "))
    r_max = float(input("Enter the maximum radius of the polygon (r_max): "))
    filename = input("Enter the filename to save/load polygons (without extension): ") + '.npy'  # Note the file extension change

    polygons = generate_convex_polygon(P, N_min, N_max, r_min, r_max)
    save_polygons_to_file(polygons, filename)

    loaded_polygons = load_polygons_from_file(filename)

    for polygon in loaded_polygons:
        # print("Generated Polygon:", polygon)
        print(repr(polygon), end=' ')
    print()
    plot_polygons(loaded_polygons)


if __name__ == "__main__":
    main()


# Sample test case
# Total number of polygons in the scene P: 3
# Minimum number of vertices N_min: 3
# Maximum number of vertices N_max: 6
# Minimum radius of the polygon r_min: 5.0
# Maximum radius of the polygon r_max: 10.0
# Filename to save/load polygons: polygons.npy
