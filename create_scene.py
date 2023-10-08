
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
        #x_center, y_center = random.uniform(-100, 100), random.uniform(-100, 100)  # Random center of the polygon
        x_center, y_center = random.uniform(0, 2), random.uniform(0, 2)  # Random center of the polygon
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

    #plt.show()


def plot_name_initials(polygons):
        plt.figure(figsize=(10, 10))
        plt.axis('equal')

        plt.title('name initials')

        for polygon in polygons:
            polygon = np.concatenate((polygon, [polygon[0]]), axis=0)  # close the polygon
            xs, ys = zip(*polygon)
            plt.plot(xs, ys)

        plt.show()


def construct_name_inital_polygons():

    #### letter Y
    name_polygons = []
    name_poly=[]
    name_poly.append([4.09, 14.3])
    name_poly.append([2.37, 13.14])
    name_poly.append([6.71, 6.78])
    name_poly.append([8.37, 8.46])
    name_polygons.append(name_poly)

    name_poly=[]
    name_poly.append([13.25, 13.78])
    name_poly.append([15.15, 12.26])
    name_poly.append([9.67, 6.62])
    name_poly.append([8.37, 8.46])
    name_polygons.append(name_poly)

    name_poly=[]
    name_poly.append([6.71, 6.78])
    name_poly.append([8.37, 8.46])
    name_poly.append([9.67, 6.62])
    name_poly.append([9.53, 0])
    name_poly.append([6.55, 0])
    name_polygons.append(name_poly)

    ### letter A
    name_poly = []
    name_poly.append([18.53, 6.96])
    name_poly.append([21.08, 7.02])
    name_poly.append([21.62, 5.04])
    name_poly.append([17.85, 5.09])
    name_polygons.append(name_poly)

    name_poly = []
    name_poly.append([18.55, 13.62])
    name_poly.append([19.91, 13.62])
    name_poly.append([20.0, 11.0])
    name_poly.append([16.0, 0.0])
    name_poly.append([13.73, 0.0])
    name_polygons.append(name_poly)

    name_poly = []
    name_poly.append([25.0, 0.0])
    name_poly.append([23.0, 0.0])
    name_poly.append([20.0, 11.0])
    name_poly.append([19.91, 13.62])
    name_poly.append([21.49, 13.68])

    name_polygons.append(name_poly)


    save_polygons_to_file(name_polygons, "name_initials.npy")



def main():

    P = int(input("Enter the total number of polygons in the scene (P): "))
    N_min = int(input("Enter the minimum number of vertices (N_min): "))
    N_max = int(input("Enter the maximum number of vertices (N_max): "))
    r_min = float(input("Enter the minimum radius of the polygon (r_min): "))
    r_max = float(input("Enter the maximum radius of the polygon (r_max): "))
    filename = input("Enter the filename to save/load polygons (without extension): ") + '.npy'  # Note the file extension change

    polygons = generate_convex_polygon(P, N_min, N_max, r_min, r_max)
    print("...... polygons", polygons)
    save_polygons_to_file(polygons, filename)

    loaded_polygons = load_polygons_from_file(filename)

    for polygon in loaded_polygons:
        # print("Generated Polygon:", polygon)
        print(repr(polygon), end=' ')
    print()
    plot_polygons(loaded_polygons)


    #visualization of the name_initial letters
    construct_name_inital_polygons()

    name_initial_polygons = load_polygons_from_file("name_initials.npy")
    plot_name_initials(name_initial_polygons)

    plt.show()


if __name__ == "__main__":
    main()


# Sample test case
# Total number of polygons in the scene P: 3
# Minimum number of vertices N_min: 3
# Maximum number of vertices N_max: 6
# Minimum radius of the polygon r_min: 5.0
# Maximum radius of the polygon r_max: 10.0
# Filename to save/load polygons: polygons.npy
