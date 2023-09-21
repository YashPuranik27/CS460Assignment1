import numpy as np
# from PIL import Image

# data = np.load(r'C:\Users\puran\Downloads\assignment1_student-1\assignment1_student\collision_checking_polygons.npy',
# allow_pickle=True)

# print(data)

import random
import math

from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt


def generate_convex_polygon(P, N_min, N_max, r_min, r_max):
    polygons = []


def save_polygons_to_file(polygons, filename):
    np.save(filename, polygons)


def load_polygons_from_file(filename):
    return np.load(filename, allow_pickle=True).tolist()

# add a function to plot the polygons


def main():
    P = int(input("Enter the total number of polygons in the scene (P): "))
    N_min = int(input("Enter the minimum number of vertices (N_min): "))
    N_max = int(input("Enter the maximum number of vertices (N_max): "))
    r_min = float(input("Enter the minimum radius of the polygon (r_min): "))
    r_max = float(input("Enter the maximum radius of the polygon (r_max): "))
    filename = input("Enter the filename to save/load polygons (without extension): ") + '.npy'


    save_polygons_to_file(polygons, filename)

    loaded_polygons = load_polygons_from_file(filename)




if __name__ == "__main__":
    main()
