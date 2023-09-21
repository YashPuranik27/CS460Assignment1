# this is a python script to test the npz file

import numpy as np

filename = r'C:\Users\puran\PycharmProjects\CS460Assignment1\polytest.npz'
data = np.load(filename, allow_pickle=True)

print("Keys in the file:", data.files)  # To list out all the keys in the npz file

polygons = []

for key in data.files:
    polygon = data[key]
    polygons.append(polygon)
    print(repr(polygon))  # Printing each polygon