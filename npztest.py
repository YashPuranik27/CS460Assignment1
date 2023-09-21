# this is a python script to test the npz file
from PIL import Image
import numpy as np

data = np.load(r'C:\Users\puran\PycharmProjects\CS460Assignment1\test.npz',
allow_pickle=True)

print(data)
