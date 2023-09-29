# Collision checking between convex polygons

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Define two polygons
polygon1 = np.array([[2, 3], [3, 8], [5, 6], [6, 4], [4, 2]])
polygon2 = np.array([[5, 5], [7, 8], [9, 6], [8, 4]])

# Calculate bounding boxes
min1, max1 = np.min(polygon1, axis=0), np.max(polygon1, axis=0)
min2, max2 = np.min(polygon2, axis=0), np.max(polygon2, axis=0)

# Check if bounding boxes collide
collide_x = min1[0] < max2[0] and max1[0] > min2[0]
collide_y = min1[1] < max2[1] and max1[1] > min2[1]
collide = collide_x and collide_y

# Visualization
fig, ax = plt.subplots()

# Draw polygons
ax.fill(polygon1[:, 0], polygon1[:, 1], 'b', label='Polygon 1', alpha=0.5)
ax.fill(polygon2[:, 0], polygon2[:, 1], 'r', label='Polygon 2', alpha=0.5)

# Draw bounding boxes
rect1 = patches.Rectangle(min1, max1[0] - min1[0], max1[1] - min1[1], linewidth=1, edgecolor='b', facecolor='none')
rect2 = patches.Rectangle(min2, max2[0] - min2[0], max2[1] - min2[1], linewidth=1, edgecolor='r', facecolor='none')

ax.add_patch(rect1)
ax.add_patch(rect2)

ax.legend()

if collide:
    ax.set_title("Bounding boxes collide")
else:
    ax.set_title("Bounding boxes don't collide")
plt.show()


